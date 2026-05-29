#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
double parseScalar(const std::string & token)
{
  std::string s = token;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (s.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (s == "nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (s == "inf" || s == "+inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (s == "-inf") {
    return -std::numeric_limits<double>::infinity();
  }
  return std::stod(token);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    cells.push_back(cell);
  }
  return cells;
}

std::string expandUser(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}
}  // namespace

class CsvPlayer : public rclcpp::Node
{
public:
  CsvPlayer()
  : Node("csv_player")
  {
    csv_path_ = expandUser(declare_parameter<std::string>(
      "csv_path", ""));
    start_offset_sec_ = declare_parameter<double>("start_offset_sec", 0.0);
    playback_rate_ = declare_parameter<double>("playback_rate", 1.0);
    sample_hz_ = std::max(1e-6, declare_parameter<double>("sample_hz", 50.0));
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/data_logging_msg");
    declare_parameter<double>("seek_time_sec", 0.0);
    declare_parameter<double>("seek_ratio", 0.0);
    declare_parameter<std::string>("status_topic", "/csv_player/status");

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);
    status_topic_ = get_parameter("status_topic").as_string();
    status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(status_topic_, 10);
    parameterCallbackHandle_ = add_on_set_parameters_callback(
      std::bind(&CsvPlayer::onParametersSet, this, std::placeholders::_1));

    playThread_ = std::thread(&CsvPlayer::playLoop, this);
  }

  ~CsvPlayer() override
  {
    stopRequested_ = true;
    controlCv_.notify_all();
    if (playThread_.joinable()) {
      playThread_.join();
    }
  }

private:
  struct Row
  {
    double t_sec{0.0};
    std::vector<double> packed;
  };

  void loadCsv(const std::string & csv_path, double start_offset_sec)
  {
    if (csv_path.empty()) {
      std::lock_guard<std::mutex> lock(stateMutex_);
      rows_.clear();
      currentIndex_ = 0;
      currentTimeSec_ = 0.0;
      totalDurationSec_ = 0.0;
      loadedCsvPath_.clear();
      publishStatusLocked();
      RCLCPP_INFO(get_logger(), "No CSV selected yet. Waiting for UI input.");
      return;
    }

    std::ifstream file(csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open csv: %s", csv_path.c_str());
      std::lock_guard<std::mutex> lock(stateMutex_);
      rows_.clear();
      currentIndex_ = 0;
      currentTimeSec_ = 0.0;
      totalDurationSec_ = 0.0;
      loadedCsvPath_ = csv_path;
      publishStatusLocked();
      return;
    }

    std::string headerLine;
    if (!std::getline(file, headerLine)) {
      RCLCPP_ERROR(get_logger(), "CSV is empty: %s", csv_path.c_str());
      std::lock_guard<std::mutex> lock(stateMutex_);
      rows_.clear();
      currentIndex_ = 0;
      currentTimeSec_ = 0.0;
      totalDurationSec_ = 0.0;
      loadedCsvPath_ = csv_path;
      publishStatusLocked();
      return;
    }

    std::unordered_map<std::string, size_t> nextHeaderIndex;
    const auto headers = splitCsvLine(headerLine);
    for (size_t i = 0; i < headers.size(); ++i) {
      nextHeaderIndex[headers[i]] = i;
    }

    headerIndex_ = nextHeaderIndex;

    const bool hasTimeColumn = headerIndex_.count("t_sec") > 0;

    std::string line;
    bool firstTimeSet = false;
    std::vector<Row> loadedRows;
    size_t sampleIndex = 0;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      const auto cells = splitCsvLine(line);
      if (cells.size() < headers.size()) {
        continue;
      }

      const double rawTime = hasTimeColumn ? getValue(cells, "t_sec") : (static_cast<double>(sampleIndex) / sample_hz_);
      ++sampleIndex;
      if (!std::isfinite(rawTime)) {
        continue;
      }
      if (!firstTimeSet) {
        firstTimeSec_ = rawTime;
        firstTimeSet = true;
      }

      const double relTime = rawTime - firstTimeSec_;
      if (relTime + 1e-9 < start_offset_sec) {
        continue;
      }

      Row row;
      row.t_sec = relTime - start_offset_sec;
      row.packed = packRow(cells);
      loadedRows.push_back(std::move(row));
    }

    {
      std::lock_guard<std::mutex> lock(stateMutex_);
      rows_ = std::move(loadedRows);
      currentIndex_ = 0;
      currentTimeSec_ = 0.0;
      totalDurationSec_ = rows_.empty() ? 0.0 : rows_.back().t_sec;
      loadedCsvPath_ = csv_path;
      anchorRowTimeSec_ = 0.0;
      anchorWallTime_ = std::chrono::steady_clock::now();
      publishStatusLocked();
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu CSV rows from %s", rows_.size(), csv_path.c_str());
  }

  double getValue(const std::vector<std::string> & cells, const std::string & key) const
  {
    const auto it = headerIndex_.find(key);
    if (it == headerIndex_.end() || it->second >= cells.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    try {
      return parseScalar(cells[it->second]);
    } catch (const std::exception &) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  std::vector<double> packRow(const std::vector<std::string> & cells) const
  {
    const bool is_debug_layout = headerIndex_.count("normalEst_x") > 0 && headerIndex_.count("fwCmd_x") > 0;
    if (is_debug_layout) {
      std::vector<double> out;
      out.reserve(cells.size());
      for (const auto & cell : cells) {
        try {
          out.push_back(parseScalar(cell));
        } catch (const std::exception &) {
          out.push_back(std::numeric_limits<double>::quiet_NaN());
        }
      }
      return out;
    }

    const bool is_new_layout = headerIndex_.count("motor_f1") > 0 || headerIndex_.count("gyro_x") > 0;
    if (is_new_layout) {
      std::vector<double> out(52, std::numeric_limits<double>::quiet_NaN());
      out[0] = getValue(cells, "pose_x");
      out[1] = getValue(cells, "pose_y");
      out[2] = getValue(cells, "pose_z");
      out[3] = getValue(cells, "pose_roll");
      out[4] = getValue(cells, "pose_pitch");
      out[5] = getValue(cells, "pose_yaw");
      out[6] = getValue(cells, "status_battery_voltage");
      out[7] = getValue(cells, "raw_battery_voltage");
      out[8] = getValue(cells, "filt_battery_voltage");
      if (!std::isfinite(out[7])) {
        out[7] = out[6];
      }
      if (!std::isfinite(out[8])) {
        out[8] = out[7];
      }
      out[9] = getValue(cells, "cmd_x");
      out[10] = getValue(cells, "cmd_y");
      out[11] = getValue(cells, "cmd_z");
      out[12] = getValue(cells, "cmd_yaw");
      out[13] = getValue(cells, "est_vx");
      out[14] = getValue(cells, "est_vy");
      out[15] = getValue(cells, "est_vz");
      out[16] = getValue(cells, "est_ax");
      out[17] = getValue(cells, "est_ay");
      out[18] = getValue(cells, "est_az");
      out[19] = getValue(cells, "gyro_x");
      out[20] = getValue(cells, "gyro_y");
      out[21] = getValue(cells, "gyro_z");
      out[22] = getValue(cells, "angAcc_x");
      out[23] = getValue(cells, "angAcc_y");
      out[24] = getValue(cells, "angAcc_z");
      out[25] = getValue(cells, "velDes_vx");
      out[26] = getValue(cells, "velDes_vy");
      out[27] = getValue(cells, "velDes_vz");
      out[28] = getValue(cells, "attDes_roll");
      out[29] = getValue(cells, "attDes_pitch");
      out[30] = getValue(cells, "attDes_yaw");
      out[31] = getValue(cells, "motor_f1");
      out[32] = getValue(cells, "motor_f2");
      out[33] = getValue(cells, "motor_f3");
      out[34] = getValue(cells, "motor_f4");
      out[35] = getValue(cells, "motor_f1_scaled");
      out[36] = getValue(cells, "motor_f2_scaled");
      out[37] = getValue(cells, "motor_f3_scaled");
      out[38] = getValue(cells, "motor_f4_scaled");
      out[39] = getValue(cells, "bodyInFx");
      out[40] = getValue(cells, "bodyInFy");
      out[41] = getValue(cells, "bodyInFz");
      out[42] = getValue(cells, "droneWorldFx");
      out[43] = getValue(cells, "droneWorldFy");
      out[44] = getValue(cells, "droneWorldFz");
      out[45] = getValue(cells, "droneWorldFx_scaled");
      out[46] = getValue(cells, "droneWorldFy_scaled");
      out[47] = getValue(cells, "droneWorldFz_scaled");
      out[48] = getValue(cells, "zero_bias_count");
      out[49] = getValue(cells, "rateDes_roll");
      out[50] = getValue(cells, "rateDes_pitch");
      out[51] = getValue(cells, "rateDes_yaw");
      return out;
    }

    std::vector<double> out(50, std::numeric_limits<double>::quiet_NaN());

    out[0] = getValue(cells, "pose_x");
    out[1] = getValue(cells, "pose_y");
    out[2] = getValue(cells, "pose_z");
    out[3] = getValue(cells, "pose_roll");
    out[4] = getValue(cells, "pose_pitch");
    out[5] = getValue(cells, "pose_yaw");
    out[6] = getValue(cells, "status_battery_voltage");
    out[7] = getValue(cells, "raw_battery_voltage");
    out[8] = getValue(cells, "filt_battery_voltage");
    if (!std::isfinite(out[7])) {
      out[7] = out[6];
    }
    if (!std::isfinite(out[8])) {
      out[8] = out[7];
    }
    out[9] = getValue(cells, "bodyInFx");
    out[10] = getValue(cells, "bodyInFy");
    out[11] = getValue(cells, "bodyInFz");
    out[12] = getValue(cells, "droneWorldFx");
    out[13] = getValue(cells, "droneWorldFy");
    out[14] = getValue(cells, "droneWorldFz");
    out[15] = getValue(cells, "droneWorldFx_scaled");
    out[16] = getValue(cells, "droneWorldFy_scaled");
    out[17] = getValue(cells, "droneWorldFz_scaled");
    out[18] = getValue(cells, "mobFx");
    out[19] = getValue(cells, "mobFy");
    out[20] = getValue(cells, "mobFz");
    out[21] = getValue(cells, "cmd_x");
    out[22] = getValue(cells, "cmd_y");
    out[23] = getValue(cells, "cmd_z");
    out[24] = getValue(cells, "cmd_yaw");
    out[25] = getValue(cells, "sp_x");
    out[26] = getValue(cells, "sp_y");
    out[27] = getValue(cells, "sp_z");
    out[28] = getValue(cells, "sp_yaw_sp");
    out[29] = getValue(cells, "est_vx");
    out[30] = getValue(cells, "est_vy");
    out[31] = getValue(cells, "est_vz");
    out[32] = getValue(cells, "est_ax");
    out[33] = getValue(cells, "est_ay");
    out[34] = getValue(cells, "est_az");
    out[35] = getValue(cells, "body_vx");
    out[36] = getValue(cells, "body_vy");
    out[37] = getValue(cells, "velDes_vx");
    out[38] = getValue(cells, "velDes_vy");
    out[39] = getValue(cells, "velDes_vz");
    out[40] = getValue(cells, "attDes_roll");
    out[41] = getValue(cells, "attDes_pitch");
    out[42] = getValue(cells, "attDes_yaw");
    out[43] = getValue(cells, "kal_qComp0");
    out[44] = getValue(cells, "kal_qComp1");
    out[45] = getValue(cells, "kal_qComp2");
    out[46] = getValue(cells, "kal_qComp3");
    out[47] = getValue(cells, "kal_stateD0");
    out[48] = getValue(cells, "kal_stateD1");
    out[49] = getValue(cells, "kal_stateD2");
    return out;
  }

  void playLoop()
  {
    requestReload(csv_path_, start_offset_sec_);

    while (rclcpp::ok() && !stopRequested_) {
      applyPendingControl();

      Row row;
      bool hasRow = false;
      bool finishedPlayback = false;
      {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!rows_.empty() && currentIndex_ < rows_.size()) {
          const double targetTimeSec =
            rows_[currentIndex_].t_sec - anchorRowTimeSec_;
          const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - anchorWallTime_).count();
          const double scaledElapsed = elapsed * std::max(1e-6, playback_rate_.load());
          if (scaledElapsed + 1e-4 >= targetTimeSec) {
            row = rows_[currentIndex_];
            currentTimeSec_ = row.t_sec;
            ++currentIndex_;
            publishStatusLocked();
            hasRow = true;
            if (currentIndex_ >= rows_.size()) {
              finishedPlayback = true;
            }
          }
        }
      }

      if (hasRow) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = std::move(row.packed);
        pub_->publish(msg);
        if (finishedPlayback) {
          RCLCPP_INFO(get_logger(), "CSV playback finished.");
        }
        continue;
      }

      std::unique_lock<std::mutex> lock(controlMutex_);
      controlCv_.wait_for(lock, std::chrono::milliseconds(5), [this]() {
        return stopRequested_ || reloadRequested_ || seekRequested_;
      });
    }
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string nextCsvPath = csv_path_;
    double nextStartOffsetSec = start_offset_sec_;
    bool shouldReload = false;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "csv_path") {
        nextCsvPath = expandUser(parameter.as_string());
        shouldReload = true;
      } else if (parameter.get_name() == "start_offset_sec") {
        nextStartOffsetSec = parameter.as_double();
        shouldReload = true;
      } else if (parameter.get_name() == "playback_rate") {
        const double nextRate = parameter.as_double();
        if (nextRate <= 0.0) {
          result.successful = false;
          result.reason = "playback_rate must be greater than zero";
          return result;
        }
        playback_rate_.store(nextRate);
      } else if (parameter.get_name() == "sample_hz") {
        const double nextSampleHz = parameter.as_double();
        if (nextSampleHz <= 0.0) {
          result.successful = false;
          result.reason = "sample_hz must be greater than zero";
          return result;
        }
        sample_hz_ = nextSampleHz;
        shouldReload = true;
      } else if (parameter.get_name() == "seek_time_sec") {
        requestSeekByTime(parameter.as_double());
      } else if (parameter.get_name() == "seek_ratio") {
        const double ratio = std::clamp(parameter.as_double(), 0.0, 1.0);
        requestSeekByRatio(ratio);
      }
    }

    if (shouldReload) {
      csv_path_ = nextCsvPath;
      start_offset_sec_ = nextStartOffsetSec;
      requestReload(csv_path_, start_offset_sec_);
    }

    return result;
  }

  void requestReload(const std::string & csv_path, double start_offset_sec)
  {
    {
      std::lock_guard<std::mutex> lock(controlMutex_);
      pendingCsvPath_ = csv_path;
      pendingStartOffsetSec_ = start_offset_sec;
      reloadRequested_ = true;
    }
    controlCv_.notify_all();
  }

  void requestSeekByTime(double seek_time_sec)
  {
    {
      std::lock_guard<std::mutex> lock(controlMutex_);
      pendingSeekTimeSec_ = std::max(0.0, seek_time_sec);
      seekRequested_ = true;
    }
    controlCv_.notify_all();
  }

  void requestSeekByRatio(double ratio)
  {
    const double totalDurationSec = totalDurationSec_.load();
    requestSeekByTime(totalDurationSec * ratio);
  }

  void applyPendingControl()
  {
    std::string csvPathToLoad;
    double startOffsetToLoad = 0.0;
    bool shouldReload = false;
    bool shouldSeek = false;
    double seekTimeSec = 0.0;

    {
      std::lock_guard<std::mutex> lock(controlMutex_);
      if (reloadRequested_) {
        csvPathToLoad = pendingCsvPath_;
        startOffsetToLoad = pendingStartOffsetSec_;
        reloadRequested_ = false;
        shouldReload = true;
      }
      if (seekRequested_) {
        seekTimeSec = pendingSeekTimeSec_;
        seekRequested_ = false;
        shouldSeek = true;
      }
    }

    if (shouldReload) {
      loadCsv(csvPathToLoad, startOffsetToLoad);
      shouldSeek = false;
    }

    if (shouldSeek) {
      seekToTime(seekTimeSec);
    }
  }

  void seekToTime(double seek_time_sec)
  {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (rows_.empty()) {
      currentIndex_ = 0;
      currentTimeSec_ = 0.0;
      publishStatusLocked();
      return;
    }

    const double clampedSeekTime = std::clamp(seek_time_sec, 0.0, totalDurationSec_.load());
    auto it = std::lower_bound(
      rows_.begin(), rows_.end(), clampedSeekTime,
      [](const Row & row, double t_sec) { return row.t_sec < t_sec; });
    currentIndex_ = static_cast<size_t>(std::distance(rows_.begin(), it));
    if (currentIndex_ >= rows_.size()) {
      currentIndex_ = rows_.size() - 1;
      currentTimeSec_ = rows_[currentIndex_].t_sec;
      currentIndex_ = rows_.size();
      anchorRowTimeSec_ = currentTimeSec_;
    } else {
      currentTimeSec_ = rows_[currentIndex_].t_sec;
      anchorRowTimeSec_ = rows_[currentIndex_].t_sec;
    }
    anchorWallTime_ = std::chrono::steady_clock::now();
    publishStatusLocked();
  }

  void publishStatusLocked()
  {
    std_msgs::msg::Float64MultiArray status;
    const double totalDurationSec = totalDurationSec_.load();
    const double progress = totalDurationSec > 1e-9 ? currentTimeSec_ / totalDurationSec : 0.0;
    status.data = {
      rows_.empty() ? 0.0 : 1.0,
      std::clamp(progress, 0.0, 1.0),
      currentTimeSec_,
      totalDurationSec,
      playback_rate_.load(),
      static_cast<double>(rows_.size()),
      static_cast<double>(currentIndex_),
    };
    status_pub_->publish(status);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle_;
  std::string status_topic_;
  std::mutex stateMutex_;
  std::mutex controlMutex_;
  std::condition_variable controlCv_;
  std::atomic<double> playback_rate_{1.0};
  std::atomic<double> totalDurationSec_{0.0};
  bool reloadRequested_{false};
  bool seekRequested_{false};
  std::string pendingCsvPath_;
  double pendingStartOffsetSec_{0.0};
  double pendingSeekTimeSec_{0.0};
  size_t currentIndex_{0};
  double currentTimeSec_{0.0};
  double anchorRowTimeSec_{0.0};
  std::chrono::steady_clock::time_point anchorWallTime_{std::chrono::steady_clock::now()};
  std::string loadedCsvPath_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  std::unordered_map<std::string, size_t> headerIndex_;
  std::vector<Row> rows_;
  std::thread playThread_;
  std::atomic<bool> stopRequested_{false};
  std::string csv_path_;
  std::string publish_topic_;
  double start_offset_sec_{0.0};
  double sample_hz_{50.0};
  double firstTimeSec_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvPlayer>());
  rclcpp::shutdown();
  return 0;
}
