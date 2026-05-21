#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
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

std::string fmt(double value)
{
  std::ostringstream oss;
  oss << std::setprecision(10) << std::fixed << value;
  return oss.str();
}

double quietNaN()
{
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

class OfflineResultLogger : public rclcpp::Node
{
public:
  OfflineResultLogger()
  : Node("offline_result_logger")
  {
    declare_parameter<std::string>("source_csv_path", "");
    declare_parameter<std::string>("raw_topic", "/data_logging_msg");
    declare_parameter<std::string>("mob_1st_topic", "/log_player/offline/drone_external_force_mob");
    declare_parameter<std::string>("mob_2nd_topic", "/log_player/offline/drone_external_force_mob_2nd");
    declare_parameter<std::string>("mob_consistency_topic", "/log_player/offline/drone_external_force_mob_2nd_tau");
    declare_parameter<std::string>("mob_consistency_alt_topic", "/log_player/offline/drone_external_force_mob_2nd_tau_ke2");
    declare_parameter<std::string>("mob_consistency_match_topic", "/log_player/offline/drone_external_force_mob_2nd_tau_consistency");
    declare_parameter<std::string>("mob_consistency_residual_topic", "/log_player/offline/drone_external_force_mob_2nd_tau_residual");
    declare_parameter<std::string>("normal_quat_topic_pure", "/log_player/offline/contact_frame_quat_pure");
    declare_parameter<std::string>("normal_quat_topic_k1", "/log_player/offline/contact_frame_quat");
    declare_parameter<std::string>("normal_quat_topic_k1_novcorr", "/log_player/offline/contact_frame_quat_k1_novcorr");
    declare_parameter<std::string>("normal_quat_topic_k2", "/log_player/offline/contact_frame_quat_k2");
    declare_parameter<std::string>("normal_quat_topic_k2_nolpf", "/log_player/offline/contact_frame_quat_k2_nolpf");
    declare_parameter<std::string>("normal_quat_topic_k2_novcorr", "/log_player/offline/contact_frame_quat_k2_novcorr");
    declare_parameter<std::string>("contact_force_x_topic", "/log_player/offline/contact_force_x");
    declare_parameter<std::string>("normal_metrics_topic_k1", "/log_player/offline/normal_debug_metrics");

    sub_raw_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("raw_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::rawCb, this, std::placeholders::_1));
    sub_mob_1st_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_1st_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mob1Cb, this, std::placeholders::_1));
    sub_mob_2nd_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_2nd_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mob2Cb, this, std::placeholders::_1));
    sub_mob_consistency_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_consistency_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mobcCb, this, std::placeholders::_1));
    sub_mob_consistency_alt_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_consistency_alt_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mobcAltCb, this, std::placeholders::_1));
    sub_mob_match_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_consistency_match_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mobMatchCb, this, std::placeholders::_1));
    sub_mob_residual_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("mob_consistency_residual_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::mobResidualCb, this, std::placeholders::_1));
    sub_normal_quat_pure_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_pure").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatPureCb, this, std::placeholders::_1));
    sub_normal_quat_k1_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_k1").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatK1Cb, this, std::placeholders::_1));
    sub_normal_quat_k1_novcorr_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_k1_novcorr").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatK1NoVelCorrCb, this, std::placeholders::_1));
    sub_normal_quat_k2_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_k2").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatK2Cb, this, std::placeholders::_1));
    sub_normal_quat_k2_nolpf_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_k2_nolpf").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatK2NoLpfCb, this, std::placeholders::_1));
    sub_normal_quat_k2_novcorr_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      get_parameter("normal_quat_topic_k2_novcorr").as_string(), 10,
      std::bind(&OfflineResultLogger::normalQuatK2NoVelCorrCb, this, std::placeholders::_1));
    sub_contact_force_x_ = create_subscription<std_msgs::msg::Float32>(
      get_parameter("contact_force_x_topic").as_string(), 10,
      std::bind(&OfflineResultLogger::contactForceXCb, this, std::placeholders::_1));
    sub_normal_metrics_k1_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("normal_metrics_topic_k1").as_string(), 10,
      std::bind(&OfflineResultLogger::normalMetricsK1Cb, this, std::placeholders::_1));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&OfflineResultLogger::onParametersSet, this, std::placeholders::_1));

    const std::string source = expandUser(get_parameter("source_csv_path").as_string());
    if (!source.empty()) {
      configureSource(source);
    }
  }

  ~OfflineResultLogger() override
  {
    closeOutput();
  }

private:
  struct WrenchData
  {
    std::array<double, 3> force{{quietNaN(), quietNaN(), quietNaN()}};
    std::array<double, 3> torque{{quietNaN(), quietNaN(), quietNaN()}};
  };

  struct NormalMetricsData
  {
    double gamma_v{quietNaN()};
    std::array<double, 3> w_s{{quietNaN(), quietNaN(), quietNaN()}};
    std::array<double, 3> n_f{{quietNaN(), quietNaN(), quietNaN()}};
    std::array<double, 3> f_g{{quietNaN(), quietNaN(), quietNaN()}};
    std::array<double, 3> n_alg{{quietNaN(), quietNaN(), quietNaN()}};
  };

  void closeOutput()
  {
    if (out_.is_open()) {
      out_.flush();
      out_.close();
    }
  }

  static std::filesystem::path buildOutputPath(const std::filesystem::path & source_path)
  {
    std::filesystem::path out = source_path;
    const std::string marker = "/0428_experient/";
    const std::string replacement = "/0428_experient_again/";
    std::string out_str = out.string();
    const auto pos = out_str.find(marker);
    if (pos != std::string::npos) {
      out_str.replace(pos, marker.size(), replacement);
      out = out_str;
    } else {
      out = source_path.parent_path().parent_path() / "0428_experient_again" / source_path.parent_path().filename() / source_path.filename();
    }
    out.replace_filename(out.stem().string() + "_again" + out.extension().string());
    return out;
  }

  bool configureSource(const std::string & source_csv_path)
  {
    std::ifstream file(source_csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open source csv: %s", source_csv_path.c_str());
      return false;
    }

    std::string header;
    if (!std::getline(file, header)) {
      RCLCPP_ERROR(get_logger(), "Source csv is empty: %s", source_csv_path.c_str());
      return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty()) {
        lines.push_back(line);
      }
    }

    const auto output_path = buildOutputPath(source_csv_path);
    std::filesystem::create_directories(output_path.parent_path());
    closeOutput();
    out_.open(output_path, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open output csv: %s", output_path.string().c_str());
      return false;
    }

    out_ << header
         << ",offline_mob1_fx,offline_mob1_fy,offline_mob1_fz,offline_mob1_tx,offline_mob1_ty,offline_mob1_tz"
         << ",offline_mob2_fx,offline_mob2_fy,offline_mob2_fz,offline_mob2_tx,offline_mob2_ty,offline_mob2_tz"
         << ",offline_mobc_fx,offline_mobc_fy,offline_mobc_fz,offline_mobc_tx,offline_mobc_ty,offline_mobc_tz"
         << ",offline_mobc2_fx,offline_mobc2_fy,offline_mobc2_fz,offline_mobc2_tx,offline_mobc2_ty,offline_mobc2_tz"
         << ",offline_tauhat_x,offline_tauhat_y,offline_tauhat_z"
         << ",offline_rxf_x,offline_rxf_y,offline_rxf_z"
         << ",offline_etau_x,offline_etau_y,offline_etau_z,offline_rho_tau"
         << ",offline_normal_pure_nx,offline_normal_pure_ny,offline_normal_pure_nz"
         << ",offline_normal_k1_nx,offline_normal_k1_ny,offline_normal_k1_nz"
         << ",offline_normal_k1_novcorr_nx,offline_normal_k1_novcorr_ny,offline_normal_k1_novcorr_nz"
         << ",offline_normal_k1_gamma_v"
         << ",offline_normal_k1_ws_x,offline_normal_k1_ws_y,offline_normal_k1_ws_z"
         << ",offline_normal_k1_nf_x,offline_normal_k1_nf_y,offline_normal_k1_nf_z"
         << ",offline_normal_k1_fg_x,offline_normal_k1_fg_y,offline_normal_k1_fg_z"
         << ",offline_normal_k1_nalg_x,offline_normal_k1_nalg_y,offline_normal_k1_nalg_z"
         << ",offline_normal_k2_nx,offline_normal_k2_ny,offline_normal_k2_nz"
         << ",offline_normal_k2_nolpf_nx,offline_normal_k2_nolpf_ny,offline_normal_k2_nolpf_nz"
         << ",offline_normal_k2_novcorr_nx,offline_normal_k2_novcorr_ny,offline_normal_k2_novcorr_nz"
         << ",offline_contact_force_x\n";

    {
      std::lock_guard<std::mutex> lock(mtx_);
      source_csv_path_ = source_csv_path;
      output_csv_path_ = output_path.string();
      source_rows_ = std::move(lines);
      row_cursor_ = 0;
    }

    RCLCPP_INFO(
      get_logger(), "offline_result_logger ready: %s -> %s",
      source_csv_path.c_str(), output_csv_path_.c_str());
    return true;
  }

  static std::array<double, 3> quatToNormal(const geometry_msgs::msg::Quaternion & q_msg)
  {
    const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (q.norm() < 1e-9) {
      return {{quietNaN(), quietNaN(), quietNaN()}};
    }
    const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
    return {{r(0, 0), r(1, 0), r(2, 0)}};
  }

  static WrenchData toWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    WrenchData out;
    out.force = {{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}};
    out.torque = {{msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z}};
    return out;
  }

  void mob1Cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mob1_ = toWrench(msg);
  }

  void mob2Cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mob2_ = toWrench(msg);
  }

  void mobcCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mobc_ = toWrench(msg);
  }

  void mobcAltCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mobc2_ = toWrench(msg);
  }

  void mobMatchCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    tauhat_ = {{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}};
    rxf_ = {{msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z}};
  }

  void mobResidualCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    e_tau_ = {{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}};
    rho_tau_ = msg->wrench.torque.x;
  }

  void normalQuatPureCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_pure_ = quatToNormal(msg->quaternion);
  }

  void normalQuatK1Cb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_k1_ = quatToNormal(msg->quaternion);
  }

  void normalQuatK2Cb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_k2_ = quatToNormal(msg->quaternion);
  }

  void normalQuatK1NoVelCorrCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_k1_novcorr_ = quatToNormal(msg->quaternion);
  }

  void normalQuatK2NoLpfCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_k2_nolpf_ = quatToNormal(msg->quaternion);
  }

  void normalQuatK2NoVelCorrCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    normal_vec_k2_novcorr_ = quatToNormal(msg->quaternion);
  }

  void contactForceXCb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    contact_force_x_ = msg->data;
  }

  void normalMetricsK1Cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (msg->data.size() < 43) {
      return;
    }

    normal_metrics_k1_.gamma_v = msg->data[0];
    normal_metrics_k1_.w_s = {{msg->data[37], msg->data[38], msg->data[39]}};
    normal_metrics_k1_.n_f = {{msg->data[31], msg->data[32], msg->data[33]}};
    normal_metrics_k1_.f_g = {{msg->data[40], msg->data[41], msg->data[42]}};
    normal_metrics_k1_.n_alg = {{msg->data[34], msg->data[35], msg->data[36]}};
  }

  void rawCb(const std_msgs::msg::Float64MultiArray::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out_.is_open() || row_cursor_ >= source_rows_.size()) {
      return;
    }

    out_ << source_rows_[row_cursor_];
    appendWrench(mob1_);
    appendWrench(mob2_);
    appendWrench(mobc_);
    appendWrench(mobc2_);
    appendVec(tauhat_);
    appendVec(rxf_);
    appendVec(e_tau_);
    out_ << "," << fmt(rho_tau_);
    appendVec(normal_vec_pure_);
    appendVec(normal_vec_k1_);
    appendVec(normal_vec_k1_novcorr_);
    out_ << "," << fmt(normal_metrics_k1_.gamma_v);
    appendVec(normal_metrics_k1_.w_s);
    appendVec(normal_metrics_k1_.n_f);
    appendVec(normal_metrics_k1_.f_g);
    appendVec(normal_metrics_k1_.n_alg);
    appendVec(normal_vec_k2_);
    appendVec(normal_vec_k2_nolpf_);
    appendVec(normal_vec_k2_novcorr_);
    out_ << "," << fmt(contact_force_x_) << "\n";

    ++row_cursor_;
    if (row_cursor_ <= 20 || (row_cursor_ % 100 == 0)) {
      out_.flush();
    }
  }

  void appendWrench(const WrenchData & wrench)
  {
    appendVec(wrench.force);
    appendVec(wrench.torque);
  }

  void appendVec(const std::array<double, 3> & vec)
  {
    out_ << "," << fmt(vec[0]) << "," << fmt(vec[1]) << "," << fmt(vec[2]);
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "source_csv_path") {
        const std::string source = expandUser(parameter.as_string());
        if (source.empty()) {
          closeOutput();
          std::lock_guard<std::mutex> lock(mtx_);
          source_rows_.clear();
          row_cursor_ = 0;
          source_csv_path_.clear();
          output_csv_path_.clear();
          continue;
        }
        if (!configureSource(source)) {
          result.successful = false;
          result.reason = "failed to configure source csv";
          return result;
        }
      }
    }

    return result;
  }

  std::mutex mtx_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_raw_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_1st_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_2nd_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_consistency_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_consistency_alt_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_match_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_mob_residual_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_pure_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_k1_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_k1_novcorr_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_k2_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_k2_nolpf_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_normal_quat_k2_novcorr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_contact_force_x_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_normal_metrics_k1_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::ofstream out_;
  std::string source_csv_path_;
  std::string output_csv_path_;
  std::vector<std::string> source_rows_;
  size_t row_cursor_{0};

  WrenchData mob1_;
  WrenchData mob2_;
  WrenchData mobc_;
  WrenchData mobc2_;
  std::array<double, 3> tauhat_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> rxf_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> e_tau_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_pure_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_k1_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_k1_novcorr_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_k2_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_k2_nolpf_{{quietNaN(), quietNaN(), quietNaN()}};
  std::array<double, 3> normal_vec_k2_novcorr_{{quietNaN(), quietNaN(), quietNaN()}};
  NormalMetricsData normal_metrics_k1_;
  double rho_tau_{quietNaN()};
  double contact_force_x_{quietNaN()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OfflineResultLogger>());
  rclcpp::shutdown();
  return 0;
}
