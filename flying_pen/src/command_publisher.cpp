#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ncurses.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  explicit CommandPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("command_publisher", options)
  {
    cf_position_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::Position>(
        "cf2/cmd_position",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    key_pub_ = this->create_publisher<std_msgs::msg::String>(
      "keyboard_input", 10);

    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/cmd_force", 10);
    status_sub_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      "cf2/status", 10,
      std::bind(&CommandPublisher::statusCallback, this, std::placeholders::_1));
    mob_force_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/cf_Fext_MOB", 10,
      std::bind(&CommandPublisher::mobForceCallback, this, std::placeholders::_1));
    zero_bias_dbg_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/cf_zero_bias_dbg", 10,
      std::bind(&CommandPublisher::zeroBiasDebugCallback, this, std::placeholders::_1));

    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/crazyflie_server");

    pos_delta_[0] = this->declare_parameter<double>("dx", 0.1);
    pos_delta_[1] = this->declare_parameter<double>("dy", 0.1);
    pos_delta_[2] = this->declare_parameter<double>("dz", 0.2);
    vel_delta_[0] = this->declare_parameter<double>("dvx", 0.05);
    vel_delta_[1] = this->declare_parameter<double>("dvy", 0.05);
    vel_delta_[2] = this->declare_parameter<double>("dvz", 0.1);
    yaw_delta_deg_ = this->declare_parameter<double>("dyaw_deg", 5.0);
    force_delta_   = this->declare_parameter<double>("df", 0.01);
    command_frame_ = this->declare_parameter<std::string>("command_frame", "drone");
    end_effector_offset_ = declareOffsetParameter();
    trajectory_type_ = this->declare_parameter<std::string>("trajectory_type", "circle");
    trajectory_period_sec_ = this->declare_parameter<double>("trajectory_period_sec", 20.0);
    trajectory_ramp_time_sec_ = this->declare_parameter<double>("trajectory_ramp_time_sec", 5.0);
    trajectory_velocity_profile_mps_ =
      this->declare_parameter<double>("trajectory_velocity_profile_mps", 0.10);
    trajectory_circle_radius_y_ = this->declare_parameter<double>("trajectory_circle_radius_y", 0.20);
    trajectory_circle_radius_z_ = this->declare_parameter<double>("trajectory_circle_radius_z", 0.20);
    trajectory_rectangle_size_y_ = this->declare_parameter<double>("trajectory_rectangle_size_y", 0.20);
    trajectory_rectangle_size_z_ = this->declare_parameter<double>("trajectory_rectangle_size_z", 0.20);
    trajectory_rectangle_corner_pause_sec_ =
      this->declare_parameter<double>("trajectory_rectangle_corner_pause_sec", 2.0);
    trajectory_accel_ellipse_radius_x_ =
      this->declare_parameter<double>("trajectory_accel_ellipse_radius_x", 0.20);
    trajectory_accel_ellipse_radius_y_ =
      this->declare_parameter<double>("trajectory_accel_ellipse_radius_y", 0.20);
    trajectory_accel_circle_initial_period_sec_ =
      this->declare_parameter<double>("trajectory_accel_circle_initial_period_sec", 20.0);
    trajectory_accel_circle_period_decrement_sec_ =
      this->declare_parameter<double>("trajectory_accel_circle_period_decrement_sec", 1.0);
    trajectory_accel_circle_min_period_sec_ =
      this->declare_parameter<double>("trajectory_accel_circle_min_period_sec", 5.0);

    cmd_xyz_yaw_.fill(0.0);
    vel_xyz_.fill(0.0);
    base_cmd_xyz_.fill(0.0);
    vel_hold_xyz_.fill(0.0);
    force_des_ = 0.0;
    use_vel_mode_ = 0.0;
    mob_force_.fill(std::numeric_limits<double>::quiet_NaN());
    latest_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    displayed_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    zero_bias_count_ = -1.0;
    zero_bias_last_result_ = "not requested yet";
    status_msg_ = "ready";
    last_tick_time_ = std::chrono::steady_clock::now();
    last_battery_display_update_ = last_tick_time_;
    trajectory_enabled_target_ = false;
    trajectory_scale_ = 0.0;
    trajectory_phase_time_sec_ = 0.0;
    trajectory_accel_circle_turns_ = 0.0;

    // history init
    last_inputs_.clear();
    for (int i = 0; i < 5; i++) last_inputs_.push_back("-");

    // ncurses init
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    color_enabled_ = false;
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(1, COLOR_YELLOW, -1);
      color_enabled_ = true;
    }

    drawLayout();

    // startup log 1회만 (이 정도는 괜찮음)
    RCLCPP_INFO(
      this->get_logger(),
      "command_publisher started. trajectory=%s period=%.3fs rectangle_speed=%.3fm/s ramp=%.3fs",
      trajectory_type_.c_str(), trajectory_period_sec_, trajectory_velocity_profile_mps_,
      trajectory_ramp_time_sec_);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  // --------------------------
  // Layout rows
  // --------------------------
  static constexpr int ROW_USAGE_HEADER   = 0;
  static constexpr int ROW_USAGE_1        = 2;
  static constexpr int ROW_USAGE_2        = 3;
  static constexpr int ROW_USAGE_3        = 4;

  static constexpr int ROW_STATUS_HEADER  = 6;
  static constexpr int ROW_STATUS_MODE    = 8;
  static constexpr int ROW_STATUS_FORCE   = 9;
  static constexpr int ROW_STATUS_MOB     = 10;
  static constexpr int ROW_STATUS_BATTERY = 11;
  static constexpr int ROW_STATUS_ZERO    = 12;
  static constexpr int ROW_STATUS_MSG     = 13;

  static constexpr int ROW_CMD_HEADER     = 15;
  static constexpr int ROW_CMD_LINE1      = 17;
  static constexpr int ROW_CMD_LINE2      = 18;
  static constexpr int ROW_CMD_HIST_HDR   = 20;
  static constexpr int ROW_CMD_HIST_0     = 21;   // newest
  static constexpr int ROW_CMD_HIST_1     = 22;
  static constexpr int ROW_CMD_HIST_2     = 23;
  static constexpr int ROW_CMD_HIST_3     = 24;
  static constexpr int ROW_CMD_HIST_4     = 25;   // oldest

  static constexpr size_t HISTORY_LEN     = 5;

  void timerCallback()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_tick_time_).count();
    last_tick_time_ = now;
    if (!(dt > 0.0 && dt < 1.0)) {
      dt = 0.05;
    }

    // ✅ 키보드 버퍼를 비울 때까지 모두 읽기 (연타 누락 방지)
    int ch;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    if (isVelocityMode()) {
      updateVelocityModeCommand(dt);
    }

    updateDisplayedBatteryVoltage();
    publishPositionCmd();
    drawStatusBlock();
    drawCommandBlock();
  }

  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_battery_voltage_ = msg->battery_voltage;
  }

  void updateDisplayedBatteryVoltage()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_battery_display_update_ < 1s) {
      return;
    }

    last_battery_display_update_ = now;
    displayed_battery_voltage_ = latest_battery_voltage_;
  }

  void handleKey(char c)
  {
    if (c == 'w')       { onPositiveX(); }
    else if (c == 's')  { onNegativeX(); }
    else if (c == 'a')  { onPositiveY(); }
    else if (c == 'd')  { onNegativeY(); }
    else if (c == 'e')  { onPositiveZ(); }
    else if (c == 'q')  { onNegativeZ(); }
    else if (c == 'z')  { cmd_xyz_yaw_[3] += yaw_delta_deg_; pushInputHistory("z : yaw += dyaw"); }
    else if (c == 'c')  { cmd_xyz_yaw_[3] -= yaw_delta_deg_; pushInputHistory("c : yaw -= dyaw"); }
    else if (c == 'x')  { resetActiveCommand(); }
    else if (c == 'g')  { toggleTrajectory(); }

    // force
    else if (c == 'j')  { force_des_ += force_delta_; publishForce(); pushInputHistory("j : force += df"); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishForce(); pushInputHistory("k : force -= df"); }
    else if (c == 'l')  { force_des_ = 0.0;           publishForce(); pushInputHistory("l : force reset"); }

    // mode set
    else if (c == 'i')  { setVelMode(1.0); }
    else if (c == 'u')  { setVelMode(0.0); }
    else if (c == 'r')  { triggerMobBiasZero(); pushInputHistory("r : zero MOB bias"); }

    // ARM/DISARM
    else if (c == 'o' || c == 'p') {
      if (key_pub_) {
        std_msgs::msg::String msg;
        msg.data = std::string(1, c);
        key_pub_->publish(msg);

        if (c == 'o') {
          status_msg_ = "published 'o' to keyboard_input (ARM)";
          pushInputHistory("o : ARM (keyboard_input)");
        } else {
          status_msg_ = "published 'p' to keyboard_input (DISARM)";
          pushInputHistory("p : DISARM (keyboard_input)");
        }
      } else {
        status_msg_ = "key_pub not ready";
      }
    }

    // quit
    else if (c == 't') {
      pushInputHistory("t : quit");
      status_msg_ = "exit key pressed";
      rclcpp::shutdown();
    }
  }

  bool isVelocityMode() const
  {
    return use_vel_mode_ > 0.5;
  }

  void onPositiveX()
  {
    if (isVelocityMode()) {
      vel_xyz_[0] += vel_delta_[0];
      pushInputHistory("w : vx += dvx");
    } else {
      cmd_xyz_yaw_[0] += pos_delta_[0];
      pushInputHistory("w : x += dx");
    }
  }

  void onNegativeX()
  {
    if (isVelocityMode()) {
      vel_xyz_[0] -= vel_delta_[0];
      pushInputHistory("s : vx -= dvx");
    } else {
      cmd_xyz_yaw_[0] -= pos_delta_[0];
      pushInputHistory("s : x -= dx");
    }
  }

  void onPositiveY()
  {
    if (isVelocityMode()) {
      vel_xyz_[1] += vel_delta_[1];
      pushInputHistory("a : vy += dvy");
    } else {
      cmd_xyz_yaw_[1] += pos_delta_[1];
      pushInputHistory("a : y += dy");
    }
  }

  void onNegativeY()
  {
    if (isVelocityMode()) {
      vel_xyz_[1] -= vel_delta_[1];
      pushInputHistory("d : vy -= dvy");
    } else {
      cmd_xyz_yaw_[1] -= pos_delta_[1];
      pushInputHistory("d : y -= dy");
    }
  }

  void onPositiveZ()
  {
    if (isVelocityMode()) {
      vel_xyz_[2] += vel_delta_[2];
      pushInputHistory("e : vz += dvz");
    } else {
      cmd_xyz_yaw_[2] += pos_delta_[2];
      pushInputHistory("e : z += dz");
    }
  }

  void onNegativeZ()
  {
    if (isVelocityMode()) {
      vel_xyz_[2] -= vel_delta_[2];
      pushInputHistory("q : vz -= dvz");
    } else {
      cmd_xyz_yaw_[2] -= pos_delta_[2];
      pushInputHistory("q : z -= dz");
    }
  }

  void updateVelocityModeCommand(double dt)
  {
    vel_hold_xyz_[0] += vel_xyz_[0] * dt;
    vel_hold_xyz_[1] += vel_xyz_[1] * dt;
    vel_hold_xyz_[2] += vel_xyz_[2] * dt;

    updateTrajectoryRamp(dt);

    if (trajectory_enabled_target_ || trajectory_scale_ > 1e-6) {
      trajectory_phase_time_sec_ += dt;
      updateAcceleratingCircleState(dt);
    }

    const auto traj_offset = computeTrajectoryOffset();
    cmd_xyz_yaw_[0] = vel_hold_xyz_[0] + traj_offset[0];
    cmd_xyz_yaw_[1] = vel_hold_xyz_[1] + traj_offset[1];
    cmd_xyz_yaw_[2] = vel_hold_xyz_[2] + traj_offset[2];
  }

  void publishPositionCmd()
  {
    crazyflie_interfaces::msg::Position msg;
    const auto drone_position = resolveDronePositionCommand();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.x   = static_cast<float>(drone_position[0]);
    msg.y   = static_cast<float>(drone_position[1]);
    msg.z   = static_cast<float>(drone_position[2]);
    msg.yaw = cmd_xyz_yaw_[3];
    cf_position_pub_->publish(msg);
  }

  std::array<double, 3> declareOffsetParameter()
  {
    const auto values = this->declare_parameter<std::vector<double>>(
      "end_effector_offset", std::vector<double>{0.0, 0.0, 0.0});

    std::array<double, 3> offset{{0.0, 0.0, 0.0}};
    if (values.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter 'end_effector_offset' must have exactly 3 values. Using [0, 0, 0].");
      return offset;
    }

    for (size_t i = 0; i < 3; ++i) {
      offset[i] = values[i];
    }
    return offset;
  }

  bool usesEndEffectorFrame() const
  {
    return command_frame_ == "end_effector";
  }

  std::array<double, 3> rotateOffsetByYawDeg(double yaw_deg) const
  {
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    return {{
      c * end_effector_offset_[0] - s * end_effector_offset_[1],
      s * end_effector_offset_[0] + c * end_effector_offset_[1],
      end_effector_offset_[2]
    }};
  }

  std::array<double, 3> resolveDronePositionCommand() const
  {
    std::array<double, 3> drone_position{{
      cmd_xyz_yaw_[0],
      cmd_xyz_yaw_[1],
      cmd_xyz_yaw_[2]
    }};

    if (!usesEndEffectorFrame()) {
      return drone_position;
    }

    const auto rotated_offset = rotateOffsetByYawDeg(cmd_xyz_yaw_[3]);
    drone_position[0] -= rotated_offset[0];
    drone_position[1] -= rotated_offset[1];
    drone_position[2] -= rotated_offset[2];
    return drone_position;
  }

  void updateTrajectoryRamp(double dt)
  {
    const double ramp_time = std::max(trajectory_ramp_time_sec_, 1e-6);
    const double delta = dt / ramp_time;
    const double target = trajectory_enabled_target_ ? 1.0 : 0.0;
    if (trajectory_scale_ < target) {
      trajectory_scale_ = std::min(target, trajectory_scale_ + delta);
    } else if (trajectory_scale_ > target) {
      trajectory_scale_ = std::max(target, trajectory_scale_ - delta);
    }
  }

  void updateAcceleratingCircleState(double dt)
  {
    if (trajectory_type_ != "accelerating_circle_xy") {
      return;
    }

    const double current_period_sec = getAcceleratingCircleCurrentPeriodSec();
    trajectory_accel_circle_turns_ += dt / current_period_sec;
  }

  std::array<double, 3> computeTrajectoryOffset() const
  {
    std::array<double, 3> offset{{0.0, 0.0, 0.0}};
    if (trajectory_scale_ <= 1e-6) {
      return offset;
    }

    if (trajectory_type_ == "rectangle") {
      return computeRectangleOffset();
    }
    if (trajectory_type_ == "accelerating_circle_xy") {
      return computeAcceleratingCircleXYOffset();
    }
    return computeCircleOffset();
  }

  std::array<double, 3> computeCircleOffset() const
  {
    const double omega = 2.0 * M_PI / std::max(trajectory_period_sec_, 1e-6);
    const double phase = omega * trajectory_phase_time_sec_;
    return {{
      0.0,
      trajectory_scale_ * trajectory_circle_radius_y_ * std::cos(phase),
      trajectory_scale_ * trajectory_circle_radius_z_ * std::sin(phase)
    }};
  }

  std::array<double, 3> computeRectangleOffset() const
  {
    const double half_y = 0.5 * trajectory_rectangle_size_y_;
    const double half_z = 0.5 * trajectory_rectangle_size_z_;
    const double pause_sec = std::max(trajectory_rectangle_corner_pause_sec_, 0.0);
    const double velocity_mps = std::max(trajectory_velocity_profile_mps_, 1e-6);
    const double edge_move_secs[4] = {
      trajectory_rectangle_size_z_ / velocity_mps,
      trajectory_rectangle_size_y_ / velocity_mps,
      trajectory_rectangle_size_z_ / velocity_mps,
      trajectory_rectangle_size_y_ / velocity_mps
    };
    const double cycle_sec =
      edge_move_secs[0] + edge_move_secs[1] + edge_move_secs[2] + edge_move_secs[3] +
      4.0 * pause_sec;
    const double t = std::fmod(trajectory_phase_time_sec_, cycle_sec);

    const auto make_offset = [&](double y, double z) {
      return std::array<double, 3>{{
        0.0,
        trajectory_scale_ * y,
        trajectory_scale_ * z
      }};
    };

    const auto blend = [](double a, double b, double s) {
      return a + (b - a) * s;
    };

    const double corners[4][2] = {
      { half_y, -half_z},
      { half_y,  half_z},
      {-half_y,  half_z},
      {-half_y, -half_z}
    };

    double segment_start = 0.0;
    for (int i = 0; i < 4; ++i) {
      if (t < segment_start + pause_sec) {
        return make_offset(corners[i][0], corners[i][1]);
      }
      segment_start += pause_sec;

      const double edge_move_sec = std::max(edge_move_secs[i], 0.0);
      if (t < segment_start + edge_move_sec) {
        const int next = (i + 1) % 4;
        const double s = (t - segment_start) / std::max(edge_move_sec, 1e-6);
        return make_offset(
          blend(corners[i][0], corners[next][0], s),
          blend(corners[i][1], corners[next][1], s));
      }
      segment_start += edge_move_sec;
    }

    return make_offset(half_y, -half_z);
  }

  std::array<double, 3> computeAcceleratingCircleXYOffset() const
  {
    const double phase = 2.0 * M_PI * trajectory_accel_circle_turns_;
    return {{
      trajectory_scale_ * trajectory_accel_ellipse_radius_x_ * std::cos(phase),
      trajectory_scale_ * trajectory_accel_ellipse_radius_y_ * std::sin(phase),
      0.0
    }};
  }

  double getAcceleratingCircleCurrentPeriodSec() const
  {
    const double completed_laps = std::floor(trajectory_accel_circle_turns_);
    const double period_sec =
      trajectory_accel_circle_initial_period_sec_ -
      completed_laps * trajectory_accel_circle_period_decrement_sec_;
    return std::max(trajectory_accel_circle_min_period_sec_, std::max(period_sec, 1e-6));
  }

  void resetTrajectoryState()
  {
    trajectory_enabled_target_ = false;
    trajectory_scale_ = 0.0;
    trajectory_phase_time_sec_ = 0.0;
    trajectory_accel_circle_turns_ = 0.0;
  }

  void publishForce()
  {
    if (!force_pub_) {
      status_msg_ = "force_pub not ready";
      return;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(force_des_);
    force_pub_->publish(msg);

    char buf[128];
    snprintf(buf, sizeof(buf), "set cmd_fx = %.3f (via su_interface)", force_des_);
    status_msg_ = buf;
  }

  void setVelMode(double mode)
  {
    double new_mode = (mode > 0.5) ? 1.0 : 0.0;
    use_vel_mode_ = new_mode;

    if (!use_vel_mode_pub_) {
      status_msg_ = "use_vel_mode_pub not ready";
      return;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(use_vel_mode_);
    use_vel_mode_pub_->publish(msg);

    if (isVelocityMode()) {
      base_cmd_xyz_[0] = cmd_xyz_yaw_[0];
      base_cmd_xyz_[1] = cmd_xyz_yaw_[1];
      base_cmd_xyz_[2] = cmd_xyz_yaw_[2];
      vel_hold_xyz_ = base_cmd_xyz_;
      vel_xyz_.fill(0.0);
      resetTrajectoryState();

      char buf[160];
      snprintf(buf, sizeof(buf),
              "entered VELOCITY mode with base=(%.3f, %.3f, %.3f)",
              base_cmd_xyz_[0], base_cmd_xyz_[1], base_cmd_xyz_[2]);
      status_msg_ = buf;
      pushInputHistory("i : enter velocity mode at current cmd");
    } else {
      vel_xyz_.fill(0.0);
      resetTrajectoryState();
      status_msg_ = "entered POSITION mode, velocity command reset";
      pushInputHistory("u : enter position mode");
    }
  }

  void resetActiveCommand()
  {
    if (isVelocityMode()) {
      vel_hold_xyz_[0] = cmd_xyz_yaw_[0];
      vel_hold_xyz_[1] = cmd_xyz_yaw_[1];
      vel_hold_xyz_[2] = cmd_xyz_yaw_[2];
      vel_xyz_.fill(0.0);
      resetTrajectoryState();
      status_msg_ = "Velocity and trajectory reset to zero, hold current position";
      pushInputHistory("x : hold current position");
    } else {
      cmd_xyz_yaw_[0] = 0.0;
      cmd_xyz_yaw_[1] = 0.0;
      cmd_xyz_yaw_[2] = 0.0;
      status_msg_ = "Position reset to zero (immediate)";
      pushInputHistory("x : reset position cmd");
    }
  }

  void toggleTrajectory()
  {
    if (!isVelocityMode()) {
      status_msg_ = "trajectory toggle is available only in VELOCITY mode";
      pushInputHistory("g : trajectory ignored in position mode");
      return;
    }

    if (!trajectory_enabled_target_ && trajectory_scale_ <= 1e-6) {
      trajectory_phase_time_sec_ = 0.0;
      trajectory_accel_circle_turns_ = 0.0;
      trajectory_enabled_target_ = true;
      status_msg_ = "trajectory ramp up started";
      pushInputHistory("g : trajectory ramp up");
    } else if (!trajectory_enabled_target_) {
      trajectory_enabled_target_ = true;
      status_msg_ = "trajectory ramp up resumed";
      pushInputHistory("g : trajectory ramp up");
    } else {
      trajectory_enabled_target_ = false;
      status_msg_ = "trajectory ramp down started";
      pushInputHistory("g : trajectory ramp down");
    }
  }

  void triggerMobBiasZero()
  {
    if (!param_client_) {
      status_msg_ = "param_client not ready";
      RCLCPP_WARN(this->get_logger(), "%s", status_msg_.c_str());
      return;
    }

    using namespace std::chrono_literals;
    if (!param_client_->wait_for_service(100ms)) {
      status_msg_ = "crazyflie_server param service not ready";
      RCLCPP_WARN(this->get_logger(), "%s", status_msg_.c_str());
      return;
    }

    const std::string param_name_primary = "cf2.params.su_wrench.zeroBias";
    const std::string param_name_legacy = "cf2.params.suWrenchObs.zeroBias";
    auto future = param_client_->set_parameters(
      {
        rclcpp::Parameter(param_name_primary, 1),
        rclcpp::Parameter(param_name_legacy, 1)
      });

    status_msg_ = "requested MOB bias zeroing";
    RCLCPP_INFO(this->get_logger(), "%s", status_msg_.c_str());

    future.wait_for(200ms);
    if (future.wait_for(0ms) != std::future_status::ready) {
      status_msg_ = "MOB bias zero request sent (waiting on server)";
      RCLCPP_WARN(this->get_logger(), "%s", status_msg_.c_str());
      return;
    }

    bool any_success = false;
    std::string detail;
    const auto results = future.get();
    for (size_t i = 0; i < results.size(); ++i) {
      const auto &result = results[i];
      if (result.successful) {
        any_success = true;
      }

      const std::string &name = (i == 0) ? param_name_primary : param_name_legacy;
      if (!detail.empty()) {
        detail += " | ";
      }
      detail += name + "=" + (result.successful ? "ok" : "fail");
      if (!result.successful && !result.reason.empty()) {
        detail += "(" + result.reason + ")";
      }
    }

    if (any_success) {
      status_msg_ = "MOB bias zero trigger sent";
      zero_bias_last_result_ = detail;
      RCLCPP_INFO(this->get_logger(), "%s: %s", status_msg_.c_str(), zero_bias_last_result_.c_str());
    } else if (!detail.empty()) {
      status_msg_ = "MOB bias zero failed";
      zero_bias_last_result_ = detail;
      RCLCPP_WARN(this->get_logger(), "%s: %s", status_msg_.c_str(), zero_bias_last_result_.c_str());
    } else {
      status_msg_ = "MOB bias zero failed";
      zero_bias_last_result_ = "set_parameters returned no details";
      RCLCPP_WARN(this->get_logger(), "%s: %s", status_msg_.c_str(), zero_bias_last_result_.c_str());
    }
  }

  void mobForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 3) {
      return;
    }

    mob_force_[0] = msg->values[0];
    mob_force_[1] = msg->values[1];
    mob_force_[2] = msg->values[2];
  }

  void zeroBiasDebugCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.empty()) {
      return;
    }

    zero_bias_count_ = msg->values[0];
  }

  // --------------------------
  // Input history (최근 5개)
  // --------------------------
  void pushInputHistory(const std::string& s)
  {
    last_inputs_.push_front(s);
    while (last_inputs_.size() > HISTORY_LEN) last_inputs_.pop_back();
    while (last_inputs_.size() < HISTORY_LEN) last_inputs_.push_back("-");
  }

  // --------------------------
  // Ncurses UI
  // --------------------------
  void drawSepLine(int row, const char* title)
  {
    move(row, 0);
    clrtoeol();
    printw("========================%s========================", title);
  }

  void drawLayout()
  {
    clear();

    drawSepLine(ROW_USAGE_HEADER, "usage");
    mvprintw(ROW_USAGE_1, 0, "position mode:    w/s(x), a/d(y), e/q(z), z/c(yaw), x(reset pos)");
    mvprintw(ROW_USAGE_2, 0, "velocity mode:    i(save base), w/s/a/d/e/q -> velocity, x(hold), g(traj), u(pos)");
    mvprintw(ROW_USAGE_3, 0, "force/bias/arm:   j/k/l (cmd_fx), r(zero bias), o/p arm/disarm, t quit");

    drawSepLine(ROW_STATUS_HEADER, "status");
    mvprintw(ROW_STATUS_MODE,  0, "mode: ");
    mvprintw(ROW_STATUS_FORCE, 0, "force command: ");
    mvprintw(ROW_STATUS_MOB,   0, "MOB force: ");
    mvprintw(ROW_STATUS_BATTERY, 0, "battery voltage: ");
    mvprintw(ROW_STATUS_ZERO,  0, "zero bias dbg: ");
    mvprintw(ROW_STATUS_MSG,   0, "status: ");

    drawSepLine(ROW_CMD_HEADER, "Position Command, Now");
    mvprintw(ROW_CMD_LINE1, 0, "x = 0.000 , y = 0.000 , z = 0.000");
    mvprintw(ROW_CMD_LINE2, 0, "yaw = 0.0 deg, vx = 0.000, vy = 0.000, vz = 0.000");

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (int i = 0; i < 5; i++) {
      mvprintw(ROW_CMD_HIST_0 + i, 0, "  %d) -", i + 1);
    }

    refresh();
  }

  void drawStatusBlock()
  {
    const char* mode_str = (use_vel_mode_ > 0.5) ? "VELOCITY" : "POSITION";

    move(ROW_STATUS_MODE, 0);
    clrtoeol();
    printw("mode: %s, frame: %s, base xyz: %.3f %.3f %.3f",
           mode_str, command_frame_.c_str(),
           base_cmd_xyz_[0], base_cmd_xyz_[1], base_cmd_xyz_[2]);

    move(ROW_STATUS_FORCE, 0);
    clrtoeol();
    printw("force command: %.3f", force_des_);

    move(ROW_STATUS_BATTERY, 0);
    clrtoeol();
    if (std::isfinite(displayed_battery_voltage_)) {
      printw("battery voltage: %.2f V (1 Hz)", displayed_battery_voltage_);
    } else {
      printw("battery voltage: waiting for cf2/status");
    }

    move(ROW_STATUS_MSG, 0);
    clrtoeol();
    printw("status: %s", status_msg_.c_str());

    move(ROW_STATUS_MOB, 0);
    clrtoeol();
    if (std::isfinite(mob_force_[0]) && std::isfinite(mob_force_[1]) && std::isfinite(mob_force_[2])) {
      const double mob_norm = std::sqrt(
        mob_force_[0] * mob_force_[0] +
        mob_force_[1] * mob_force_[1] +
        mob_force_[2] * mob_force_[2]);
      printw("MOB force: x=%.3f, y=%.3f, z=%.3f [N], norm=%.3f",
             mob_force_[0], mob_force_[1], mob_force_[2], mob_norm);
    } else {
      printw("MOB force: waiting for cf2/cf_Fext_MOB");
    }

    move(ROW_STATUS_ZERO, 0);
    clrtoeol();
    if (std::isfinite(zero_bias_count_)) {
      printw("zero bias dbg: count=%.0f, last=%s",
             zero_bias_count_, zero_bias_last_result_.c_str());
    } else {
      printw("zero bias dbg: waiting for cf2/cf_zero_bias_dbg, last=%s",
             zero_bias_last_result_.c_str());
    }

    refresh();
  }

  void drawCommandBlock()
  {
    move(ROW_CMD_LINE1, 0);
    clrtoeol();
    const auto drone_position = resolveDronePositionCommand();
    printw("cmd xyz = %.3f , %.3f , %.3f -> drone xyz = %.3f , %.3f , %.3f",
           cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2],
           drone_position[0], drone_position[1], drone_position[2]);

    move(ROW_CMD_LINE2, 0);
    clrtoeol();
    if (trajectory_type_ == "accelerating_circle_xy") {
      printw("yaw = %.1f deg, vx = %.3f, vy = %.3f, vz = %.3f, traj=%s %.2f, period=%.2f, laps=%.2f",
             cmd_xyz_yaw_[3], vel_xyz_[0], vel_xyz_[1], vel_xyz_[2],
             trajectory_type_.c_str(), trajectory_scale_,
             getAcceleratingCircleCurrentPeriodSec(), trajectory_accel_circle_turns_);
    } else if (trajectory_type_ == "rectangle") {
      printw("yaw = %.1f deg, vx = %.3f, vy = %.3f, vz = %.3f, traj=%s %.2f, speed=%.2f m/s, size_yz=(%.2f, %.2f), pause=%.2f",
             cmd_xyz_yaw_[3], vel_xyz_[0], vel_xyz_[1], vel_xyz_[2],
             trajectory_type_.c_str(), trajectory_scale_, trajectory_velocity_profile_mps_,
             trajectory_rectangle_size_y_, trajectory_rectangle_size_z_,
             trajectory_rectangle_corner_pause_sec_);
    } else {
      printw("yaw = %.1f deg, vx = %.3f, vy = %.3f, vz = %.3f, traj=%s %.2f",
             cmd_xyz_yaw_[3], vel_xyz_[0], vel_xyz_[1], vel_xyz_[2],
             trajectory_type_.c_str(), trajectory_scale_);
    }

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (int i = 0; i < 5; i++) {
      move(ROW_CMD_HIST_0 + i, 0);
      clrtoeol();
      printw("  %d) %s", i + 1, last_inputs_[static_cast<size_t>(i)].c_str());
    }

    refresh();
  }

  // --------------------------
  // Members
  // --------------------------
  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              key_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             force_pub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr mob_force_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr zero_bias_dbg_sub_;
  std::shared_ptr<rclcpp::AsyncParametersClient>                   param_client_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 3> pos_delta_;
  std::array<double, 3> vel_xyz_;
  std::array<double, 3> vel_delta_;
  std::array<double, 3> base_cmd_xyz_;
  std::array<double, 3> vel_hold_xyz_;
  double yaw_delta_deg_;
  double force_des_;
  double force_delta_;
  double use_vel_mode_;
  bool trajectory_enabled_target_;
  double trajectory_scale_;
  double trajectory_phase_time_sec_;
  std::string trajectory_type_;
  double trajectory_period_sec_;
  double trajectory_ramp_time_sec_;
  double trajectory_velocity_profile_mps_;
  double trajectory_circle_radius_y_;
  double trajectory_circle_radius_z_;
  double trajectory_rectangle_size_y_;
  double trajectory_rectangle_size_z_;
  double trajectory_rectangle_corner_pause_sec_;
  double trajectory_accel_ellipse_radius_x_;
  double trajectory_accel_ellipse_radius_y_;
  double trajectory_accel_circle_initial_period_sec_;
  double trajectory_accel_circle_period_decrement_sec_;
  double trajectory_accel_circle_min_period_sec_;
  double trajectory_accel_circle_turns_;
  std::array<double, 3> mob_force_;
  double latest_battery_voltage_;
  double displayed_battery_voltage_;
  double zero_bias_count_;
  std::string command_frame_;
  std::array<double, 3> end_effector_offset_;

  std::string status_msg_;
  std::string zero_bias_last_result_;
  bool color_enabled_;
  std::chrono::steady_clock::time_point last_tick_time_;
  std::chrono::steady_clock::time_point last_battery_display_update_;

  std::deque<std::string> last_inputs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  bool has_params_file = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--params-file") {
      has_params_file = true;
      break;
    }
  }

  if (!has_params_file) {
    const auto config_path =
      ament_index_cpp::get_package_share_directory("flying_pen") + "/config/command.yaml";
    options.arguments({"--ros-args", "--params-file", config_path});
  }

  auto node = std::make_shared<CommandPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
