#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

class LogDecoder : public rclcpp::Node
{
public:
  LogDecoder()
  : Node("log_decoder")
  {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/data_logging_msg");
    layout_ = declare_parameter<std::string>("layout", "auto");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    drone_frame_ = declare_parameter<std::string>("drone_frame", "crazyflie");
    ee_frame_ = declare_parameter<std::string>("end_effector_frame", "end_effector");
    cmd_drone_frame_ = declare_parameter<std::string>("cmd_drone_frame", "cmd_drone");
    cmd_ee_frame_ = declare_parameter<std::string>("cmd_end_effector_frame", "cmd_end_effector");
    position_lpf_enable_ = declare_parameter<bool>("position_lpf_enable", true);
    position_lpf_cutoff_hz_ = declare_parameter<double>("position_lpf_cutoff_hz", 2.0);

    auto ee_offset_param = declare_parameter<std::vector<double>>(
      "end_effector_offset", std::vector<double>{0.09, 0.0, 0.085});
    if (ee_offset_param.size() != 3) {
      ee_offset_param = {0.09, 0.0, 0.085};
    }
    ee_offset_body_ = Eigen::Vector3d(ee_offset_param[0], ee_offset_param[1], ee_offset_param[2]);

    pose_topic_ = declare_parameter<std::string>("pose_topic", "/log_player/out/pose");
    vel_topic_ = declare_parameter<std::string>("vel_topic", "/log_player/out/vel");
    acc_topic_ = declare_parameter<std::string>("acc_topic", "/log_player/out/acc");
    angvel_topic_ = declare_parameter<std::string>("angvel_topic", "/log_player/out/ang_vel");
    ee_pose_topic_ = declare_parameter<std::string>("ee_pose_topic", "/log_player/out/ee_pose");
    ee_vel_topic_ = declare_parameter<std::string>("ee_vel_topic", "/log_player/out/ee_velocity");
    ee_acc_topic_ = declare_parameter<std::string>("ee_acc_topic", "/log_player/out/ee_acceleration");
    cmd_drone_topic_ = declare_parameter<std::string>("cmd_drone_topic", "/log_player/out/cmd_drone");
    cmd_ee_topic_ = declare_parameter<std::string>("cmd_ee_topic", "/log_player/out/cmd_ee");
    cmd_active_topic_ = declare_parameter<std::string>("cmd_active_topic", "/log_player/out/cmd_active");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/log_player/out/cmd_vel");
    battery_status_topic_ = declare_parameter<std::string>("battery_status_topic", "/log_player/out/battery_status");
    battery_raw_topic_ = declare_parameter<std::string>("battery_raw_topic", "/log_player/out/battery_raw");
    battery_filtered_topic_ = declare_parameter<std::string>("battery_filtered_topic", "/log_player/out/battery_filtered");
    online_force_topic_ = declare_parameter<std::string>("online_force_topic", "/log_player/online/force_estimate");
    online_world_force_raw_topic_ = declare_parameter<std::string>(
      "online_world_force_raw_topic", "/log_player/online/world_force_raw");
    online_world_force_scaled_topic_ = declare_parameter<std::string>(
      "online_world_force_scaled_topic", "/log_player/online/world_force_scaled");

    auto qos = rclcpp::SensorDataQoS();
    sub_raw_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      raw_topic_, qos, std::bind(&LogDecoder::rawCb, this, std::placeholders::_1));

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    pub_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(vel_topic_, 10);
    pub_acc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(acc_topic_, 10);
    pub_angvel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(angvel_topic_, 10);
    pub_ee_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(ee_pose_topic_, 10);
    pub_ee_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(ee_vel_topic_, 10);
    pub_ee_acc_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(ee_acc_topic_, 10);
    pub_cmd_drone_ = create_publisher<geometry_msgs::msg::PoseStamped>(cmd_drone_topic_, 10);
    pub_cmd_ee_ = create_publisher<geometry_msgs::msg::PoseStamped>(cmd_ee_topic_, 10);
    pub_cmd_active_ = create_publisher<geometry_msgs::msg::PoseStamped>(cmd_active_topic_, 10);
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(cmd_vel_topic_, 10);
    pub_battery_status_ = create_publisher<std_msgs::msg::Float64>(battery_status_topic_, 10);
    pub_battery_raw_ = create_publisher<std_msgs::msg::Float64>(battery_raw_topic_, 10);
    pub_battery_filtered_ = create_publisher<std_msgs::msg::Float64>(battery_filtered_topic_, 10);
    pub_online_force_ = create_publisher<geometry_msgs::msg::WrenchStamped>(online_force_topic_, 10);
    pub_online_world_force_raw_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      online_world_force_raw_topic_, 10);
    pub_online_world_force_scaled_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      online_world_force_scaled_topic_, 10);

    RCLCPP_INFO(get_logger(), "log_decoder subscribed to %s", raw_topic_.c_str());
  }

private:
  enum class LayoutType
  {
    Auto,
    Basic32,
    Specific49,
    Specific50
  };

  static double wrapToPi(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  LayoutType resolvedLayout(size_t n) const
  {
    if (layout_ == "basic32") {
      return LayoutType::Basic32;
    }
    if (layout_ == "specific49") {
      return LayoutType::Specific49;
    }
    if (layout_ == "specific50") {
      return LayoutType::Specific50;
    }
    if (n == 50) {
      return LayoutType::Specific50;
    }
    if (n >= 49) {
      return LayoutType::Specific49;
    }
    return LayoutType::Basic32;
  }

  geometry_msgs::msg::PoseStamped makePose(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & pos,
    const tf2::Quaternion & quat) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    return msg;
  }

  geometry_msgs::msg::Vector3Stamped makeVec(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & vec,
    const std::string & frame) const
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.vector.x = vec.x();
    msg.vector.y = vec.y();
    msg.vector.z = vec.z();
    return msg;
  }

  geometry_msgs::msg::WrenchStamped makeWrench(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & force_world) const
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.wrench.force.x = force_world.x();
    msg.wrench.force.y = force_world.y();
    msg.wrench.force.z = force_world.z();
    return msg;
  }

  std_msgs::msg::Float64 makeScalar(double value) const
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    return msg;
  }

  static double finiteOrFallback(double primary, double fallback)
  {
    return std::isfinite(primary) ? primary : fallback;
  }

  static bool allFinite(const Eigen::Vector3d & v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  static bool allFinite(const Eigen::Vector4d & v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()) && std::isfinite(v.w());
  }

  static double lpfAlphaFromCutoff(double dt, double cutoff_hz)
  {
    if (!(std::isfinite(dt) && dt > 0.0 && std::isfinite(cutoff_hz) && cutoff_hz > 0.0)) {
      return 1.0;
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    return std::clamp(dt / (tau + dt), 0.0, 1.0);
  }

  Eigen::Vector3d computeBodyRates(
    const Eigen::Vector3d & rpy,
    const Eigen::Vector3d & rpy_prev,
    double dt) const
  {
    if (!(std::isfinite(dt) && dt > 1e-6)) {
      return Eigen::Vector3d::Zero();
    }

    const double roll = rpy.x();
    const double pitch = rpy.y();
    const double roll_dot = wrapToPi(rpy.x() - rpy_prev.x()) / dt;
    const double pitch_dot = wrapToPi(rpy.y() - rpy_prev.y()) / dt;
    const double yaw_dot = wrapToPi(rpy.z() - rpy_prev.z()) / dt;

    Eigen::Matrix3d t;
    t <<
      1.0, 0.0, -std::sin(pitch),
      0.0, std::cos(roll), std::sin(roll) * std::cos(pitch),
      0.0, -std::sin(roll), std::cos(roll) * std::cos(pitch);

    return t * Eigen::Vector3d(roll_dot, pitch_dot, yaw_dot);
  }

  void rawCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto stamp = get_clock()->now();
    const auto mode = resolvedLayout(msg->data.size());
    if (mode == LayoutType::Basic32 && msg->data.size() < 32) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected 32-length raw log, got %zu", msg->data.size());
      return;
    }
    if (mode == LayoutType::Specific50 && msg->data.size() < 50) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected 50-length raw log, got %zu", msg->data.size());
      return;
    }
    if (mode == LayoutType::Specific49 && msg->data.size() < 49) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected 49-length raw log, got %zu", msg->data.size());
      return;
    }

    Eigen::Vector3d pos(msg->data[0], msg->data[1], msg->data[2]);
    Eigen::Vector3d rpy(msg->data[3], msg->data[4], msg->data[5]);
    tf2::Quaternion q;
    q.setRPY(rpy.x(), rpy.y(), rpy.z());

    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();
    Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();
    Eigen::Vector4d cmd_xyzyaw = Eigen::Vector4d::Zero();
    const double battery_status = msg->data[6];
    const double battery_raw = msg->data[7];
    const double battery_filtered = msg->data[8];

    if (mode == LayoutType::Basic32) {
      vel = Eigen::Vector3d(msg->data[11], msg->data[12], msg->data[13]);
      acc = Eigen::Vector3d(msg->data[14], msg->data[15], msg->data[16]);
      cmd_xyzyaw = Eigen::Vector4d(msg->data[7], msg->data[8], msg->data[9], msg->data[10]);
    } else if (mode == LayoutType::Specific50) {
      vel = Eigen::Vector3d(msg->data[29], msg->data[30], msg->data[31]);
      acc = Eigen::Vector3d(msg->data[32], msg->data[33], msg->data[34]);
      cmd_xyzyaw = Eigen::Vector4d(msg->data[25], msg->data[26], msg->data[27], msg->data[28]);
      cmd_vel = Eigen::Vector3d(msg->data[37], msg->data[38], msg->data[39]);
    } else {
      vel = Eigen::Vector3d(msg->data[13], msg->data[14], msg->data[15]);
      acc = Eigen::Vector3d(msg->data[16], msg->data[17], msg->data[18]);
      cmd_xyzyaw = Eigen::Vector4d(msg->data[9], msg->data[10], msg->data[11], msg->data[12]);
      body_rates = Eigen::Vector3d(msg->data[19], msg->data[20], msg->data[21]);
      cmd_vel = Eigen::Vector3d(msg->data[25], msg->data[26], msg->data[27]);
    }

    if (mode != LayoutType::Specific49 && have_prev_) {
      const double dt = (stamp - prev_stamp_).seconds();
      body_rates = computeBodyRates(rpy, prev_rpy_, dt);
    }

    const Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
    const Eigen::Matrix3d r_bw = q_eigen.toRotationMatrix();
    Eigen::Vector3d pos_filt = pos;
    if (position_lpf_enable_ && allFinite(pos)) {
      if (!pos_lpf_initialized_ || !have_prev_) {
        pos_filt_ = pos;
        pos_lpf_initialized_ = true;
      } else {
        const double dt = (stamp - prev_stamp_).seconds();
        const double alpha = lpfAlphaFromCutoff(dt, position_lpf_cutoff_hz_);
        pos_filt_ += alpha * (pos - pos_filt_);
      }
      pos_filt = pos_filt_;
    } else {
      pos_lpf_initialized_ = false;
    }
    const Eigen::Vector3d ee_pos = pos_filt + r_bw * ee_offset_body_;

    Eigen::Vector3d ee_vel = vel;
    Eigen::Vector3d ee_acc = acc;
    if (have_prev_) {
      const double dt = (stamp - prev_stamp_).seconds();
      if (std::isfinite(dt) && dt > 1e-6) {
        ee_vel = (ee_pos - prev_ee_pos_) / dt;
        ee_acc = (ee_vel - prev_ee_vel_) / dt;
      }
    }

    pub_pose_->publish(makePose(stamp, pos_filt, q));
    pub_vel_->publish(makeVec(stamp, vel, world_frame_));
    pub_acc_->publish(makeVec(stamp, acc, world_frame_));
    pub_angvel_->publish(makeVec(stamp, body_rates, drone_frame_));
    pub_ee_pose_->publish(makePose(stamp, ee_pos, q));
    pub_ee_vel_->publish(makeVec(stamp, ee_vel, world_frame_));
    pub_ee_acc_->publish(makeVec(stamp, ee_acc, world_frame_));
    pub_cmd_vel_->publish(makeVec(stamp, cmd_vel, drone_frame_));
    pub_battery_status_->publish(makeScalar(battery_status));
    pub_battery_raw_->publish(makeScalar(finiteOrFallback(battery_raw, battery_status)));
    pub_battery_filtered_->publish(makeScalar(finiteOrFallback(battery_filtered, battery_raw)));

    if (allFinite(cmd_xyzyaw)) {
      tf2::Quaternion q_cmd;
      q_cmd.setRPY(0.0, 0.0, cmd_xyzyaw[3]);
      const Eigen::Quaterniond q_cmd_eigen(q_cmd.w(), q_cmd.x(), q_cmd.y(), q_cmd.z());
      const Eigen::Vector3d cmd_drone_pos(cmd_xyzyaw[0], cmd_xyzyaw[1], cmd_xyzyaw[2]);
      const Eigen::Vector3d cmd_ee_pos = cmd_drone_pos + q_cmd_eigen.toRotationMatrix() * ee_offset_body_;
      pub_cmd_drone_->publish(makePose(stamp, cmd_drone_pos, q_cmd));
      pub_cmd_ee_->publish(makePose(stamp, cmd_ee_pos, q_cmd));
      pub_cmd_active_->publish(makePose(stamp, cmd_ee_pos, q_cmd));
    }

    if (mode == LayoutType::Specific50) {
      const Eigen::Vector3d world_force_raw(msg->data[12], msg->data[13], msg->data[14]);
      const Eigen::Vector3d world_force_scaled(
        finiteOrFallback(msg->data[15], msg->data[12]),
        finiteOrFallback(msg->data[16], msg->data[13]),
        finiteOrFallback(msg->data[17], msg->data[14]));
      const Eigen::Vector3d mob_force(
        finiteOrFallback(msg->data[18], world_force_scaled.x()),
        finiteOrFallback(msg->data[19], world_force_scaled.y()),
        finiteOrFallback(msg->data[20], world_force_scaled.z()));
      pub_online_world_force_raw_->publish(
        makeWrench(stamp, world_force_raw));
      pub_online_world_force_scaled_->publish(
        makeWrench(stamp, world_force_scaled));
      pub_online_force_->publish(
        makeWrench(stamp, mob_force));
    } else if (mode == LayoutType::Specific49) {
      const Eigen::Vector3d world_force_raw(msg->data[42], msg->data[43], msg->data[44]);
      const Eigen::Vector3d world_force_scaled(
        finiteOrFallback(msg->data[45], msg->data[42]),
        finiteOrFallback(msg->data[46], msg->data[43]),
        finiteOrFallback(msg->data[47], msg->data[44]));
      pub_online_world_force_raw_->publish(makeWrench(stamp, world_force_raw));
      pub_online_world_force_scaled_->publish(makeWrench(stamp, world_force_scaled));
      pub_online_force_->publish(makeWrench(stamp, world_force_scaled));
    }

    have_prev_ = true;
    prev_stamp_ = stamp;
    prev_rpy_ = rpy;
    prev_ee_pos_ = ee_pos;
    prev_ee_vel_ = ee_vel;
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_raw_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angvel_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ee_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ee_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ee_acc_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_drone_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_ee_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cmd_active_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_battery_status_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_battery_raw_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_battery_filtered_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_online_force_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_online_world_force_raw_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_online_world_force_scaled_;

  std::string raw_topic_;
  std::string layout_;
  std::string world_frame_;
  std::string drone_frame_;
  std::string ee_frame_;
  std::string cmd_drone_frame_;
  std::string cmd_ee_frame_;
  bool position_lpf_enable_{true};
  double position_lpf_cutoff_hz_{2.0};
  std::string pose_topic_;
  std::string vel_topic_;
  std::string acc_topic_;
  std::string angvel_topic_;
  std::string ee_pose_topic_;
  std::string ee_vel_topic_;
  std::string ee_acc_topic_;
  std::string cmd_drone_topic_;
  std::string cmd_ee_topic_;
  std::string cmd_active_topic_;
  std::string cmd_vel_topic_;
  std::string battery_status_topic_;
  std::string battery_raw_topic_;
  std::string battery_filtered_topic_;
  std::string online_force_topic_;
  std::string online_world_force_raw_topic_;
  std::string online_world_force_scaled_topic_;

  Eigen::Vector3d ee_offset_body_{0.09, 0.0, 0.085};
  bool pos_lpf_initialized_{false};
  Eigen::Vector3d pos_filt_{Eigen::Vector3d::Zero()};
  bool have_prev_{false};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d prev_rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_ee_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_ee_vel_{Eigen::Vector3d::Zero()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LogDecoder>());
  rclcpp::shutdown();
  return 0;
}
