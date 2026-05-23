// data_logging_debug.cpp
// 목적: suWrenchObs SI 디버그 로그를 별도 CSV + Float64MultiArray로 저장한다.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/status.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

using std::placeholders::_1;

static std::string expand_user_debug(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

static std::string now_mmddhhmm_debug()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[64];
  std::strftime(buf, sizeof(buf), "%m%d%H%M", &tm);
  return std::string(buf);
}

static inline double qnan_debug()
{
  return std::numeric_limits<double>::quiet_NaN();
}

class DataLoggingDebugNode : public rclcpp::Node
{
public:
  //  0.. 5 : pose_x y z roll pitch yaw
  //  6.. 9 : cmd_x y z yaw [m, rad]
  // 10..12 : fw_cmd_x y z [m]
  // 13..16 : thrust_f1 f2 f3 f4 [N]
  // 17..20 : pwm_1 2 3 4 [ratio]
  // 21..23 : body_force xyz [N]
  // 24..26 : world_force xyz [N]
  // 27..29 : body_torque xyz [N*m]
  // 30..32 : state_vel xyz [m/s]
  // 33..35 : pos_vel xyz [m/s]
  // 36..38 : acc xyz [m/s^2]
  // 39..41 : vel_des xyz [m/s]
  // 42..44 : att_des rpy [deg]
  // 45     : status_battery_voltage [V]
  // 46     : pm_vbat [V]
  // 47     : zero_bias_count
  // 48..50 : mob_force_none xyz [N]
  // 51..53 : mob_force_residual xyz [N]
  // 54..56 : mob_torque xyz [N*m]
  // 57..59 : mob_residual xyz [N*m]
  // 60..62 : body-frame accel xyz [G], after manual bias correction and before gravity-trim/LPF
  // 63..65 : body-frame gyro xyz [deg/s], Mahony/complementary gyro input
  static constexpr int kDataLen = 66;

  DataLoggingDebugNode()
  : Node("data_logging_debug")
  {
    csv_dir_ = expand_user_debug(this->declare_parameter<std::string>(
      "csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"));
    publish_topic_ = this->declare_parameter<std::string>("publish_topic", "/data_logging_msg_debug");
    cf_ns_ = this->declare_parameter<std::string>("cf_ns", "/cf2");
    loop_hz_ = this->declare_parameter<double>("loop_hz", 50.0);

    std::filesystem::create_directories(csv_dir_);
    csv_path_ = (std::filesystem::path(csv_dir_) / (now_mmddhhmm_debug() + "_debug.csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cf_ns_ + "/pose", 10, std::bind(&DataLoggingDebugNode::poseCallback, this, _1));
    sub_cmd_position_ = this->create_subscription<crazyflie_interfaces::msg::Position>(
      cf_ns_ + "/cmd_position", 10, std::bind(&DataLoggingDebugNode::cmdPositionCallback, this, _1));
    sub_fw_cmd_position_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_ctrl_target_pos", 10, std::bind(&DataLoggingDebugNode::fwCmdPositionCallback, this, _1));
    sub_status_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      cf_ns_ + "/status", 10, std::bind(&DataLoggingDebugNode::statusCallback, this, _1));
    sub_motor_thrust_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_motor_thrust", 10, std::bind(&DataLoggingDebugNode::motorThrustCallback, this, _1));
    sub_motor_pwm_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_motor_pwm", 10, std::bind(&DataLoggingDebugNode::motorPwmCallback, this, _1));
    sub_body_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_body_force", 10, std::bind(&DataLoggingDebugNode::bodyForceCallback, this, _1));
    sub_world_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_world_force", 10, std::bind(&DataLoggingDebugNode::worldForceCallback, this, _1));
    sub_body_torque_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_body_torque", 10, std::bind(&DataLoggingDebugNode::bodyTorqueCallback, this, _1));
    sub_state_vel_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_state_vel", 10, std::bind(&DataLoggingDebugNode::stateVelCallback, this, _1));
    sub_pos_vel_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_pos_vel", 10, std::bind(&DataLoggingDebugNode::posVelCallback, this, _1));
    sub_acc_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_acc", 10, std::bind(&DataLoggingDebugNode::accCallback, this, _1));
    sub_acc_raw_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_imu_acc_raw", 10, std::bind(&DataLoggingDebugNode::accRawCallback, this, _1));
    sub_gyro_raw_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_imu_gyro_raw", 10, std::bind(&DataLoggingDebugNode::gyroRawCallback, this, _1));
    sub_vel_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/vel_des", 10, std::bind(&DataLoggingDebugNode::velDesCallback, this, _1));
    sub_att_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/att_des", 10, std::bind(&DataLoggingDebugNode::attDesCallback, this, _1));
    sub_voltage_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_voltage", 10, std::bind(&DataLoggingDebugNode::voltageCallback, this, _1));
    sub_debug_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_debug", 10, std::bind(&DataLoggingDebugNode::debugCallback, this, _1));
    sub_mob_force_none_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_mob_force_none", 10, std::bind(&DataLoggingDebugNode::mobForceNoneCallback, this, _1));
    sub_mob_force_residual_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_mob_force_residual", 10, std::bind(&DataLoggingDebugNode::mobForceResidualCallback, this, _1));
    sub_mob_torque_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_mob_torque", 10, std::bind(&DataLoggingDebugNode::mobTorqueCallback, this, _1));
    sub_mob_residual_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_mob_residual", 10, std::bind(&DataLoggingDebugNode::mobResidualCallback, this, _1));

    RCLCPP_INFO(get_logger(), "data_logging_debug node started");
  }

  ~DataLoggingDebugNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  double loop_hz() const { return loop_hz_; }

  void loopOnce()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(kDataLen);

    push3(out, pose_xyz_);
    push3(out, pose_rpy_);
    push4(out, cmd_xyzyaw_);
    push3(out, fw_cmd_xyz_);
    push4(out, motor_thrust_);
    push4(out, motor_pwm_);
    push3(out, body_force_);
    push3(out, world_force_);
    push3(out, body_torque_);
    push3(out, state_vel_);
    push3(out, pos_vel_);
    push3(out, acc_);
    push3(out, vel_des_);
    push3(out, att_des_);
    out.data.push_back(status_batt_v_);
    out.data.push_back(pm_vbat_);
    out.data.push_back(zero_bias_count_);
    push3(out, mob_force_none_);
    push3(out, mob_force_residual_);
    push3(out, mob_torque_);
    push3(out, mob_residual_);
    push3(out, acc_raw_body_);
    push3(out, gyro_raw_body_);

    if (out.data.size() != static_cast<size_t>(kDataLen)) {
      out.data.resize(kDataLen, qnan_debug());
    }

    data_pub_->publish(out);
    log_csv_row(out);
  }

private:
  static void push3(std_msgs::msg::Float64MultiArray & m, const std::array<double, 3> & a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
  }

  static void push4(std_msgs::msg::Float64MultiArray & m, const std::array<double, 4> & a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
    m.data.push_back(a[3]);
  }

  void copy3(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg, std::array<double, 3> & dst)
  {
    if (msg->values.size() >= 3) {
      dst[0] = msg->values[0];
      dst[1] = msg->values[1];
      dst[2] = msg->values[2];
    }
  }

  void copy4(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg, std::array<double, 4> & dst)
  {
    if (msg->values.size() >= 4) {
      dst[0] = msg->values[0];
      dst[1] = msg->values[1];
      dst[2] = msg->values[2];
      dst[3] = msg->values[3];
    }
  }

  void write_csv_header()
  {
    csv_
      << "pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,"
      << "cmd_x,cmd_y,cmd_z,cmd_yaw,"
      << "fwCmd_x,fwCmd_y,fwCmd_z,"
      << "f1,f2,f3,f4,"
      << "pwm1,pwm2,pwm3,pwm4,"
      << "bodyFx,bodyFy,bodyFz,"
      << "worldFx,worldFy,worldFz,"
      << "bodyTx,bodyTy,bodyTz,"
      << "stateVx,stateVy,stateVz,"
      << "posVx,posVy,posVz,"
      << "accWx,accWy,accWz,"
      << "velDes_vx,velDes_vy,velDes_vz,"
      << "attDes_roll,attDes_pitch,attDes_yaw,"
      << "status_battery_voltage,pm_vbat,"
      << "zero_bias_count,"
      << "mobForceNone_x,mobForceNone_y,mobForceNone_z,"
      << "mobForceResidual_x,mobForceResidual_y,mobForceResidual_z,"
      << "mobTorque_x,mobTorque_y,mobTorque_z,"
      << "mobResidual_x,mobResidual_y,mobResidual_z,"
      << "accRawBody_x,accRawBody_y,accRawBody_z,"
      << "gyroBody_x,gyroBody_y,gyroBody_z\n";
    csv_.flush();
  }

  void log_csv_row(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_ << std::setprecision(10) << std::fixed;
    for (size_t i = 0; i < msg.data.size(); ++i) {
      if (i > 0) {
        csv_ << ",";
      }
      csv_ << msg.data[i];
    }
    csv_ << "\n";
    if (++csv_line_count_ % 100 == 0) {
      csv_.flush();
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_xyz_[0] = msg->pose.position.x;
    pose_xyz_[1] = msg->pose.position.y;
    pose_xyz_[2] = msg->pose.position.z;

    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_rpy_[0] = roll;
    pose_rpy_[1] = pitch;
    pose_rpy_[2] = yaw;
  }

  void motorThrustCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy4(msg, motor_thrust_); }
  void motorPwmCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy4(msg, motor_pwm_); }
  void bodyForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, body_force_); }
  void worldForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, world_force_); }
  void bodyTorqueCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, body_torque_); }
  void stateVelCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, state_vel_); }
  void posVelCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, pos_vel_); }
  void accCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, acc_); }
  void accRawCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, acc_raw_body_); }
  void gyroRawCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, gyro_raw_body_); }
  void velDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, vel_des_); }
  void attDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, att_des_); }
  void cmdPositionCallback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
  {
    cmd_xyzyaw_[0] = msg->x;
    cmd_xyzyaw_[1] = msg->y;
    cmd_xyzyaw_[2] = msg->z;
    cmd_xyzyaw_[3] = msg->yaw;
  }
  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    status_batt_v_ = msg->battery_voltage;
  }
  void fwCmdPositionCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    copy3(msg, fw_cmd_xyz_);
  }
  void voltageCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg->values.empty()) {
      pm_vbat_ = msg->values[0];
    }
  }

  void debugCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg->values.empty()) {
      zero_bias_count_ = msg->values[0];
    }
  }

  void mobForceNoneCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, mob_force_none_); }
  void mobForceResidualCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, mob_force_residual_); }
  void mobTorqueCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, mob_torque_); }
  void mobResidualCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, mob_residual_); }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr sub_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_fw_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_motor_thrust_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_motor_pwm_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_body_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_world_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_body_torque_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_state_vel_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_pos_vel_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_acc_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_acc_raw_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_gyro_raw_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_att_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_voltage_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_debug_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_mob_force_none_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_mob_force_residual_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_mob_torque_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_mob_residual_;

  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};

  std::string publish_topic_;
  std::string cf_ns_;
  double loop_hz_{50.0};

  std::array<double, 3> pose_xyz_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> pose_rpy_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> cmd_xyzyaw_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> fw_cmd_xyz_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> motor_thrust_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> motor_pwm_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> body_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> world_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> body_torque_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> state_vel_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> pos_vel_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> acc_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> acc_raw_body_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> gyro_raw_body_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> vel_des_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> att_des_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  double status_batt_v_ = qnan_debug();
  double pm_vbat_ = qnan_debug();
  double zero_bias_count_ = qnan_debug();
  std::array<double, 3> mob_force_none_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_force_residual_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_torque_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_residual_ = {qnan_debug(), qnan_debug(), qnan_debug()};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLoggingDebugNode>();
  rclcpp::Rate rate(node->loop_hz());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->loopOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
