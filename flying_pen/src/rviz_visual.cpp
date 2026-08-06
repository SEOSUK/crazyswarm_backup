#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <deque>
#include <cmath>
#include <optional>
#include <vector>

using namespace std::chrono_literals;

namespace {
constexpr double kDegToRad = M_PI / 180.0;

bool isFiniteVector(const Eigen::Vector3d & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
}

class RvizVisual : public rclcpp::Node
{
public:
  struct HistorySample
  {
    int id{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point origin;
    geometry_msgs::msg::Quaternion orientation;
  };

  RvizVisual()
  : Node("rviz_visual"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    data_topic_ = this->declare_parameter<std::string>("topic", "/data_logging_msg_debug");
    ee_velocity_topic_ = this->declare_parameter<std::string>("ee_velocity_topic", "cf2/cf_su_ee_velocity");
    ee_cmd_topic_ = this->declare_parameter<std::string>("ee_cmd_topic", "cf2/cf_su_ee_cmd");
    history_sample_period_ = this->declare_parameter<double>("history_sample_period", 0.2);
    history_publish_period_ = this->declare_parameter<double>("history_publish_period", 0.10);
    history_duration_ = this->declare_parameter<double>("history_duration", 30.0);
    history_frame_axis_scale_ = this->declare_parameter<double>("history_frame_axis_scale", 0.3);
    ee_offset_ = declareOffsetParameter();

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10),
      rmw_qos_profile_sensor_data);

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      data_topic_, qos, std::bind(&RvizVisual::dataCallback, this, std::placeholders::_1));
    ee_velocity_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      ee_velocity_topic_, 10, std::bind(&RvizVisual::eeVelocityCallback, this, std::placeholders::_1));
    ee_cmd_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      ee_cmd_topic_, 10, std::bind(&RvizVisual::eeCmdCallback, this, std::placeholders::_1));

    clear_history_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/clear_history",
      std::bind(
        &RvizVisual::clearHistoryServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = this->create_wall_timer(10ms, std::bind(&RvizVisual::publishTfTimer, this));

    raw_cmd_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cmd_position_marker", 10);
    fw_cmd_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/fw_cmd_position_marker", 10);
    raw_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/mob_force_pure_marker", 10);
    scaled_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/mob_force_residual_marker", 10);
    normal_est_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/normal_est_marker", 10);
    acc_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/acc_marker", 10);
    vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vel_marker", 10);
    ee_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ee_vel_marker", 10);
    wall_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/wall_marker", 10);
    ee_history_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ee_trajectory_history", 10);
    contact_history_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rviz/contact_frame_history", 10);

    pos_.setZero();
    rpy_meas_.setZero();
    cmd_pos_.setZero();
    fw_cmd_pos_.setZero();
    mob_force_pure_.setZero();
    mob_force_residual_.setZero();
    normal_est_.setZero();
    world_vel_.setZero();
    world_acc_.setZero();
    ee_vel_used_.setZero();
    ee_cmd_pos_.setZero();

    RCLCPP_INFO(
      get_logger(),
      "rviz_visual started. subscribing %s, %s and %s",
      data_topic_.c_str(),
      ee_velocity_topic_.c_str(),
      ee_cmd_topic_.c_str());
  }

private:
  void dataCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 82) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "msg size too small (%zu), expected >= 82", msg->data.size());
      return;
    }

    pos_[0] = msg->data[0];
    pos_[1] = msg->data[1];
    pos_[2] = msg->data[2];

    rpy_meas_[0] = msg->data[3];
    rpy_meas_[1] = msg->data[4];
    rpy_meas_[2] = msg->data[5];

    cmd_pos_[0] = msg->data[6];
    cmd_pos_[1] = msg->data[7];
    cmd_pos_[2] = msg->data[8];
    cmd_yaw_deg_ = msg->data[9];

    fw_cmd_pos_[0] = msg->data[10];
    fw_cmd_pos_[1] = msg->data[11];
    fw_cmd_pos_[2] = msg->data[12];

    world_vel_[0] = msg->data[30];
    world_vel_[1] = msg->data[31];
    world_vel_[2] = msg->data[32];

    world_acc_[0] = msg->data[36];
    world_acc_[1] = msg->data[37];
    world_acc_[2] = msg->data[38];

    mob_force_pure_[0] = msg->data[48];
    mob_force_pure_[1] = msg->data[49];
    mob_force_pure_[2] = msg->data[50];

    mob_force_residual_[0] = msg->data[51];
    mob_force_residual_[1] = msg->data[52];
    mob_force_residual_[2] = msg->data[53];

    normal_est_[0] = msg->data[76];
    normal_est_[1] = msg->data[77];
    normal_est_[2] = msg->data[78];

    ee_vel_used_[0] = msg->data[79];
    ee_vel_used_[1] = msg->data[80];
    ee_vel_used_[2] = msg->data[81];

    pose_valid_ = isFiniteVector(pos_) && isFiniteVector(rpy_meas_);
  }

  void eeVelocityCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 3) {
      return;
    }

    ee_vel_used_[0] = msg->values[0];
    ee_vel_used_[1] = msg->values[1];
    ee_vel_used_[2] = msg->values[2];
  }

  void eeCmdCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 4) {
      return;
    }

    ee_cmd_pos_[0] = msg->values[0];
    ee_cmd_pos_[1] = msg->values[1];
    ee_cmd_pos_[2] = msg->values[2];
    ee_cmd_yaw_deg_ = msg->values[3];
    ee_cmd_valid_ = isFiniteVector(ee_cmd_pos_) && std::isfinite(ee_cmd_yaw_deg_);
  }

  void publishTfTimer()
  {
    const auto stamp = get_clock()->now();
    maybeHandleHistoryViewerReset(stamp);
    publishWall(stamp);
    pruneSmoothTrajectoryHistory(stamp);
    const auto expired_history_ids = pruneFrameHistory(stamp);

    if (!pose_valid_) {
      publishTrajectoryHistory(stamp);
      return;
    }

    const auto ee_pos = computeEndEffectorPosition();
    if (!isFiniteVector(ee_pos)) {
      publishTrajectoryHistory(stamp);
      return;
    }

    geometry_msgs::msg::TransformStamped tf_meas;
    tf_meas.header.stamp = stamp;
    tf_meas.header.frame_id = "world";
    tf_meas.child_frame_id = "crazyflie";
    tf_meas.transform.translation.x = pos_[0];
    tf_meas.transform.translation.y = pos_[1];
    tf_meas.transform.translation.z = pos_[2];

    tf2::Quaternion q_meas;
    q_meas.setRPY(rpy_meas_[0], rpy_meas_[1], rpy_meas_[2]);
    tf_meas.transform.rotation.x = q_meas.x();
    tf_meas.transform.rotation.y = q_meas.y();
    tf_meas.transform.rotation.z = q_meas.z();
    tf_meas.transform.rotation.w = q_meas.w();
    tf_broadcaster_->sendTransform(tf_meas);

    geometry_msgs::msg::TransformStamped tf_ee;
    tf_ee.header.stamp = stamp;
    tf_ee.header.frame_id = "world";
    tf_ee.child_frame_id = "end_effector";
    tf_ee.transform.translation.x = ee_pos.x();
    tf_ee.transform.translation.y = ee_pos.y();
    tf_ee.transform.translation.z = ee_pos.z();
    tf_ee.transform.rotation = tf_meas.transform.rotation;
    tf_broadcaster_->sendTransform(tf_ee);

    const auto normal_frame_axes = computeNormalFrameAxes();
    const auto normal_frame_quat = makeQuaternionFromAxes(normal_frame_axes);

    geometry_msgs::msg::TransformStamped tf_cmd;
    tf_cmd.header.stamp = stamp;
    tf_cmd.header.frame_id = "world";
    tf_cmd.child_frame_id = "crazyflie_cmd";
    tf_cmd.transform.translation.x = cmd_pos_[0];
    tf_cmd.transform.translation.y = cmd_pos_[1];
    tf_cmd.transform.translation.z = cmd_pos_[2];

    tf2::Quaternion q_cmd;
    q_cmd.setRPY(0.0, 0.0, cmd_yaw_deg_ * kDegToRad);
    tf_cmd.transform.rotation.x = q_cmd.x();
    tf_cmd.transform.rotation.y = q_cmd.y();
    tf_cmd.transform.rotation.z = q_cmd.z();
    tf_cmd.transform.rotation.w = q_cmd.w();
    tf_broadcaster_->sendTransform(tf_cmd);

    geometry_msgs::msg::TransformStamped tf_fw_cmd;
    tf_fw_cmd.header.stamp = stamp;
    tf_fw_cmd.header.frame_id = "world";
    tf_fw_cmd.child_frame_id = "crazyflie_fw_cmd";
    tf_fw_cmd.transform.translation.x = fw_cmd_pos_[0];
    tf_fw_cmd.transform.translation.y = fw_cmd_pos_[1];
    tf_fw_cmd.transform.translation.z = fw_cmd_pos_[2];
    tf_fw_cmd.transform.rotation = tf_cmd.transform.rotation;
    tf_broadcaster_->sendTransform(tf_fw_cmd);

    if (ee_cmd_valid_) {
      geometry_msgs::msg::TransformStamped tf_ee_cmd;
      tf_ee_cmd.header.stamp = stamp;
      tf_ee_cmd.header.frame_id = "world";
      tf_ee_cmd.child_frame_id = "end_effector_cmd";
      tf_ee_cmd.transform.translation.x = ee_cmd_pos_[0];
      tf_ee_cmd.transform.translation.y = ee_cmd_pos_[1];
      tf_ee_cmd.transform.translation.z = ee_cmd_pos_[2];

      tf2::Quaternion q_ee_cmd;
      q_ee_cmd.setRPY(0.0, 0.0, ee_cmd_yaw_deg_ * kDegToRad);
      tf_ee_cmd.transform.rotation.x = q_ee_cmd.x();
      tf_ee_cmd.transform.rotation.y = q_ee_cmd.y();
      tf_ee_cmd.transform.rotation.z = q_ee_cmd.z();
      tf_ee_cmd.transform.rotation.w = q_ee_cmd.w();
      tf_broadcaster_->sendTransform(tf_ee_cmd);
    }

    geometry_msgs::msg::Point p0;
    p0.x = pos_[0];
    p0.y = pos_[1];
    p0.z = pos_[2];

    geometry_msgs::msg::Point p_ee;
    p_ee.x = ee_pos.x();
    p_ee.y = ee_pos.y();
    p_ee.z = ee_pos.z();

    geometry_msgs::msg::Point p_cmd;
    p_cmd.x = cmd_pos_[0];
    p_cmd.y = cmd_pos_[1];
    p_cmd.z = cmd_pos_[2];

    geometry_msgs::msg::Point p_fw_cmd;
    p_fw_cmd.x = fw_cmd_pos_[0];
    p_fw_cmd.y = fw_cmd_pos_[1];
    p_fw_cmd.z = fw_cmd_pos_[2];

    publishSphere(raw_cmd_pub_, stamp, "world", "cmd_position", 0, p_cmd, 0.05, 0.0f, 0.45f, 0.90f, 0.85f);
    publishSphere(fw_cmd_pub_, stamp, "world", "fw_cmd_position", 0, p_fw_cmd, 0.06, 0.90f, 0.35f, 0.10f, 0.90f);

    Eigen::Vector3d normal_est_display = normal_est_;
    normal_est_display.z() *= 1.3;

    publishArrow(raw_force_pub_, stamp, "world", "mob_force_pure", 0, p0, mob_force_pure_, 10.0, 0.02, 0.04, 0.06, 1.0f, 0.2f, 0.2f);
    publishArrow(scaled_force_pub_, stamp, "world", "mob_force_residual", 0, p0, mob_force_residual_, 10.0, 0.02, 0.04, 0.06, 0.7f, 0.0f, 0.8f);
    publishArrow(normal_est_pub_, stamp, "world", "normal_estimation", 0, p0, normal_est_display, 0.35, 0.02, 0.04, 0.06, 0.1f, 0.8f, 0.2f);
    publishArrow(acc_pub_, stamp, "world", "acceleration", 0, p0, world_acc_, 0.5, 0.015, 0.03, 0.05, 0.0f, 0.0f, 1.0f);
    publishArrow(vel_pub_, stamp, "world", "velocity", 0, p0, world_vel_, 1.0, 0.015, 0.03, 0.05, 1.0f, 0.8f, 0.0f);
    publishArrow(ee_vel_pub_, stamp, "world", "ee_velocity", 0, p_ee, ee_vel_used_, 1.0, 0.015, 0.03, 0.05, 0.0f, 0.9f, 0.9f);
    pushSmoothTrajectorySample(ee_pos, stamp);
    const auto new_history_sample = pushFrameHistorySample(ee_pos, normal_frame_quat, stamp);
    publishFrameHistoryDelta(stamp, expired_history_ids, new_history_sample);
    maybePublishTrajectoryHistory(stamp);
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

  Eigen::Vector3d computeEndEffectorPosition() const
  {
    tf2::Quaternion q_meas;
    q_meas.setRPY(rpy_meas_[0], rpy_meas_[1], rpy_meas_[2]);
    tf2::Matrix3x3 rot(q_meas);
    const tf2::Vector3 offset_body(ee_offset_[0], ee_offset_[1], ee_offset_[2]);
    const tf2::Vector3 offset_world = rot * offset_body;

    return Eigen::Vector3d(
      pos_[0] + offset_world.x(),
      pos_[1] + offset_world.y(),
      pos_[2] + offset_world.z());
  }

  struct FrameAxes
  {
    Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};
    bool valid{false};
  };

  FrameAxes computeNormalFrameAxes() const
  {
    FrameAxes axes;

    Eigen::Vector3d x_axis = normal_est_;
    x_axis.z() *= 1.3;
    const double x_norm = x_axis.norm();
    if (x_norm < 1e-6) {
      return axes;
    }
    x_axis /= x_norm;

    Eigen::Vector3d t1_axis = Eigen::Vector3d::UnitZ().cross(x_axis);
    if (t1_axis.norm() < 1e-6) {
      t1_axis = Eigen::Vector3d::UnitY().cross(x_axis);
    }
    if (t1_axis.norm() < 1e-6) {
      return axes;
    }
    t1_axis.normalize();

    Eigen::Vector3d t2_axis = x_axis.cross(t1_axis);
    if (t2_axis.norm() < 1e-6) {
      return axes;
    }
    t2_axis.normalize();

    axes.x = x_axis;
    axes.y = t1_axis;
    axes.z = t2_axis;
    axes.valid = true;
    return axes;
  }

  geometry_msgs::msg::Quaternion makeQuaternionFromAxes(const FrameAxes & axes) const
  {
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.w = 1.0;
    if (!axes.valid) {
      return q_msg;
    }

    Eigen::Matrix3d rot;
    rot.col(0) = axes.x;
    rot.col(1) = axes.y;
    rot.col(2) = axes.z;
    const Eigen::Quaterniond q(rot);
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }

  void pruneSmoothTrajectoryHistory(const rclcpp::Time & stamp)
  {
    const double keep_duration = std::max(1e-3, history_duration_);
    while (!smooth_trajectory_history_.empty()) {
      const double age = (stamp - smooth_trajectory_history_.front().stamp).seconds();
      if (age <= keep_duration) {
        break;
      }
      smooth_trajectory_history_.pop_front();
    }
  }

  void maybeHandleHistoryViewerReset(const rclcpp::Time & stamp)
  {
    const size_t ee_history_subs = ee_history_pub_->get_subscription_count();
    const size_t contact_history_subs = contact_history_pub_->get_subscription_count();

    const bool ee_reconnected = last_ee_history_sub_count_ == 0 && ee_history_subs > 0;
    const bool contact_reconnected =
      last_contact_history_sub_count_ == 0 && contact_history_subs > 0;

    last_ee_history_sub_count_ = ee_history_subs;
    last_contact_history_sub_count_ = contact_history_subs;

    if (!ee_reconnected && !contact_reconnected) {
      return;
    }

    clearHistoryMarkers(stamp);
    clearHistoryState();
  }

  void clearHistoryState()
  {
    smooth_trajectory_history_.clear();
    frame_history_.clear();
    next_history_sample_id_ = 0;
    last_history_sample_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    last_history_publish_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  }

  void clearHistoryServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto stamp = get_clock()->now();
    clearHistoryMarkers(stamp);
    clearHistoryState();
    response->success = true;
    response->message = "rviz history cleared";
    RCLCPP_INFO(get_logger(), "Cleared RViz history via service call.");
  }

  void clearHistoryMarkers(const rclcpp::Time & stamp)
  {
    ee_history_pub_->publish(makeDeleteMarker("ee_trajectory_history", 1000, stamp));

    visualization_msgs::msg::MarkerArray out;
    for (const auto & sample : frame_history_) {
      out.markers.push_back(makeDeleteMarker("contact_frame_history_x_segment", sample.id, stamp));
      out.markers.push_back(makeDeleteMarker("contact_frame_history_y_segment", sample.id, stamp));
      out.markers.push_back(makeDeleteMarker("contact_frame_history_z_segment", sample.id, stamp));
    }
    if (!out.markers.empty()) {
      contact_history_pub_->publish(out);
    }
  }

  std::vector<int> pruneFrameHistory(const rclcpp::Time & stamp)
  {
    std::vector<int> expired_ids;
    const double keep_duration = std::max(1e-3, history_duration_);
    while (!frame_history_.empty()) {
      const double age = (stamp - frame_history_.front().stamp).seconds();
      if (age <= keep_duration) {
        break;
      }
      expired_ids.push_back(frame_history_.front().id);
      frame_history_.pop_front();
    }
    return expired_ids;
  }

  void pushSmoothTrajectorySample(const Eigen::Vector3d & ee_pos, const rclcpp::Time & stamp)
  {
    if (!isFiniteVector(ee_pos)) {
      return;
    }

    geometry_msgs::msg::Point sample;
    sample.x = ee_pos.x();
    sample.y = ee_pos.y();
    sample.z = ee_pos.z();
    smooth_trajectory_history_.push_back(TrajectorySample{stamp, sample});
  }

  std::optional<HistorySample> pushFrameHistorySample(
    const Eigen::Vector3d & ee_pos,
    const geometry_msgs::msg::Quaternion & frame_quat,
    const rclcpp::Time & stamp)
  {
    if (
      !isFiniteVector(ee_pos) ||
      !std::isfinite(frame_quat.x) ||
      !std::isfinite(frame_quat.y) ||
      !std::isfinite(frame_quat.z) ||
      !std::isfinite(frame_quat.w))
    {
      return std::nullopt;
    }

    const double sample_period = std::max(1e-3, history_sample_period_);

    if (
      last_history_sample_time_.nanoseconds() > 0 &&
      (stamp - last_history_sample_time_).seconds() < sample_period)
    {
      return std::nullopt;
    }

    HistorySample sample;
    sample.id = next_history_sample_id_++;
    sample.stamp = stamp;
    sample.origin.x = ee_pos.x();
    sample.origin.y = ee_pos.y();
    sample.origin.z = ee_pos.z();
    sample.orientation = frame_quat;
    frame_history_.push_back(sample);
    last_history_sample_time_ = stamp;
    return sample;
  }

  visualization_msgs::msg::Marker makeDeleteMarker(
    const std::string & ns,
    int id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  visualization_msgs::msg::Marker makeHistoryFrameLineMarker(
    const rclcpp::Time & stamp,
    const std::string & ns,
    int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.004;
    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeContactHistoryMarkerArray(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray out;
    if (smooth_trajectory_history_.size() >= 2) {
      auto traj_marker = makeHistoryFrameLineMarker(stamp, "ee_trajectory_history", 1000);
      traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      traj_marker.scale.x = 0.010;
      traj_marker.color.a = 0.65f;
      traj_marker.color.r = 0.22f;
      traj_marker.color.g = 0.22f;
      traj_marker.color.b = 0.26f;
      for (const auto & sample : smooth_trajectory_history_) {
        traj_marker.points.push_back(sample.point);
      }
      out.markers.push_back(traj_marker);
    } else {
      out.markers.push_back(makeDeleteMarker("ee_trajectory_history", 1000, stamp));
    }
    return out;
  }

  visualization_msgs::msg::MarkerArray makeDeleteFrameMarkers(
    const rclcpp::Time & stamp,
    const std::vector<int> & expired_ids) const
  {
    visualization_msgs::msg::MarkerArray out;
    for (const int id : expired_ids) {
      out.markers.push_back(makeDeleteMarker("contact_frame_history_x_segment", id, stamp));
      out.markers.push_back(makeDeleteMarker("contact_frame_history_y_segment", id, stamp));
      out.markers.push_back(makeDeleteMarker("contact_frame_history_z_segment", id, stamp));
    }
    return out;
  }

  visualization_msgs::msg::MarkerArray makeAddFrameMarkers(
    const rclcpp::Time & stamp,
    const HistorySample & sample) const
  {
    visualization_msgs::msg::MarkerArray out;

    const double axis_len = std::max(1e-3, history_frame_axis_scale_);
    tf2::Quaternion q(
      sample.orientation.x,
      sample.orientation.y,
      sample.orientation.z,
      sample.orientation.w);
    q.normalize();
    tf2::Matrix3x3 rot(q);

    geometry_msgs::msg::Point px = sample.origin;
    geometry_msgs::msg::Point py = sample.origin;
    geometry_msgs::msg::Point pz = sample.origin;

    const tf2::Vector3 ex = rot.getColumn(0);
    const tf2::Vector3 ey = rot.getColumn(1);
    const tf2::Vector3 ez = rot.getColumn(2);

    px.x += 2.0 * axis_len * ex.x();
    px.y += 2.0 * axis_len * ex.y();
    px.z += 2.0 * axis_len * ex.z();

    py.x += axis_len * ey.x();
    py.y += axis_len * ey.y();
    py.z += axis_len * ey.z();

    pz.x += axis_len * ez.x();
    pz.y += axis_len * ez.y();
    pz.z += axis_len * ez.z();

    auto x_marker = makeHistoryFrameLineMarker(stamp, "contact_frame_history_x_segment", sample.id);
    auto y_marker = makeHistoryFrameLineMarker(stamp, "contact_frame_history_y_segment", sample.id);
    auto z_marker = makeHistoryFrameLineMarker(stamp, "contact_frame_history_z_segment", sample.id);

    x_marker.scale.x = 0.0045;
    x_marker.color.a = 0.9f;
    x_marker.color.r = 1.0f;
    x_marker.color.g = 0.2f;
    x_marker.color.b = 0.2f;
    x_marker.points.push_back(sample.origin);
    x_marker.points.push_back(px);

    y_marker.scale.x = 0.0035;
    y_marker.color.a = 0.8f;
    y_marker.color.r = 0.2f;
    y_marker.color.g = 1.0f;
    y_marker.color.b = 0.2f;
    y_marker.points.push_back(sample.origin);
    y_marker.points.push_back(py);

    z_marker.scale.x = 0.0035;
    z_marker.color.a = 0.8f;
    z_marker.color.r = 0.2f;
    z_marker.color.g = 0.4f;
    z_marker.color.b = 1.0f;
    z_marker.points.push_back(sample.origin);
    z_marker.points.push_back(pz);

    out.markers.push_back(x_marker);
    out.markers.push_back(y_marker);
    out.markers.push_back(z_marker);
    return out;
  }

  void publishFrameHistoryDelta(
    const rclcpp::Time & stamp,
    const std::vector<int> & expired_ids,
    const std::optional<HistorySample> & new_sample)
  {
    if (!expired_ids.empty()) {
      contact_history_pub_->publish(makeDeleteFrameMarkers(stamp, expired_ids));
    }
    if (new_sample.has_value()) {
      contact_history_pub_->publish(makeAddFrameMarkers(stamp, *new_sample));
    }
  }

  void publishTrajectoryHistory(const rclcpp::Time & stamp)
  {
    ee_history_pub_->publish(makeDeleteMarker("ee_trajectory_history", 1000, stamp));
    contact_history_pub_->publish(makeContactHistoryMarkerArray(stamp));
  }

  void maybePublishTrajectoryHistory(const rclcpp::Time & stamp)
  {
    const double publish_period = std::max(1e-3, history_publish_period_);
    if (
      last_history_publish_time_.nanoseconds() > 0 &&
      (stamp - last_history_publish_time_).seconds() < publish_period)
    {
      return;
    }

    publishTrajectoryHistory(stamp);
    last_history_publish_time_ = stamp;
  }

  void publishArrow(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    const geometry_msgs::msg::Point& start,
    const Eigen::Vector3d& vec,
    double scale_factor,
    double sx,
    double sy,
    double sz,
    float r,
    float g,
    float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point end = start;
    end.x += vec.x() * scale_factor;
    end.y += vec.y() * scale_factor;
    end.z += vec.z() * scale_factor;

    marker.points = {start, end};
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);
    pub->publish(marker);
  }

  void publishSphere(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    const geometry_msgs::msg::Point& center,
    double scale,
    float r,
    float g,
    float b,
    float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration(0, 0);
    pub->publish(marker);
  }

  void publishWall(const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = "wall";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.8;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 1.0;
    marker.scale.z = 0.6;
    marker.color.r = 0.3f;
    marker.color.g = 0.3f;
    marker.color.b = 1.0f;
    marker.color.a = 0.3f;
    marker.lifetime = rclcpp::Duration(0, 0);
    wall_pub_->publish(marker);
  }

  struct TrajectorySample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Point point;
  };

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr ee_velocity_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr ee_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_history_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fw_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scaled_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr normal_est_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr acc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_history_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contact_history_pub_;

  Eigen::Vector3d pos_;
  Eigen::Vector3d rpy_meas_;
  Eigen::Vector3d cmd_pos_;
  Eigen::Vector3d fw_cmd_pos_;
  double cmd_yaw_deg_{0.0};
  Eigen::Vector3d mob_force_pure_;
  Eigen::Vector3d mob_force_residual_;
  Eigen::Vector3d normal_est_;
  Eigen::Vector3d world_vel_;
  Eigen::Vector3d world_acc_;
  Eigen::Vector3d ee_vel_used_;
  Eigen::Vector3d ee_cmd_pos_;
  std::array<double, 3> ee_offset_;
  std::string data_topic_;
  std::string ee_velocity_topic_;
  std::string ee_cmd_topic_;
  double ee_cmd_yaw_deg_{0.0};
  bool ee_cmd_valid_{false};
  double history_sample_period_{0.2};
  double history_publish_period_{0.10};
  double history_duration_{30.0};
  double history_frame_axis_scale_{0.3};
  int next_history_sample_id_{0};
  size_t last_ee_history_sub_count_{0};
  size_t last_contact_history_sub_count_{0};
  rclcpp::Time last_history_sample_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_history_publish_time_{0, 0, RCL_ROS_TIME};
  std::deque<TrajectorySample> smooth_trajectory_history_;
  std::deque<HistorySample> frame_history_;
  bool pose_valid_{false};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
