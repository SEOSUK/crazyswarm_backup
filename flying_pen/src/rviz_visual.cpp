#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <deque>
#include <cmath>

using namespace std::chrono_literals;

class RvizVisual : public rclcpp::Node
{
public:
  RvizVisual()
  : Node("rviz_visual"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    history_sample_period_ = this->declare_parameter<double>("history_sample_period", 0.2);
    history_duration_ = this->declare_parameter<double>("history_duration", 30.0);
    ee_offset_ = declareOffsetParameter();

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10),
      rmw_qos_profile_sensor_data);

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/data_logging_msg", qos, std::bind(&RvizVisual::dataCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&RvizVisual::publishTfTimer, this));

    raw_cmd_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cmd_position_marker", 10);
    fw_cmd_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/fw_cmd_position_marker", 10);
    raw_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/force_raw_marker", 10);
    scaled_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/force_scaled_marker", 10);
    acc_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/acc_marker", 10);
    vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vel_marker", 10);
    wall_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/wall_marker", 10);
    ee_history_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ee_trajectory_history", 10);

    pos_.setZero();
    rpy_meas_.setZero();
    cmd_pos_.setZero();
    fw_cmd_pos_.setZero();
    world_force_raw_.setZero();
    world_force_scaled_.setZero();
    world_vel_.setZero();
    world_acc_.setZero();

    RCLCPP_INFO(get_logger(), "rviz_visual started. subscribing /data_logging_msg");
  }

private:
  void dataCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 52) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "msg size too small (%zu), expected >= 52", msg->data.size());
      return;
    }

    pos_[0] = msg->data[0];
    pos_[1] = msg->data[1];
    pos_[2] = msg->data[2];

    rpy_meas_[0] = msg->data[3];
    rpy_meas_[1] = msg->data[4];
    rpy_meas_[2] = msg->data[5];

    cmd_pos_[0] = msg->data[9];
    cmd_pos_[1] = msg->data[10];
    cmd_pos_[2] = msg->data[11];
    cmd_yaw_rad_ = msg->data[12];

    fw_cmd_pos_[0] = msg->data[13];
    fw_cmd_pos_[1] = msg->data[14];
    fw_cmd_pos_[2] = msg->data[15];

    world_vel_[0] = msg->data[16];
    world_vel_[1] = msg->data[17];
    world_vel_[2] = msg->data[18];

    world_acc_[0] = msg->data[19];
    world_acc_[1] = msg->data[20];
    world_acc_[2] = msg->data[21];

    world_force_raw_[0] = msg->data[45];
    world_force_raw_[1] = msg->data[46];
    world_force_raw_[2] = msg->data[47];

    world_force_scaled_[0] = msg->data[48];
    world_force_scaled_[1] = msg->data[49];
    world_force_scaled_[2] = msg->data[50];
  }

  void publishTfTimer()
  {
    const auto stamp = get_clock()->now();
    const auto ee_pos = computeEndEffectorPosition();

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

    geometry_msgs::msg::TransformStamped tf_cmd;
    tf_cmd.header.stamp = stamp;
    tf_cmd.header.frame_id = "world";
    tf_cmd.child_frame_id = "crazyflie_cmd";
    tf_cmd.transform.translation.x = cmd_pos_[0];
    tf_cmd.transform.translation.y = cmd_pos_[1];
    tf_cmd.transform.translation.z = cmd_pos_[2];

    tf2::Quaternion q_cmd;
    q_cmd.setRPY(0.0, 0.0, cmd_yaw_rad_);
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

    geometry_msgs::msg::Point p0;
    p0.x = pos_[0];
    p0.y = pos_[1];
    p0.z = pos_[2];

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

    publishArrow(raw_force_pub_, stamp, "world", "force_raw", 0, p0, world_force_raw_, 10.0, 0.02, 0.04, 0.06, 1.0f, 0.2f, 0.2f);
    publishArrow(scaled_force_pub_, stamp, "world", "force_scaled", 0, p0, world_force_scaled_, 10.0, 0.02, 0.04, 0.06, 0.7f, 0.0f, 0.8f);
    publishArrow(acc_pub_, stamp, "world", "acceleration", 0, p0, world_acc_, 0.5, 0.015, 0.03, 0.05, 0.0f, 0.0f, 1.0f);
    publishArrow(vel_pub_, stamp, "world", "velocity", 0, p0, world_vel_, 1.0, 0.015, 0.03, 0.05, 1.0f, 0.8f, 0.0f);
    publishWall(stamp);
    pushTrajectorySample(ee_pos, stamp);
    publishTrajectoryHistory(stamp);
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

  void pushTrajectorySample(const Eigen::Vector3d & ee_pos, const rclcpp::Time & stamp)
  {
    const double sample_period = std::max(1e-3, history_sample_period_);
    const double keep_duration = std::max(sample_period, history_duration_);

    while (!trajectory_history_.empty()) {
      const double age = (stamp - trajectory_history_.front().stamp).seconds();
      if (age <= keep_duration) {
        break;
      }
      trajectory_history_.pop_front();
    }

    if (
      last_history_sample_time_.nanoseconds() > 0 &&
      (stamp - last_history_sample_time_).seconds() < sample_period)
    {
      return;
    }

    geometry_msgs::msg::Point sample;
    sample.x = ee_pos.x();
    sample.y = ee_pos.y();
    sample.z = ee_pos.z();
    trajectory_history_.push_back(TrajectorySample{stamp, sample});
    last_history_sample_time_ = stamp;
  }

  void publishTrajectoryHistory(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = "ee_trajectory_history";
    marker.id = 1000;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = trajectory_history_.size() >= 2 ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    marker.scale.x = 0.01;
    marker.color.a = 0.65f;
    marker.color.r = 0.22f;
    marker.color.g = 0.22f;
    marker.color.b = 0.26f;
    marker.lifetime = rclcpp::Duration(0, 0);

    if (trajectory_history_.size() >= 2) {
      for (const auto & sample : trajectory_history_) {
        marker.points.push_back(sample.point);
      }
    }

    ee_history_pub_->publish(marker);
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
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fw_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scaled_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr acc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_history_pub_;

  Eigen::Vector3d pos_;
  Eigen::Vector3d rpy_meas_;
  Eigen::Vector3d cmd_pos_;
  Eigen::Vector3d fw_cmd_pos_;
  double cmd_yaw_rad_{0.0};
  Eigen::Vector3d world_force_raw_;
  Eigen::Vector3d world_force_scaled_;
  Eigen::Vector3d world_vel_;
  Eigen::Vector3d world_acc_;
  std::array<double, 3> ee_offset_;
  double history_sample_period_{0.2};
  double history_duration_{30.0};
  rclcpp::Time last_history_sample_time_{0, 0, RCL_ROS_TIME};
  std::deque<TrajectorySample> trajectory_history_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
