#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <deque>
#include <utility>
#include <mutex>
#include <string>
#include <vector>

class RvizVisual : public rclcpp::Node
{
public:
  RvizVisual()
  : Node("rviz_visual")
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "world");
    cf_frame_ = declare_parameter<std::string>("cf_frame", "crazyflie");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "end_effector");
    cmd_drone_frame_ = declare_parameter<std::string>("cmd_drone_frame", "cmd_drone");
    cmd_ee_frame_ = declare_parameter<std::string>("cmd_ee_frame", "cmd_end_effector");

    pose_topic_ = declare_parameter<std::string>("pose_topic", "/log_player/out/pose");
    ee_pose_topic_ = declare_parameter<std::string>("ee_pose_topic", "/log_player/out/ee_pose");
    cmd_drone_topic_ = declare_parameter<std::string>("cmd_drone_topic", "/log_player/out/cmd_drone");
    cmd_ee_topic_ = declare_parameter<std::string>("cmd_ee_topic", "/log_player/out/cmd_ee");
    online_force_topic_ = declare_parameter<std::string>("online_force_topic", "/log_player/online/force_estimate");
    offline_force_source_ = declare_parameter<std::string>("offline_force_source", "consistency");
    offline_normal_quat_topic_pure_ = declare_parameter<std::string>(
      "offline_normal_quat_topic_pure", "/log_player/offline/contact_frame_quat_pure");
    offline_normal_quat_topic_k1_ = declare_parameter<std::string>(
      "offline_normal_quat_topic_k1", "/log_player/offline/contact_frame_quat");
    offline_normal_quat_topic_k2_ = declare_parameter<std::string>(
      "offline_normal_quat_topic_k2", "/log_player/offline/contact_frame_quat_k2");
    offline_normal_metrics_topic_pure_ = declare_parameter<std::string>(
      "offline_normal_metrics_topic_pure", "/log_player/offline/normal_debug_metrics_pure");
    offline_normal_metrics_topic_k1_ = declare_parameter<std::string>(
      "offline_normal_metrics_topic_k1", "/log_player/offline/normal_debug_metrics");
    offline_normal_metrics_topic_k2_ = declare_parameter<std::string>(
      "offline_normal_metrics_topic_k2", "/log_player/offline/normal_debug_metrics_k2");
    auto ee_offset_param = declare_parameter<std::vector<double>>(
      "end_effector_offset", std::vector<double>{0.1, 0.0, 0.04});
    auto true_normal_param = declare_parameter<std::vector<double>>(
      "true_normal_world", std::vector<double>{1.0, 0.0, 0.0});

    arrow_scale_ = declare_parameter<double>("arrow_scale", 10.0);
    normal_scale_ = declare_parameter<double>("normal_scale", 0.20);
    publish_hz_ = declare_parameter<double>("publish_hz", 30.0);
    wall_x_offset_ = declare_parameter<double>("wall_x_offset", 0.0);
    wall_y_width_ = declare_parameter<double>("wall_y_width", 1.0);
    wall_z_height_ = declare_parameter<double>("wall_z_height", 1.5);
    wall_x_thickness_ = declare_parameter<double>("wall_x_thickness", 0.01);
    ee_history_enabled_ = declare_parameter<bool>("ee_history_enabled", true);
    ee_history_duration_sec_ = declare_parameter<double>("ee_history_duration_sec", 20.0);
    ee_history_width_ = declare_parameter<double>("ee_history_width", 0.01);

    parameterCallbackHandle_ = add_on_set_parameters_callback(
      std::bind(&RvizVisual::onParametersSet, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10, std::bind(&RvizVisual::poseCb, this, std::placeholders::_1));
    sub_ee_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10, std::bind(&RvizVisual::eePoseCb, this, std::placeholders::_1));
    sub_cmd_drone_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_drone_topic_, 10, std::bind(&RvizVisual::cmdDroneCb, this, std::placeholders::_1));
    sub_cmd_ee_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      cmd_ee_topic_, 10, std::bind(&RvizVisual::cmdEeCb, this, std::placeholders::_1));
    sub_online_force_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      online_force_topic_, 10, std::bind(&RvizVisual::onlineForceCb, this, std::placeholders::_1));
    sub_offline_force_1st_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/log_player/offline/ee_applied_mob", 10, std::bind(&RvizVisual::offlineForce1stCb, this, std::placeholders::_1));
    sub_offline_force_2nd_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/log_player/offline/ee_applied_mob_2nd", 10, std::bind(&RvizVisual::offlineForce2ndCb, this, std::placeholders::_1));
    sub_offline_force_consistency_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/log_player/offline/ee_applied_mob_2nd_tau", 10, std::bind(&RvizVisual::offlineForceConsistencyCb, this, std::placeholders::_1));
    sub_offline_force_consistency_alt_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/log_player/offline/ee_applied_mob_2nd_tau_ke2", 10, std::bind(&RvizVisual::offlineForceConsistencyAltCb, this, std::placeholders::_1));
    sub_offline_normal_quat_pure_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      offline_normal_quat_topic_pure_, 10,
      std::bind(&RvizVisual::offlineNormalQuatPureCb, this, std::placeholders::_1));
    sub_offline_normal_quat_k1_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      offline_normal_quat_topic_k1_, 10,
      std::bind(&RvizVisual::offlineNormalQuatK1Cb, this, std::placeholders::_1));
    sub_offline_normal_quat_k2_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      offline_normal_quat_topic_k2_, 10,
      std::bind(&RvizVisual::offlineNormalQuatK2Cb, this, std::placeholders::_1));
    sub_offline_normal_metrics_pure_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      offline_normal_metrics_topic_pure_, 10,
      std::bind(&RvizVisual::offlineNormalMetricsPureCb, this, std::placeholders::_1));
    sub_offline_normal_metrics_k1_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      offline_normal_metrics_topic_k1_, 10,
      std::bind(&RvizVisual::offlineNormalMetricsK1Cb, this, std::placeholders::_1));
    sub_offline_normal_metrics_k2_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      offline_normal_metrics_topic_k2_, 10,
      std::bind(&RvizVisual::offlineNormalMetricsK2Cb, this, std::placeholders::_1));

    pub_online_force_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/online_force_estimate", 10);
    pub_offline_force_pure_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_force_pure", 10);
    pub_offline_force_k1_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_force_k1", 10);
    pub_offline_force_k2_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_force_k2", 10);
    pub_offline_rxf_pure_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_rxf_pure", 10);
    pub_offline_rxf_k1_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_rxf_k1", 10);
    pub_offline_rxf_k2_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_rxf_k2", 10);
    pub_offline_normal_pure_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_normal_pure", 10);
    pub_offline_normal_k1_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_normal_k1", 10);
    pub_offline_normal_k2_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_normal_k2", 10);
    pub_offline_normal_true_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/offline_normal_true", 10);
    pub_wall_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/contact_wall", 10);
    pub_ee_history_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/rviz/ee_history", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1e-3, publish_hz_)));
    timer_ = create_wall_timer(period, std::bind(&RvizVisual::loop, this));

    if (ee_offset_param.size() == 3) {
      ee_offset_body_ = Eigen::Vector3d(ee_offset_param[0], ee_offset_param[1], ee_offset_param[2]);
    }
    if (true_normal_param.size() == 3) {
      true_normal_world_ = Eigen::Vector3d(true_normal_param[0], true_normal_param[1], true_normal_param[2]);
    }
  }

private:
  visualization_msgs::msg::Marker makeArrow(
    const rclcpp::Time & stamp,
    const std::string & ns,
    int id,
    const geometry_msgs::msg::Point & origin,
    const Eigen::Vector3d & vec,
    const std::array<float, 4> & rgba,
    double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parent_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.scale.z = 0.03;
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = rgba[3];

    geometry_msgs::msg::Point tip = origin;
    tip.x += scale * vec.x();
    tip.y += scale * vec.y();
    tip.z += scale * vec.z();
    marker.points.push_back(origin);
    marker.points.push_back(tip);
    return marker;
  }

  visualization_msgs::msg::Marker makeWallMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parent_frame_;
    marker.header.stamp = stamp;
    marker.ns = "contact_wall";
    marker.id = 10;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = wall_x_offset_;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = wall_z_height_ * 0.5;
    marker.scale.x = wall_x_thickness_;
    marker.scale.y = wall_y_width_;
    marker.scale.z = wall_z_height_;
    marker.color.r = 0.95f;
    marker.color.g = 0.75f;
    marker.color.b = 0.20f;
    marker.color.a = 0.25f;
    return marker;
  }

  visualization_msgs::msg::Marker makeEeHistoryMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parent_frame_;
    marker.header.stamp = stamp;
    marker.ns = "ee_history";
    marker.id = 11;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = ee_history_width_;
    marker.color.r = 1.0f;
    marker.color.g = 0.4f;
    marker.color.b = 0.1f;
    marker.color.a = 0.9f;
    for (const auto & entry : ee_history_points_) {
      marker.points.push_back(entry.second);
    }
    return marker;
  }

  geometry_msgs::msg::TransformStamped makeTf(
    const rclcpp::Time & stamp,
    const std::string & child_frame,
    const geometry_msgs::msg::Pose & pose) const
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    return tf;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    pose_ = *msg;
    have_pose_ = true;
  }

  void eePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    ee_pose_ = *msg;
    have_ee_pose_ = true;
    if (ee_history_enabled_) {
      const rclcpp::Time msg_stamp(msg->header.stamp);
      ee_history_points_.emplace_back(msg_stamp, msg->pose.position);
      const rclcpp::Duration max_age =
        rclcpp::Duration::from_seconds(std::max(1e-3, ee_history_duration_sec_));
      while (
        !ee_history_points_.empty() &&
        (msg_stamp - ee_history_points_.front().first) > max_age)
      {
        ee_history_points_.pop_front();
      }
    }
  }

  void cmdDroneCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cmd_drone_pose_ = *msg;
    have_cmd_drone_ = true;
  }

  void cmdEeCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cmd_ee_pose_ = *msg;
    have_cmd_ee_ = true;
  }

  void onlineForceCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    online_force_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    have_online_force_ = true;
  }

  void offlineForce1stCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    offline_force_1st_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    have_offline_force_1st_ = true;
  }

  void offlineForce2ndCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    offline_force_2nd_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    have_offline_force_2nd_ = true;
  }

  void offlineForceConsistencyCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    offline_force_consistency_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    have_offline_force_consistency_ = true;
  }

  void offlineForceConsistencyAltCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    offline_force_consistency_alt_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    have_offline_force_consistency_alt_ = true;
  }

  void updateNormalQuat(
    const geometry_msgs::msg::QuaternionStamped::SharedPtr msg,
    geometry_msgs::msg::Quaternion & quat_out,
    bool & have_quat_out)
  {
    quat_out = msg->quaternion;
    have_quat_out = true;
  }

  void updateNormalMetrics(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg,
    Eigen::Vector3d & normal_out,
    bool & have_normal_out)
  {
    if (msg->data.size() < 34) {
      return;
    }
    normal_out = Eigen::Vector3d(msg->data[31], msg->data[32], msg->data[33]);
    have_normal_out = std::isfinite(normal_out.x()) && std::isfinite(normal_out.y()) &&
      std::isfinite(normal_out.z()) && normal_out.norm() > 1e-9;
  }

  void offlineNormalQuatPureCb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalQuat(msg, offline_normal_quat_pure_, have_offline_normal_quat_pure_);
  }

  void offlineNormalQuatK1Cb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalQuat(msg, offline_normal_quat_k1_, have_offline_normal_quat_k1_);
  }

  void offlineNormalQuatK2Cb(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalQuat(msg, offline_normal_quat_k2_, have_offline_normal_quat_k2_);
  }

  void offlineNormalMetricsPureCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalMetrics(msg, offline_normal_from_metrics_pure_, have_offline_normal_metrics_pure_);
  }

  void offlineNormalMetricsK1Cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalMetrics(msg, offline_normal_from_metrics_k1_, have_offline_normal_metrics_k1_);
  }

  void offlineNormalMetricsK2Cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateNormalMetrics(msg, offline_normal_from_metrics_k2_, have_offline_normal_metrics_k2_);
  }

  bool resolveNormalVector(
    const geometry_msgs::msg::Quaternion & quat,
    bool have_quat,
    const Eigen::Vector3d & metrics_normal,
    bool have_metrics,
    Eigen::Vector3d & normal_vec) const
  {
    if (have_metrics) {
      normal_vec = metrics_normal;
      return true;
    }
    if (!have_quat) {
      return false;
    }

    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 rot(q);
    normal_vec = Eigen::Vector3d(rot[0][0], rot[1][0], rot[2][0]);
    return std::isfinite(normal_vec.x()) && std::isfinite(normal_vec.y()) &&
      std::isfinite(normal_vec.z()) && normal_vec.norm() > 1e-9;
  }

  void loop()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const auto stamp = now();
    pub_wall_marker_->publish(makeWallMarker(stamp));
    if (ee_history_enabled_ && ee_history_points_.size() >= 2) {
      pub_ee_history_marker_->publish(makeEeHistoryMarker(stamp));
    }
    if (have_pose_) {
      tf_broadcaster_->sendTransform(makeTf(stamp, cf_frame_, pose_.pose));
    }
    if (have_cmd_drone_) {
      tf_broadcaster_->sendTransform(makeTf(stamp, cmd_drone_frame_, cmd_drone_pose_.pose));
    }
    if (have_cmd_ee_) {
      tf_broadcaster_->sendTransform(makeTf(stamp, cmd_ee_frame_, cmd_ee_pose_.pose));
    }

    geometry_msgs::msg::Point origin;
    if (have_ee_pose_) {
      origin = ee_pose_.pose.position;
    } else if (have_pose_) {
      origin = pose_.pose.position;
    } else {
      return;
    }

    if (have_online_force_) {
      pub_online_force_marker_->publish(makeArrow(
        stamp, "online_force", 0, origin, online_force_,
        {0.2f, 0.8f, 1.0f, 0.95f}, arrow_scale_));
    }
    Eigen::Vector3d r_world = Eigen::Vector3d::Zero();
    bool have_r_world = false;
    if (have_pose_ && have_ee_pose_) {
      r_world = Eigen::Vector3d(
        ee_pose_.pose.position.x - pose_.pose.position.x,
        ee_pose_.pose.position.y - pose_.pose.position.y,
        ee_pose_.pose.position.z - pose_.pose.position.z);
      have_r_world = r_world.norm() > 1e-9;
    } else if (have_pose_) {
      const auto & q = pose_.pose.orientation;
      const Eigen::Quaterniond q_wb(q.w, q.x, q.y, q.z);
      r_world = q_wb.toRotationMatrix() * ee_offset_body_;
      have_r_world = r_world.norm() > 1e-9;
    }

    const std::array<float, 4> pure_rgba{0.15f, 0.45f, 0.95f, 0.95f};
    const std::array<float, 4> k1_rgba{0.95f, 0.35f, 0.10f, 0.95f};
    const std::array<float, 4> k2_rgba{0.45f, 0.70f, 0.20f, 0.95f};

    if (have_offline_force_2nd_) {
      pub_offline_force_pure_marker_->publish(makeArrow(
        stamp, "offline_force_pure", 0, origin, offline_force_2nd_, pure_rgba, arrow_scale_));
      if (have_r_world) {
        pub_offline_rxf_pure_marker_->publish(makeArrow(
          stamp, "offline_rxf_pure", 0, origin, r_world.cross(offline_force_2nd_), pure_rgba, arrow_scale_));
      }
    }
    if (have_offline_force_consistency_) {
      pub_offline_force_k1_marker_->publish(makeArrow(
        stamp, "offline_force_k1", 0, origin, offline_force_consistency_, k1_rgba, arrow_scale_));
      if (have_r_world) {
        pub_offline_rxf_k1_marker_->publish(makeArrow(
          stamp, "offline_rxf_k1", 0, origin, r_world.cross(offline_force_consistency_), k1_rgba, arrow_scale_));
      }
    }
    if (have_offline_force_consistency_alt_) {
      pub_offline_force_k2_marker_->publish(makeArrow(
        stamp, "offline_force_k2", 0, origin, offline_force_consistency_alt_, k2_rgba, arrow_scale_));
      if (have_r_world) {
        pub_offline_rxf_k2_marker_->publish(makeArrow(
          stamp, "offline_rxf_k2", 0, origin, r_world.cross(offline_force_consistency_alt_), k2_rgba, arrow_scale_));
      }
    }

    Eigen::Vector3d normal_vec = Eigen::Vector3d::Zero();
    if (resolveNormalVector(
        offline_normal_quat_pure_, have_offline_normal_quat_pure_,
        offline_normal_from_metrics_pure_, have_offline_normal_metrics_pure_, normal_vec))
    {
      pub_offline_normal_pure_marker_->publish(makeArrow(
        stamp, "offline_normal_pure", 0, origin, normal_vec.normalized(),
        pure_rgba, normal_scale_));
    }
    if (resolveNormalVector(
        offline_normal_quat_k1_, have_offline_normal_quat_k1_,
        offline_normal_from_metrics_k1_, have_offline_normal_metrics_k1_, normal_vec))
    {
      pub_offline_normal_k1_marker_->publish(makeArrow(
        stamp, "offline_normal_k1", 0, origin, normal_vec.normalized(),
        k1_rgba, normal_scale_));
    }
    if (resolveNormalVector(
        offline_normal_quat_k2_, have_offline_normal_quat_k2_,
        offline_normal_from_metrics_k2_, have_offline_normal_metrics_k2_, normal_vec))
    {
      pub_offline_normal_k2_marker_->publish(makeArrow(
        stamp, "offline_normal_k2", 0, origin, normal_vec.normalized(),
        k2_rgba, normal_scale_));
    }
    if (true_normal_world_.norm() > 1e-9) {
      pub_offline_normal_true_marker_->publish(makeArrow(
        stamp, "offline_normal_true", 0, origin, true_normal_world_.normalized(),
        {1.0f, 1.0f, 0.2f, 0.95f}, normal_scale_));
    }
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "wall_x_offset") {
        wall_x_offset_ = parameter.as_double();
      } else if (parameter.get_name() == "wall_y_width") {
        wall_y_width_ = parameter.as_double();
      } else if (parameter.get_name() == "wall_z_height") {
        wall_z_height_ = parameter.as_double();
      } else if (parameter.get_name() == "wall_x_thickness") {
        wall_x_thickness_ = parameter.as_double();
      } else if (parameter.get_name() == "ee_history_enabled") {
        ee_history_enabled_ = parameter.as_bool();
        if (!ee_history_enabled_) {
          ee_history_points_.clear();
        }
      } else if (parameter.get_name() == "ee_history_duration_sec") {
        ee_history_duration_sec_ = parameter.as_double();
      } else if (parameter.get_name() == "ee_history_width") {
        ee_history_width_ = parameter.as_double();
      }
    }

    if (
      wall_y_width_ <= 0.0 || wall_z_height_ <= 0.0 || wall_x_thickness_ <= 0.0 ||
      ee_history_duration_sec_ <= 0.0 || ee_history_width_ <= 0.0)
    {
      result.successful = false;
      result.reason = "marker dimensions and ee_history_duration_sec must be greater than zero";
    }
    return result;
  }

  std::mutex mtx_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ee_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_drone_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_ee_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_online_force_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_offline_force_1st_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_offline_force_2nd_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_offline_force_consistency_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_offline_force_consistency_alt_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_offline_normal_quat_pure_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_offline_normal_quat_k1_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_offline_normal_quat_k2_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_offline_normal_metrics_pure_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_offline_normal_metrics_k1_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_offline_normal_metrics_k2_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_online_force_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_force_pure_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_force_k1_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_force_k2_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_rxf_pure_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_rxf_k1_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_rxf_k2_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_normal_pure_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_normal_k1_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_normal_k2_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_offline_normal_true_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_wall_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ee_history_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string parent_frame_;
  std::string cf_frame_;
  std::string ee_frame_;
  std::string cmd_drone_frame_;
  std::string cmd_ee_frame_;
  std::string pose_topic_;
  std::string ee_pose_topic_;
  std::string cmd_drone_topic_;
  std::string cmd_ee_topic_;
  std::string online_force_topic_;
  std::string offline_force_source_;
  std::string offline_normal_quat_topic_pure_;
  std::string offline_normal_quat_topic_k1_;
  std::string offline_normal_quat_topic_k2_;
  std::string offline_normal_metrics_topic_pure_;
  std::string offline_normal_metrics_topic_k1_;
  std::string offline_normal_metrics_topic_k2_;
  double arrow_scale_{10.0};
  double normal_scale_{0.2};
  double publish_hz_{30.0};
  double wall_x_offset_{0.0};
  double wall_y_width_{1.0};
  double wall_z_height_{1.5};
  double wall_x_thickness_{0.01};
  bool ee_history_enabled_{true};
  double ee_history_duration_sec_{20.0};
  double ee_history_width_{0.01};
  Eigen::Vector3d ee_offset_body_{0.1, 0.0, 0.04};
  Eigen::Vector3d true_normal_world_{1.0, 0.0, 0.0};

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PoseStamped ee_pose_;
  geometry_msgs::msg::PoseStamped cmd_drone_pose_;
  geometry_msgs::msg::PoseStamped cmd_ee_pose_;
  geometry_msgs::msg::Quaternion offline_normal_quat_pure_;
  geometry_msgs::msg::Quaternion offline_normal_quat_k1_;
  geometry_msgs::msg::Quaternion offline_normal_quat_k2_;
  Eigen::Vector3d online_force_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_force_1st_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_force_2nd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_force_consistency_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_force_consistency_alt_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_normal_from_metrics_pure_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_normal_from_metrics_k1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offline_normal_from_metrics_k2_{Eigen::Vector3d::Zero()};
  std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Point>> ee_history_points_;
  bool have_pose_{false};
  bool have_ee_pose_{false};
  bool have_cmd_drone_{false};
  bool have_cmd_ee_{false};
  bool have_online_force_{false};
  bool have_offline_force_1st_{false};
  bool have_offline_force_2nd_{false};
  bool have_offline_force_consistency_{false};
  bool have_offline_force_consistency_alt_{false};
  bool have_offline_normal_quat_pure_{false};
  bool have_offline_normal_quat_k1_{false};
  bool have_offline_normal_quat_k2_{false};
  bool have_offline_normal_metrics_pure_{false};
  bool have_offline_normal_metrics_k1_{false};
  bool have_offline_normal_metrics_k2_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
