#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crazyflie_interfaces/msg/hover.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <crazyflie_interfaces/msg/velocity_world.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ncurses.h>

#include <array>
#include <cmath>
#include <chrono>
#include <deque>
#include <limits>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class CommandVelocityPublisher : public rclcpp::Node
{
public:
  explicit CommandVelocityPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("command_velocity_publisher", options)
  {
    cf_ns_ = this->declare_parameter<std::string>("cf_namespace", "cf2");
    interface_type_ = this->declare_parameter<std::string>("interface_type", "hover");

    cf_hover_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::Hover>(
        cf_ns_ + "/cmd_hover",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    cf_velocity_world_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::VelocityWorld>(
        cf_ns_ + "/cmd_velocity_world",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    key_pub_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>("su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>("su/cmd_force", 10);
    status_sub_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      cf_ns_ + "/status", 10,
      std::bind(&CommandVelocityPublisher::statusCallback, this, std::placeholders::_1));

    vel_delta_[0] = this->declare_parameter<double>("dvx", 0.1);
    vel_delta_[1] = this->declare_parameter<double>("dvy", 0.1);
    vel_delta_[2] = this->declare_parameter<double>("dvz", 0.2);
    hover_z_delta_ = this->declare_parameter<double>("dz", 0.05);
    yaw_rate_delta_ = this->declare_parameter<double>("dyaw_rate", 0.2);
    hover_z_distance_ = this->declare_parameter<double>("hover_z_distance", 0.6);
    force_delta_ = this->declare_parameter<double>("df", 0.01);

    vel_xyz_.fill(0.0);
    yaw_rate_ = 0.0;
    force_des_ = 0.0;
    use_vel_mode_ = 0.0;
    status_msg_ = "ready";
    latest_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    displayed_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    last_battery_display_update_ = std::chrono::steady_clock::now();

    last_inputs_.clear();
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      last_inputs_.push_back("-");
    }

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
    warnIfUnsupportedByBackend();

    RCLCPP_INFO(
      this->get_logger(),
      "command_velocity_publisher started. ns=%s interface=%s",
      cf_ns_.c_str(), interface_type_.c_str());

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandVelocityPublisher::timerCallback, this));
  }

  ~CommandVelocityPublisher() override
  {
    endwin();
  }

private:
  static constexpr int ROW_USAGE_HEADER   = 0;
  static constexpr int ROW_USAGE_1        = 2;
  static constexpr int ROW_USAGE_2        = 3;
  static constexpr int ROW_USAGE_3        = 4;

  static constexpr int ROW_STATUS_HEADER  = 6;
  static constexpr int ROW_STATUS_MODE    = 8;
  static constexpr int ROW_STATUS_FORCE   = 9;
  static constexpr int ROW_STATUS_BATTERY = 10;
  static constexpr int ROW_STATUS_MSG     = 11;

  static constexpr int ROW_CMD_HEADER     = 13;
  static constexpr int ROW_CMD_LINE1      = 15;
  static constexpr int ROW_CMD_LINE2      = 16;
  static constexpr int ROW_CMD_LINE3      = 17;
  static constexpr int ROW_CMD_HIST_HDR   = 19;
  static constexpr int ROW_CMD_HIST_0     = 20;
  static constexpr int ROW_CMD_HIST_1     = 21;
  static constexpr int ROW_CMD_HIST_2     = 22;
  static constexpr int ROW_CMD_HIST_3     = 23;
  static constexpr int ROW_CMD_HIST_4     = 24;

  static constexpr size_t HISTORY_LEN     = 5;

  bool isHoverInterface() const
  {
    return interface_type_ == "hover";
  }

  bool isVelocityWorldInterface() const
  {
    return interface_type_ == "velocity_world";
  }

  void timerCallback()
  {
    int ch;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    updateDisplayedBatteryVoltage();
    publishVelocityCmd();
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
    if (c == 'w') {
      vel_xyz_[0] += vel_delta_[0];
      pushInputHistory("w : vx += dvx");
    } else if (c == 's') {
      vel_xyz_[0] -= vel_delta_[0];
      pushInputHistory("s : vx -= dvx");
    } else if (c == 'a') {
      vel_xyz_[1] += vel_delta_[1];
      pushInputHistory("a : vy += dvy");
    } else if (c == 'd') {
      vel_xyz_[1] -= vel_delta_[1];
      pushInputHistory("d : vy -= dvy");
    } else if (c == 'e') {
      if (isHoverInterface()) {
        hover_z_distance_ += hover_z_delta_;
        pushInputHistory("e : hover z += dz");
      } else {
        vel_xyz_[2] += vel_delta_[2];
        pushInputHistory("e : vz += dvz");
      }
    } else if (c == 'q') {
      if (isHoverInterface()) {
        hover_z_distance_ -= hover_z_delta_;
        pushInputHistory("q : hover z -= dz");
      } else {
        vel_xyz_[2] -= vel_delta_[2];
        pushInputHistory("q : vz -= dvz");
      }
    } else if (c == 'z') {
      yaw_rate_ += yaw_rate_delta_;
      pushInputHistory("z : yaw_rate += dpsi");
    } else if (c == 'c') {
      yaw_rate_ -= yaw_rate_delta_;
      pushInputHistory("c : yaw_rate -= dpsi");
    } else if (c == 'x') {
      resetVelocityCmd();
      pushInputHistory("x : reset velocity cmd");
    } else if (c == 'j') {
      force_des_ += force_delta_;
      publishForce();
      pushInputHistory("j : force += df");
    } else if (c == 'k') {
      force_des_ -= force_delta_;
      publishForce();
      pushInputHistory("k : force -= df");
    } else if (c == 'l') {
      force_des_ = 0.0;
      publishForce();
      pushInputHistory("l : force reset");
    } else if (c == 'm') {
      publishKeyboardCommand(c);
    } else if (c == 'i') {
      setVelMode(1.0);
      pushInputHistory("i : set VELOCITY=1");
    } else if (c == 'u') {
      setVelMode(0.0);
      pushInputHistory("u : set POSITION=0");
    } else if (c == 'o' || c == 'p') {
      publishKeyboardCommand(c);
    } else if (c == 't') {
      pushInputHistory("t : quit");
      status_msg_ = "exit key pressed";
      rclcpp::shutdown();
    }
  }

  void publishKeyboardCommand(char c)
  {
    if (!key_pub_) {
      status_msg_ = "key_pub not ready";
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(1, c);
    key_pub_->publish(msg);

    if (c == 'o') {
      status_msg_ = "published 'o' to keyboard_input (ARM)";
      pushInputHistory("o : ARM (keyboard_input)");
    } else if (c == 'p') {
      status_msg_ = "published 'p' to keyboard_input (DISARM)";
      pushInputHistory("p : DISARM (keyboard_input)");
    } else if (c == 'm') {
      status_msg_ = "published 'm' to keyboard_input (ZERO BIAS)";
      pushInputHistory("m : ZERO BIAS (keyboard_input)");
    }
  }

  void publishVelocityCmd()
  {
    if (isHoverInterface()) {
      crazyflie_interfaces::msg::Hover msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "body";
      msg.vx = static_cast<float>(vel_xyz_[0]);
      msg.vy = static_cast<float>(vel_xyz_[1]);
      msg.yaw_rate = static_cast<float>(yaw_rate_);
      msg.z_distance = static_cast<float>(hover_z_distance_);
      cf_hover_pub_->publish(msg);
      return;
    }

    if (isVelocityWorldInterface()) {
      crazyflie_interfaces::msg::VelocityWorld msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "world";
      msg.vel.x = static_cast<float>(vel_xyz_[0]);
      msg.vel.y = static_cast<float>(vel_xyz_[1]);
      msg.vel.z = static_cast<float>(vel_xyz_[2]);
      msg.yaw_rate = static_cast<float>(yaw_rate_);
      cf_velocity_world_pub_->publish(msg);
      return;
    }

    status_msg_ = "unknown interface_type: " + interface_type_;
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
    use_vel_mode_ = (mode > 0.5) ? 1.0 : 0.0;

    if (!use_vel_mode_pub_) {
      status_msg_ = "use_vel_mode_pub not ready";
      return;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(use_vel_mode_);
    use_vel_mode_pub_->publish(msg);

    resetVelocityCmd();

    char buf[128];
    snprintf(
      buf, sizeof(buf),
      "use_vel_mode = %.0f (%s mode), reset immediately",
      use_vel_mode_, (use_vel_mode_ > 0.5 ? "VELOCITY" : "POSITION"));
    status_msg_ = buf;
  }

  void resetVelocityCmd()
  {
    vel_xyz_.fill(0.0);
    yaw_rate_ = 0.0;
    status_msg_ = "Velocity command reset to zero";
  }

  void warnIfUnsupportedByBackend()
  {
    if (isHoverInterface()) {
      status_msg_ = "cmd_hover publisher enabled";
    } else if (isVelocityWorldInterface()) {
      status_msg_ = "cmd_velocity_world publisher enabled";
    }
  }


  void pushInputHistory(const std::string & s)
  {
    last_inputs_.push_front(s);
    while (last_inputs_.size() > HISTORY_LEN) {
      last_inputs_.pop_back();
    }
    while (last_inputs_.size() < HISTORY_LEN) {
      last_inputs_.push_back("-");
    }
  }

  void drawSepLine(int row, const char * title)
  {
    move(row, 0);
    clrtoeol();
    printw("========================%s========================", title);
  }

  void drawLayout()
  {
    clear();

    drawSepLine(ROW_USAGE_HEADER, "usage");
    mvprintw(ROW_USAGE_1, 0, "velocity command: w/s(vx), a/d(vy), e/q(vz or hover z), x(reset)");
    mvprintw(ROW_USAGE_2, 0, "yaw rate / mode:  z/c(yaw_rate), i(set vel=1), u(set pos=0)");
    mvprintw(ROW_USAGE_3, 0, "force / arm:      j/k/l (cmd_fx), m(zero bias), o/p arm/disarm, t quit");

    drawSepLine(ROW_STATUS_HEADER, "status");
    mvprintw(ROW_STATUS_MODE, 0, "mode: ");
    mvprintw(ROW_STATUS_FORCE, 0, "force command: ");
    mvprintw(ROW_STATUS_BATTERY, 0, "battery voltage: ");
    mvprintw(ROW_STATUS_MSG, 0, "status: ");

    drawSepLine(ROW_CMD_HEADER, "Velocity Command");
    mvprintw(ROW_CMD_LINE1, 0, "interface = -, topic = -");
    mvprintw(ROW_CMD_LINE2, 0, "vx = 0.000, vy = 0.000, vz = 0.000, yaw_rate = 0.000");
    mvprintw(ROW_CMD_LINE3, 0, "hover_z = 0.000");

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      mvprintw(ROW_CMD_HIST_0 + static_cast<int>(i), 0, "  %zu) -", i + 1);
    }

    refresh();
  }

  void drawStatusBlock()
  {
    move(ROW_STATUS_MODE, 0);
    clrtoeol();
    printw(
      "mode: use_vel_mode = %.0f (%s), interface_type: %s, namespace: %s",
      use_vel_mode_, (use_vel_mode_ > 0.5 ? "VELOCITY" : "POSITION"),
      interface_type_.c_str(), cf_ns_.c_str());

    move(ROW_STATUS_FORCE, 0);
    clrtoeol();
    printw("force command: %.3f", force_des_);

    move(ROW_STATUS_BATTERY, 0);
    clrtoeol();
    if (std::isfinite(displayed_battery_voltage_)) {
      printw("battery voltage: %.2f V (1 Hz)", displayed_battery_voltage_);
    } else {
      printw("battery voltage: waiting for %s/status", cf_ns_.c_str());
    }

    move(ROW_STATUS_MSG, 0);
    clrtoeol();
    if (color_enabled_) {
      attron(COLOR_PAIR(1));
    }
    printw("status: %s", status_msg_.c_str());
    if (color_enabled_) {
      attroff(COLOR_PAIR(1));
    }

    refresh();
  }

  void drawCommandBlock()
  {
    move(ROW_CMD_LINE1, 0);
    clrtoeol();
    printw(
      "interface = %s, topic = %s/%s",
      interface_type_.c_str(),
      cf_ns_.c_str(),
      (isHoverInterface() ? "cmd_hover" : "cmd_velocity_world"));

    move(ROW_CMD_LINE2, 0);
    clrtoeol();
    printw(
      "vx = %.3f, vy = %.3f, vz = %.3f, yaw_rate = %.3f",
      vel_xyz_[0], vel_xyz_[1], vel_xyz_[2], yaw_rate_);

    move(ROW_CMD_LINE3, 0);
    clrtoeol();
    if (isHoverInterface()) {
      printw("hover_z_distance = %.3f", hover_z_distance_);
    } else {
      printw("hover_z_distance (unused) = %.3f", hover_z_distance_);
    }

    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      move(ROW_CMD_HIST_0 + static_cast<int>(i), 0);
      clrtoeol();
      printw("  %zu) %s", i + 1, last_inputs_[i].c_str());
    }

    refresh();
  }

  rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr cf_hover_pub_;
  rclcpp::Publisher<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr cf_velocity_world_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_pub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string cf_ns_;
  std::string interface_type_;
  std::array<double, 3> vel_xyz_;
  std::array<double, 3> vel_delta_;
  double hover_z_delta_;
  double yaw_rate_delta_;
  double hover_z_distance_;
  double yaw_rate_;
  double force_delta_;
  double force_des_;
  double use_vel_mode_;
  double latest_battery_voltage_;
  double displayed_battery_voltage_;
  std::string status_msg_;
  std::deque<std::string> last_inputs_;
  std::chrono::steady_clock::time_point last_battery_display_update_;
  bool color_enabled_;
};

int main(int argc, char * argv[])
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

  auto node = std::make_shared<CommandVelocityPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
