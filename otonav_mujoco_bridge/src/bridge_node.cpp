// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// OtoNav MuJoCo bridge node.
//
// Owns the sim loop and the authoritative clock (ADR-2): each tick it actuates
// from the latest /cmd_vel via diff-drive inverse kinematics, advances the
// hardware backend by one publish period, then publishes /clock, /odom, /imu
// and /scan stamped with sim time. The driving timer is a wall timer so the
// node never waits on the very clock it produces.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "otonav_control/diff_drive_kinematics.hpp"
#include "otonav_mujoco_bridge/can_bus_interface.hpp"
#include "otonav_mujoco_bridge/mujoco_interface.hpp"
#include "otonav_mujoco_bridge/robot_hardware_interface.hpp"
#include "otonav_mujoco_bridge/topic_names.hpp"

namespace otonav_mujoco_bridge {

namespace {
rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  return d;
}

std::string default_model_path() {
  return ament_index_cpp::get_package_share_directory("otonav_description") + "/model/otonav.xml";
}
}  // namespace

class OtoNavBridge : public rclcpp::Node {
 public:
  OtoNavBridge() : rclcpp::Node("otonav_bridge") {
    backend_ = declare_parameter<std::string>("backend", "mujoco",
                                              describe("Hardware backend: 'mujoco' or 'canbus'."));
    const std::string model_path = declare_parameter<std::string>(
        "model_path", default_model_path(), describe("Absolute path to the MJCF model."));
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05,
                                              describe("Wheel radius [m]; must match the MJCF."));
    track_width_ = declare_parameter<double>(
        "track_width", 0.30, describe("Wheel separation [m]; must match the MJCF."));
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0,
                                              describe("Sensor/clock publish rate [Hz]."));
    const int lidar_beams = declare_parameter<int>(
        "lidar_beams", 13, describe("Number of lidar rangefinder beams in the MJCF."));
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom", describe("Odometry frame."));
    base_frame_ =
        declare_parameter<std::string>("base_frame", "base_link", describe("Base frame."));
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link", describe("IMU frame."));
    laser_frame_ =
        declare_parameter<std::string>("laser_frame", "laser_link", describe("Lidar frame."));
    const bool enable_viewer = declare_parameter<bool>(
        "enable_viewer", false,
        describe("Open the MuJoCo viewer (local dev only; headless build is a no-op)."));
    if (enable_viewer) {
      RCLCPP_WARN(get_logger(),
                  "enable_viewer=true but this build is headless (ADR-1): physics runs, no window. "
                  "Rendering is opt-in and never a precondition (Hard Rule #7).");
    }

    hw_ = make_hardware(backend_, model_path, lidar_beams);
    hw_->init();

    const double dt = hw_->timestep();
    steps_per_publish_ = std::max(1, static_cast<int>(std::lround((1.0 / publish_rate_) / dt)));
    RCLCPP_INFO(get_logger(), "backend=%s model_dt=%.4fs publish_rate=%.1fHz steps/publish=%d",
                backend_.c_str(), dt, publish_rate_, steps_per_publish_);

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(topics::kClock, rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(topics::kOdom, rclcpp::QoS(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topics::kImu, rclcpp::SensorDataQoS());
    scan_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>(topics::kScan, rclcpp::SensorDataQoS());

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        topics::kCmdVel, rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_v_ = msg->linear.x;
          cmd_w_ = msg->angular.z;
        });

    last_sim_time_ = hw_->sim_time();
    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), [this]() { tick(); });
  }

 private:
  static std::unique_ptr<RobotHardwareInterface> make_hardware(const std::string & backend,
                                                               const std::string & model_path,
                                                               int lidar_beams) {
    if (backend == "mujoco") {
      return std::make_unique<MujocoInterface>(model_path, lidar_beams);
    }
    if (backend == "canbus") {
      return std::make_unique<CanBusInterface>();
    }
    throw std::runtime_error("Unknown backend '" + backend + "' (use 'mujoco' or 'canbus').");
  }

  rclcpp::Time sim_stamp() const {
    return rclcpp::Time(static_cast<int64_t>(std::llround(hw_->sim_time() * 1e9)), RCL_ROS_TIME);
  }

  void tick() {
    // Actuate from the latest command, then advance the sim one publish period.
    const otonav_control::WheelSpeeds w =
        otonav_control::inverse_kinematics({cmd_v_, cmd_w_}, wheel_radius_, track_width_);
    hw_->write(WheelCommand{w.left, w.right});
    for (int i = 0; i < steps_per_publish_; ++i) {
      hw_->step();
    }

    const rclcpp::Time stamp = sim_stamp();
    publish_clock(stamp);
    publish_odom(stamp);
    publish_imu(stamp);
    publish_scan(stamp);
  }

  void publish_clock(const rclcpp::Time & stamp) {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = stamp;
    clock_pub_->publish(msg);
  }

  void publish_odom(const rclcpp::Time & stamp) {
    const WheelState ws = hw_->read_wheels();
    const otonav_control::BodyTwist t = otonav_control::forward_kinematics(
        {ws.left_vel, ws.right_vel}, wheel_radius_, track_width_);

    const double now = hw_->sim_time();
    const double dt = std::max(0.0, now - last_sim_time_);
    last_sim_time_ = now;

    // Integrate the pose on sim time (midpoint heading).
    theta_ += t.w * dt;
    x_ += t.v * std::cos(theta_) * dt;
    y_ += t.v * std::sin(theta_) * dt;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    msg.twist.twist.linear.x = t.v;
    msg.twist.twist.angular.z = t.w;
    odom_pub_->publish(msg);
  }

  void publish_imu(const rclcpp::Time & stamp) {
    const ImuSample s = hw_->read_imu();
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_;
    msg.orientation.w = s.orientation[0];
    msg.orientation.x = s.orientation[1];
    msg.orientation.y = s.orientation[2];
    msg.orientation.z = s.orientation[3];
    msg.angular_velocity.x = s.angular_velocity[0];
    msg.angular_velocity.y = s.angular_velocity[1];
    msg.angular_velocity.z = s.angular_velocity[2];
    msg.linear_acceleration.x = s.linear_acceleration[0];
    msg.linear_acceleration.y = s.linear_acceleration[1];
    msg.linear_acceleration.z = s.linear_acceleration[2];
    imu_pub_->publish(msg);
  }

  void publish_scan(const rclcpp::Time & stamp) {
    const RangeScan s = hw_->read_scan();
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = laser_frame_;
    msg.angle_min = static_cast<float>(s.angle_min);
    msg.angle_max = static_cast<float>(s.angle_max);
    msg.angle_increment = static_cast<float>(s.angle_increment);
    msg.time_increment = 0.0F;
    msg.scan_time = static_cast<float>(1.0 / publish_rate_);
    msg.range_min = static_cast<float>(s.range_min);
    msg.range_max = static_cast<float>(s.range_max);
    msg.ranges = s.ranges;
    scan_pub_->publish(msg);
  }

  std::string backend_;
  double wheel_radius_{0.05};
  double track_width_{0.30};
  double publish_rate_{100.0};
  int steps_per_publish_{1};
  std::string odom_frame_, base_frame_, imu_frame_, laser_frame_;

  std::unique_ptr<RobotHardwareInterface> hw_;

  double cmd_v_{0.0};
  double cmd_w_{0.0};
  double x_{0.0}, y_{0.0}, theta_{0.0};
  double last_sim_time_{0.0};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace otonav_mujoco_bridge

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<otonav_mujoco_bridge::OtoNavBridge>());
  rclcpp::shutdown();
  return 0;
}
