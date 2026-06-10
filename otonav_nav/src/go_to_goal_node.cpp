// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Go-to-goal navigation node: drives the robot to a goal with a P-controller and
// halts when the lidar sees an obstacle straight ahead. Consumes /odom and /scan,
// publishes /cmd_vel. Pure control law lives in go_to_goal.hpp / obstacle_guard.hpp.
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "otonav_nav/go_to_goal.hpp"
#include "otonav_nav/obstacle_guard.hpp"

namespace otonav_nav {

namespace {
rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  return d;
}

double yaw_from_quaternion(double z, double w, double x, double y) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class GoToGoalNode : public rclcpp::Node {
 public:
  GoToGoalNode() : rclcpp::Node("otonav_nav") {
    params_.kp_lin = declare_parameter<double>("kp_lin", 0.8, describe("Linear P gain."));
    params_.kp_ang = declare_parameter<double>("kp_ang", 2.0, describe("Angular P gain."));
    params_.v_max = declare_parameter<double>("v_max", 0.6, describe("Max forward speed [m/s]."));
    params_.w_max = declare_parameter<double>("w_max", 1.5, describe("Max yaw rate [rad/s]."));
    params_.goal_tolerance =
        declare_parameter<double>("goal_tolerance", 0.1, describe("Goal reached radius [m]."));

    obstacle_stop_distance_ = declare_parameter<double>(
        "obstacle_stop_distance", 0.4, describe("Halt if a front beam is closer [m]."));
    front_half_angle_ = declare_parameter<double>(
        "front_half_angle", 0.35, describe("Half-width of the front lidar sector [rad]."));
    const double control_rate =
        declare_parameter<double>("control_rate", 20.0, describe("Control loop rate [Hz]."));

    const bool use_initial_goal = declare_parameter<bool>(
        "use_initial_goal", false, describe("Seed the goal from goal_x/goal_y at startup."));
    const double goal_x = declare_parameter<double>("goal_x", 0.0, describe("Initial goal x [m]."));
    const double goal_y = declare_parameter<double>("goal_y", 0.0, describe("Initial goal y [m]."));
    if (use_initial_goal) {
      goal_x_ = goal_x;
      goal_y_ = goal_y;
      has_goal_ = true;
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          x_ = msg->pose.pose.position.x;
          y_ = msg->pose.pose.position.y;
          const auto & q = msg->pose.pose.orientation;
          yaw_ = yaw_from_quaternion(q.z, q.w, q.x, q.y);
          have_odom_ = true;
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_ = *msg;
          have_scan_ = true;
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::QoS(10), [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          goal_x_ = msg->pose.position.x;
          goal_y_ = msg->pose.position.y;
          has_goal_ = true;
          RCLCPP_INFO(get_logger(), "new goal: (%.2f, %.2f)", goal_x_, goal_y_);
        });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_rate),
                               [this]() { control_step(); });
  }

 private:
  void control_step() {
    if (!has_goal_ || !have_odom_) {
      return;
    }
    VelocityCommand cmd = go_to_goal(goal_x_ - x_, goal_y_ - y_, yaw_, params_);

    bool blocked = false;
    if (have_scan_) {
      blocked = obstacle_ahead(scan_.ranges, scan_.angle_min, scan_.angle_increment,
                               front_half_angle_, obstacle_stop_distance_);
    }
    if (blocked) {
      cmd.v = 0.0;
      cmd.w = 0.0;
      if (!was_blocked_) {
        RCLCPP_WARN(get_logger(), "obstacle ahead < %.2f m — halting", obstacle_stop_distance_);
      }
    }
    was_blocked_ = blocked;

    if (cmd.reached && !reached_logged_) {
      RCLCPP_INFO(get_logger(), "goal reached");
      reached_logged_ = true;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.v;
    twist.angular.z = cmd.w;
    cmd_pub_->publish(twist);
  }

  GoToGoalParams params_;
  double obstacle_stop_distance_{0.4};
  double front_half_angle_{0.35};

  double goal_x_{0.0};
  double goal_y_{0.0};
  bool has_goal_{false};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  bool have_odom_{false};

  sensor_msgs::msg::LaserScan scan_;
  bool have_scan_{false};
  bool was_blocked_{false};
  bool reached_logged_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace otonav_nav

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<otonav_nav::GoToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
