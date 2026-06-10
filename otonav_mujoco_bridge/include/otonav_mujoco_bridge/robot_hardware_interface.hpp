// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// The sim-to-real seam (ADR-4). Control/nav code talks only to ROS messages and
// this interface — never to MuJoCo types (Hard Rule #4). Swapping MujocoInterface
// for a real-hardware backend is the entire sim-to-real story.
#ifndef OTONAV_MUJOCO_BRIDGE__ROBOT_HARDWARE_INTERFACE_HPP_
#define OTONAV_MUJOCO_BRIDGE__ROBOT_HARDWARE_INTERFACE_HPP_

#include <array>
#include <vector>

namespace otonav_mujoco_bridge {

/// Commanded wheel angular velocities [rad/s].
struct WheelCommand {
  double left_rad_s{0.0};
  double right_rad_s{0.0};
};

/// Measured wheel state: angle [rad] and angular velocity [rad/s].
struct WheelState {
  double left_pos{0.0};
  double left_vel{0.0};
  double right_pos{0.0};
  double right_vel{0.0};
};

/// IMU feedback. Orientation quaternion is (w, x, y, z).
struct ImuSample {
  std::array<double, 4> orientation{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};
};

/// Planar range scan, ROS LaserScan conventions ([range_min, range_max], CCW).
struct RangeScan {
  std::vector<float> ranges;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

/// Abstract hardware backend. write() actuates; read_*() return feedback;
/// step() advances the hardware/sim by one timestep() and sim_time() is the
/// authoritative clock the bridge publishes on /clock (ADR-2).
class RobotHardwareInterface {
 public:
  virtual ~RobotHardwareInterface() = default;

  virtual void init() = 0;
  virtual void write(const WheelCommand & cmd) = 0;
  virtual void step() = 0;

  virtual double sim_time() const = 0;  // [s]
  virtual double timestep() const = 0;  // [s] advanced per step()

  virtual WheelState read_wheels() const = 0;
  virtual ImuSample read_imu() const = 0;
  virtual RangeScan read_scan() const = 0;
};

}  // namespace otonav_mujoco_bridge

#endif  // OTONAV_MUJOCO_BRIDGE__ROBOT_HARDWARE_INTERFACE_HPP_
