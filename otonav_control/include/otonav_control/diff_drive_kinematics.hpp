// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Diff-drive kinematics — pure math, no ROS, no MuJoCo (Hard Rule #4).
// The single home for the kinematics referenced in architecture.md §6.
#ifndef OTONAV_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_
#define OTONAV_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_

namespace otonav_control {

/// Wheel angular velocities [rad/s].
struct WheelSpeeds {
  double left;
  double right;
};

/// Planar body twist: forward velocity v [m/s], yaw rate w [rad/s].
struct BodyTwist {
  double v;
  double w;
};

/// Body twist -> wheel angular velocities.
///   w_L = (2v - wL) / (2r),  w_R = (2v + wL) / (2r)
inline WheelSpeeds inverse_kinematics(const BodyTwist & twist, double wheel_radius,
                                      double track_width) {
  const double r = wheel_radius;
  const double l = track_width;
  return WheelSpeeds{(2.0 * twist.v - twist.w * l) / (2.0 * r),
                     (2.0 * twist.v + twist.w * l) / (2.0 * r)};
}

/// Wheel angular velocities -> body twist (used for odometry).
///   v = r (w_R + w_L) / 2,  w = r (w_R - w_L) / L
inline BodyTwist forward_kinematics(const WheelSpeeds & wheels, double wheel_radius,
                                    double track_width) {
  const double r = wheel_radius;
  const double l = track_width;
  return BodyTwist{r * (wheels.right + wheels.left) / 2.0, r * (wheels.right - wheels.left) / l};
}

}  // namespace otonav_control

#endif  // OTONAV_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_
