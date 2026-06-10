// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Go-to-goal P-controller — pure math, no ROS/MuJoCo. Unit-testable in isolation.
#ifndef OTONAV_NAV__GO_TO_GOAL_HPP_
#define OTONAV_NAV__GO_TO_GOAL_HPP_

#include <algorithm>
#include <cmath>

namespace otonav_nav {

struct GoToGoalParams {
  double kp_lin{0.8};
  double kp_ang{2.0};
  double v_max{0.6};
  double w_max{1.5};
  double goal_tolerance{0.1};
};

struct VelocityCommand {
  double v{0.0};
  double w{0.0};
  bool reached{false};
};

/// Wrap an angle to [-pi, pi].
inline double wrap_angle(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

/// P-control toward a goal. dx,dy = goal minus robot in the world frame; yaw =
/// robot heading. Turns toward the goal and only drives forward while facing it.
inline VelocityCommand go_to_goal(double dx, double dy, double yaw, const GoToGoalParams & p) {
  const double dist = std::hypot(dx, dy);
  if (dist < p.goal_tolerance) {
    return VelocityCommand{0.0, 0.0, true};
  }
  const double heading_err = wrap_angle(std::atan2(dy, dx) - yaw);
  const double w = std::clamp(p.kp_ang * heading_err, -p.w_max, p.w_max);
  double v = p.kp_lin * dist * std::max(0.0, std::cos(heading_err));
  v = std::clamp(v, 0.0, p.v_max);
  return VelocityCommand{v, w, false};
}

}  // namespace otonav_nav

#endif  // OTONAV_NAV__GO_TO_GOAL_HPP_
