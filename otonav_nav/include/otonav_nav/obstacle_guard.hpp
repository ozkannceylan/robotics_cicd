// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Lidar stop-on-obstacle guard — pure math, no ROS/MuJoCo.
#ifndef OTONAV_NAV__OBSTACLE_GUARD_HPP_
#define OTONAV_NAV__OBSTACLE_GUARD_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace otonav_nav {

/// True if any beam within +/- front_half_angle of straight ahead reports a
/// finite range below stop_distance. Beam i points at angle_min + i*increment.
inline bool obstacle_ahead(const std::vector<float> & ranges, double angle_min,
                           double angle_increment, double front_half_angle, double stop_distance) {
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double angle = angle_min + static_cast<double>(i) * angle_increment;
    if (std::abs(angle) > front_half_angle) {
      continue;
    }
    const float r = ranges[i];
    if (std::isfinite(r) && r > 0.0F && r < static_cast<float>(stop_distance)) {
      return true;
    }
  }
  return false;
}

}  // namespace otonav_nav

#endif  // OTONAV_NAV__OBSTACLE_GUARD_HPP_
