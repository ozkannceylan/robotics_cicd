// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Single source of truth for the bridge's topic names (Code Conventions).
#ifndef OTONAV_MUJOCO_BRIDGE__TOPIC_NAMES_HPP_
#define OTONAV_MUJOCO_BRIDGE__TOPIC_NAMES_HPP_

namespace otonav_mujoco_bridge::topics
{
inline constexpr const char * kCmdVel = "cmd_vel";
inline constexpr const char * kClock = "/clock";
inline constexpr const char * kOdom = "odom";
inline constexpr const char * kImu = "imu";
inline constexpr const char * kScan = "scan";
}  // namespace otonav_mujoco_bridge::topics

#endif  // OTONAV_MUJOCO_BRIDGE__TOPIC_NAMES_HPP_
