// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// Real-hardware stub. Demonstrates the swap promised by ADR-4: control/nav code
// is identical; only this backend changes. Intentionally unimplemented.
#ifndef OTONAV_MUJOCO_BRIDGE__CAN_BUS_INTERFACE_HPP_
#define OTONAV_MUJOCO_BRIDGE__CAN_BUS_INTERFACE_HPP_

#include "otonav_mujoco_bridge/robot_hardware_interface.hpp"

namespace otonav_mujoco_bridge
{

/// Placeholder CAN-bus backend for a physical diff-drive base. Every method
/// throws std::logic_error — the point is the type signature, not a driver.
class CanBusInterface : public RobotHardwareInterface
{
public:
  CanBusInterface() = default;

  void init() override;
  void write(const WheelCommand & cmd) override;
  void step() override;

  double sim_time() const override;
  double timestep() const override;

  WheelState read_wheels() const override;
  ImuSample read_imu() const override;
  RangeScan read_scan() const override;
};

}  // namespace otonav_mujoco_bridge

#endif  // OTONAV_MUJOCO_BRIDGE__CAN_BUS_INTERFACE_HPP_
