// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
#include "otonav_mujoco_bridge/can_bus_interface.hpp"

#include <stdexcept>

namespace otonav_mujoco_bridge {

namespace {
[[noreturn]] void not_implemented() {
  throw std::logic_error(
      "CanBusInterface is a stub (ADR-4): wire a real CAN driver here to run on hardware.");
}
}  // namespace

void CanBusInterface::init() { not_implemented(); }
void CanBusInterface::write(const WheelCommand &) { not_implemented(); }
void CanBusInterface::step() { not_implemented(); }
double CanBusInterface::sim_time() const { not_implemented(); }
double CanBusInterface::timestep() const { not_implemented(); }
WheelState CanBusInterface::read_wheels() const { not_implemented(); }
ImuSample CanBusInterface::read_imu() const { not_implemented(); }
RangeScan CanBusInterface::read_scan() const { not_implemented(); }

}  // namespace otonav_mujoco_bridge
