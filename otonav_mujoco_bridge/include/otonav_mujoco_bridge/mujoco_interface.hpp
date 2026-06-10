// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
// MuJoCo implementation of RobotHardwareInterface. The ONLY translation unit
// allowed to touch MuJoCo types (Hard Rule #4). MuJoCo types are forward-declared
// here and included only in the .cpp to keep the header light.
#ifndef OTONAV_MUJOCO_BRIDGE__MUJOCO_INTERFACE_HPP_
#define OTONAV_MUJOCO_BRIDGE__MUJOCO_INTERFACE_HPP_

#include <string>
#include <vector>

#include "otonav_mujoco_bridge/robot_hardware_interface.hpp"

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;  // NOLINT(modernize-use-using) — C SDK typedef
typedef struct mjData_ mjData;    // NOLINT(modernize-use-using)

namespace otonav_mujoco_bridge
{

class MujocoInterface : public RobotHardwareInterface
{
public:
  explicit MujocoInterface(std::string model_path, int lidar_beams = 13, double range_max = 10.0);
  ~MujocoInterface() override;

  MujocoInterface(const MujocoInterface &) = delete;
  MujocoInterface & operator=(const MujocoInterface &) = delete;

  void init() override;
  void write(const WheelCommand & cmd) override;
  void step() override;

  double sim_time() const override;
  double timestep() const override;

  WheelState read_wheels() const override;
  ImuSample read_imu() const override;
  RangeScan read_scan() const override;

private:
  double sensor_value(int sensor_id) const;  // first scalar of a sensor
  const double * sensor_ptr(int sensor_id) const;

  std::string model_path_;
  int lidar_beams_;
  double range_max_;

  mjModel * model_{nullptr};
  mjData * data_{nullptr};

  // Cached actuator / sensor ids resolved by name in init().
  int act_left_{-1};
  int act_right_{-1};
  int s_left_pos_{-1};
  int s_left_vel_{-1};
  int s_right_pos_{-1};
  int s_right_vel_{-1};
  int s_quat_{-1};
  int s_gyro_{-1};
  int s_acc_{-1};
  std::vector<int> s_lidar_;

  double angle_min_{0.0};
  double angle_max_{0.0};
  double angle_increment_{0.0};
};

}  // namespace otonav_mujoco_bridge

#endif  // OTONAV_MUJOCO_BRIDGE__MUJOCO_INTERFACE_HPP_
