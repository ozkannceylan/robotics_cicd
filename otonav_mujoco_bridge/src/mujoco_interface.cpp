// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
#include "otonav_mujoco_bridge/mujoco_interface.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace otonav_mujoco_bridge
{

namespace
{
int require_actuator(const mjModel * m, const char * name)
{
  const int id = mj_name2id(m, mjOBJ_ACTUATOR, name);
  if (id < 0) {
    throw std::runtime_error(std::string("MJCF missing actuator: ") + name);
  }
  return id;
}

int require_sensor(const mjModel * m, const char * name)
{
  const int id = mj_name2id(m, mjOBJ_SENSOR, name);
  if (id < 0) {
    throw std::runtime_error(std::string("MJCF missing sensor: ") + name);
  }
  return id;
}
}  // namespace

MujocoInterface::MujocoInterface(std::string model_path, int lidar_beams, double range_max)
: model_path_(std::move(model_path)), lidar_beams_(lidar_beams), range_max_(range_max)
{
}

MujocoInterface::~MujocoInterface()
{
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

void MujocoInterface::init()
{
  char error[1024] = "";
  model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
  }
  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    throw std::runtime_error("mj_makeData failed");
  }

  act_left_ = require_actuator(model_, "left_wheel_act");
  act_right_ = require_actuator(model_, "right_wheel_act");
  s_left_pos_ = require_sensor(model_, "left_wheel_pos");
  s_left_vel_ = require_sensor(model_, "left_wheel_vel");
  s_right_pos_ = require_sensor(model_, "right_wheel_pos");
  s_right_vel_ = require_sensor(model_, "right_wheel_vel");
  s_quat_ = require_sensor(model_, "imu_quat");
  s_gyro_ = require_sensor(model_, "imu_gyro");
  s_acc_ = require_sensor(model_, "imu_acc");

  s_lidar_.clear();
  s_lidar_.reserve(lidar_beams_);
  for (int i = 0; i < lidar_beams_; ++i) {
    s_lidar_.push_back(require_sensor(model_, ("lidar_" + std::to_string(i)).c_str()));
  }

  // Fan spans [-pi/2, pi/2] inclusive; must match the MJCF site layout.
  angle_min_ = -M_PI / 2.0;
  angle_max_ = M_PI / 2.0;
  angle_increment_ = (lidar_beams_ > 1) ? (angle_max_ - angle_min_) / (lidar_beams_ - 1) : 0.0;

  mj_forward(model_, data_);  // populate sensordata before the first read
}

const double * MujocoInterface::sensor_ptr(int sensor_id) const
{
  return data_->sensordata + model_->sensor_adr[sensor_id];
}

double MujocoInterface::sensor_value(int sensor_id) const
{
  return sensor_ptr(sensor_id)[0];
}

void MujocoInterface::write(const WheelCommand & cmd)
{
  data_->ctrl[act_left_] = cmd.left_rad_s;
  data_->ctrl[act_right_] = cmd.right_rad_s;
}

void MujocoInterface::step()
{
  mj_step(model_, data_);
}

double MujocoInterface::sim_time() const
{
  return data_->time;
}

double MujocoInterface::timestep() const
{
  return model_->opt.timestep;
}

WheelState MujocoInterface::read_wheels() const
{
  WheelState w;
  w.left_pos = sensor_value(s_left_pos_);
  w.left_vel = sensor_value(s_left_vel_);
  w.right_pos = sensor_value(s_right_pos_);
  w.right_vel = sensor_value(s_right_vel_);
  return w;
}

ImuSample MujocoInterface::read_imu() const
{
  ImuSample imu;
  const double * q = sensor_ptr(s_quat_);
  const double * g = sensor_ptr(s_gyro_);
  const double * a = sensor_ptr(s_acc_);
  imu.orientation = {{q[0], q[1], q[2], q[3]}};
  imu.angular_velocity = {{g[0], g[1], g[2]}};
  imu.linear_acceleration = {{a[0], a[1], a[2]}};
  return imu;
}

RangeScan MujocoInterface::read_scan() const
{
  RangeScan scan;
  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.angle_increment = angle_increment_;
  scan.range_min = 0.0;
  scan.range_max = range_max_;
  scan.ranges.reserve(s_lidar_.size());
  for (const int id : s_lidar_) {
    const double r = sensor_value(id);
    // MuJoCo rangefinder returns -1 when no geom is hit within range.
    scan.ranges.push_back(
      (r < 0.0 || r > range_max_) ? static_cast<float>(range_max_) : static_cast<float>(r));
  }
  return scan;
}

}  // namespace otonav_mujoco_bridge
