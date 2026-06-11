# Changelog

All notable changes to OtoNav-CI are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this project adheres
to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.1] - 2026-06-11

### Fixed
- Release image build: the multi-stage `Dockerfile` COPY list predated
  `otonav_nav` (Phase 3), so the `v0.1.0` tag build failed at rosdep. Added the
  missing package COPY. `v0.1.0` shipped no image; `v0.1.1` is the first
  published runner image.

## [0.1.0] - 2026-06-11

First end-to-end release: a deliberately simple differential-drive robot inside a
production-grade ROS 2 Humble + MuJoCo SIL CI/CD pipeline.

### Added
- **Repo & CI hygiene** — protected `main` (ruleset: PR required, `lint-build-test`
  status check, no direct/force push), PR template, CODEOWNERS, bug/task issue
  templates, pinned `clang-format` + `pre-commit`.
- **Sim bridge** (`otonav_mujoco_bridge`) — C++ MuJoCo bridge owning the `mj_step`
  loop and `/clock` (sim-time authority); `/cmd_vel` → diff-drive inverse
  kinematics, publishes `/odom`, `/imu`, `/scan`. `RobotHardwareInterface` HAL
  seam with `MujocoInterface` + `CanBusInterface` stub; MuJoCo types confined to
  one translation unit.
- **Robot model** (`otonav_description`) — MJCF diff-drive robot (velocity-actuated
  wheels, front/rear casters, IMU site, 13-beam lidar fan) and the goal-reach scene.
- **Kinematics** (`otonav_control`) — header-only diff-drive forward/inverse
  kinematics with unit tests.
- **Navigation** (`otonav_nav`) — go-to-goal P-controller + lidar stop-on-obstacle
  guard, with unit tests for both laws.
- **Bring-up** (`otonav_bringup`) — `sim.launch.py` (headless by default) and two
  `launch_testing` scenarios: an `/odom` smoke test and a sim-time goal-reach
  integration test (goal < 20 s, obstacle clearance ≥ 0.3 m) with a failure rosbag.
- **CI** — Humble + MuJoCo container job: cached MuJoCo SDK + ccache, `colcon build`
  (Release) and `colcon test` (lint + gtest + SIL), JUnit artifacts, rosbag on
  failure; advisory `pre-commit` job. FastDDS CI fallback profile documented.
- **Containers** — multi-stage `docker/Dockerfile` (builder → slim runner) with a
  correct sourced entrypoint; MuJoCo version pinned in one place.

[Unreleased]: https://github.com/ozkannceylan/robotics_cicd/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/ozkannceylan/robotics_cicd/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/ozkannceylan/robotics_cicd/releases/tag/v0.1.0
