# TODO — OtoNav-CI

Work top to bottom. Each checkbox = one PR where feasible. Phases mirror `project_plan.md`. If time pressure hits, consult the Cut Lines there — never skip Phase 0 or Phase 2.

## Phase 0 — Repo & Hygiene Scaffold
- [x] Create GitHub repo, push initial commit with the four md docs + LICENSE + .gitignore (ROS/C++/Python)
- [x] Add `.github/PULL_REQUEST_TEMPLATE.md` (description, linked issue, test evidence, safety/regression checklist)
- [x] Add `CODEOWNERS` (own the critical paths: `/otonav_mujoco_bridge/`, `/.github/`)
- [x] Add issue templates (bug, task)
- [x] Enable branch protection on `main`: require PR, require status checks, no force-push
- [~] Enable native Merge Queue on `main` — repo is now **public**, but the rulesets REST API rejects the `merge_queue` rule (HTTP 422). Enable via the web UI: Settings → Rules → `main-protection` → "Require merge queue". Until then the branch-protection ruleset (PR required + `lint-build-test` check + no direct/force push) is the gate.
- [x] Add `pre-commit` config: clang-format, trailing-whitespace, yaml check; document install in README
- [x] Add stub CI workflow (checkout + echo) so the required check exists from day one
- [x] Verify: open a trivial PR → template appears → stub check runs → merges via queue

## Phase 1 — Minimal Sim Bridge
- [x] `otonav_description`: MJCF diff-drive model (chassis, 2 actuated wheels w/ velocity actuators, caster, lidar fan, IMU site); validated it compiles and drives via `mj_step`
- [x] `otonav_mujoco_bridge` package skeleton (C++, ament_cmake), MuJoCo found via CMake (`MUJOCO_DIR` from Docker ARG/env)
- [x] `RobotHardwareInterface` abstract class + `MujocoInterface` implementation + `CanBusInterface` stub
- [x] Bridge node: mj_step loop, publishes `/clock` (sim time authority)
- [x] Subscribe `/cmd_vel` → diff-drive inverse kinematics (`otonav_control`) → `mjData->ctrl`
- [x] Publish `/odom` (integrate from wheel states), `/imu`, `/scan` from MuJoCo sensordata
- [~] Viewer behind `gui` launch arg; `gui:=false` runs headless. **Headless validated in Docker; in-process viewer is compiled out (no GLFW/display in CI)** — `gui:=true` logs a warning. Enable later via a `-DOTONAV_WITH_VIEWER=ON` build for local dev.
- [x] `otonav_bringup`: `sim.launch.py` with `gui` + `use_sim_time` args
- [~] Smoke check: drives via `/cmd_vel`. Validated headless in Docker by asserting `/odom` publishes (teleop-in-viewer deferred with the viewer above).
- Note: Phase 1 is built & validated inside the `docker/` Humble+MuJoCo image (host runs ROS 2 Jazzy, not Humble). This pulls the Phase 2 Dockerfile forward as the build harness; CI wiring (ccache, SIL, lint-all) remains Phase 2.

## Phase 2 — CI Pipeline
- [x] `docker/Dockerfile`: builder stage (toolchain + MuJoCo SDK + colcon build) → runner stage (ros-core + runtime libs + install tree) — landed in Phase 1 as the build harness
- [x] `docker/entrypoint.sh`: source ROS + workspace, `exec "$@"`; wire as ENTRYPOINT — landed in Phase 1
- [x] CI job: pre-commit/lint stage (`pre-commit/action`, advisory)
- [x] CI job: colcon build with ccache + `actions/cache` (MuJoCo SDK + ccache cached) inside `container: ros:humble-ros-base`
- [x] CI job: unit tests (gtest) for `otonav_control` kinematics with JUnit XML upload (`colcon test` + upload-artifact)
- [x] CI job: headless SIL smoke test — `add_launch_test(test_odom_smoke.py)` asserts `/clock /odom /imu /scan` publish, `ROS_LOCALHOST_ONLY=1` (runs inside `colcon test`)
- [~] Wire all jobs as required checks — `lint-build-test` is the required check (in the ruleset). `pre-commit` is advisory; add it via the UI when desired.
- [x] Keep `config/fastdds_ci_profile.xml` (UDPv4 unicast initialPeers fallback) with README note explaining when to use it
- [x] Verify: PR with a deliberately failing unit test is blocked; revert; green PR lands

## Phase 3 — SIL Integration Test
- [x] `otonav_nav`: go-to-goal P-controller node (param-driven gains, goal via `/goal_pose` or `goal_x/goal_y` params); pure law in `go_to_goal.hpp`
- [x] Add lidar stop-on-obstacle guard (front-sector min-range → zero cmd_vel); pure law in `obstacle_guard.hpp`
- [x] Unit tests for controller math (no sim needed): `test_go_to_goal.cpp`, `test_obstacle_guard.cpp`
- [x] `launch_testing` scenario `test_goal_reach.py`: headless sim + nav, goal reached < 20 s sim time, obstacle clearance ≥ 0.3 m, conditional waits only. (Added a front caster + stiffer wheel servo so the chassis stays level — a nose-down pitch was tipping the front lidar into the floor and false-triggering stop-on-obstacle.)
- [x] rosbag record of scenario topics; upload as CI artifact on failure (`if: failure()` in ci.yml)
- [x] Verify artifact path: break controller gain in a branch, download the failure bag from the run, revert

## Phase 4 — Release Engineering
- [x] CHANGELOG.md (keep-a-changelog format), v0.1.0 entry
- [x] Tag `v0.1.0`, cut `release/v0.1` branch
- [x] Release workflow: on tag push, build runner image, push to GHCR with semver + sha tags
- [x] README badges: CI status, latest release, GHCR image

## Phase 5 — Interview Drill
- [ ] Rehearse 3-minute repo tour: PR → checks → queue → tag → GHCR image → failure-bag artifact
- [ ] Rehearse Q&A anchors in architecture.md §7 out loud (EN)
- [ ] Prepare one open PR sitting in the merge queue for live demo
- [ ] Write 5 talking-point bullets mapping repo features to job-description lines (PR lifecycle, merge queues, repo hygiene, Docker/K8s, middleware)
