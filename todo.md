# TODO — OtoNav-CI

Work top to bottom. Each checkbox = one PR where feasible. Phases mirror `project_plan.md`. If time pressure hits, consult the Cut Lines there — never skip Phase 0 or Phase 2.

## Phase 0 — Repo & Hygiene Scaffold
- [x] Create GitHub repo, push initial commit with the four md docs + LICENSE + .gitignore (ROS/C++/Python)
- [x] Add `.github/PULL_REQUEST_TEMPLATE.md` (description, linked issue, test evidence, safety/regression checklist)
- [x] Add `CODEOWNERS` (own the critical paths: `/otonav_mujoco_bridge/`, `/.github/`)
- [x] Add issue templates (bug, task)
- [x] Enable branch protection on `main`: require PR, require status checks, no force-push
- [x] Enable native Merge Queue on `main`
- [x] Add `pre-commit` config: clang-format, trailing-whitespace, yaml check; document install in README
- [x] Add stub CI workflow (checkout + echo) so the required check exists from day one
- [x] Verify: open a trivial PR → template appears → stub check runs → merges via queue

## Phase 1 — Minimal Sim Bridge
- [ ] `otonav_description`: MJCF diff-drive model (chassis, 2 actuated wheels w/ velocity actuators, caster, lidar site, IMU site); validate with `simulate` viewer manually
- [ ] `otonav_mujoco_bridge` package skeleton (C++, ament_cmake), MuJoCo found via CMake (path from env/ARG)
- [ ] `RobotHardwareInterface` abstract class + `MujocoInterface` implementation + `CanBusInterface` stub
- [ ] Bridge node: mj_step loop, publishes `/clock` (sim time authority)
- [ ] Subscribe `/cmd_vel` → diff-drive inverse kinematics → `mjData->ctrl`
- [ ] Publish `/odom` (integrate from wheel states), `/imu`, `/scan` from MuJoCo sensordata
- [ ] Viewer behind `gui` launch arg; confirm `gui:=false` runs with no display
- [ ] `otonav_bringup`: `sim.launch.py` with `gui` + `use_sim_time` args
- [ ] Smoke check: teleop_twist_keyboard drives the robot in viewer

## Phase 2 — CI Pipeline
- [ ] `docker/Dockerfile`: builder stage (toolchain + MuJoCo SDK + colcon build) → runner stage (ros-core + runtime libs + install tree)
- [ ] `docker/entrypoint.sh`: source ROS + workspace, `exec "$@"`; wire as ENTRYPOINT
- [ ] CI job: pre-commit/lint stage
- [ ] CI job: colcon build with ccache + `actions/cache` keyed on lockable inputs
- [ ] CI job: unit tests (gtest) for `otonav_control` kinematics (forward/inverse round-trip, odom integration) with JUnit XML upload
- [ ] CI job: headless SIL smoke test (launch sim `gui:=false`, assert `/odom` publishing within sim-time budget), `ROS_LOCALHOST_ONLY=1`
- [ ] Wire all jobs as required checks for merge queue
- [ ] Keep `config/fastdds_ci_profile.xml` (UDPv4 unicast initialPeers fallback) with README note explaining when to use it
- [ ] Verify: PR with a deliberately failing unit test is blocked; revert; green PR lands via queue

## Phase 3 — SIL Integration Test
- [ ] `otonav_nav`: go-to-goal P-controller node (param-driven gains, goal via topic)
- [ ] Add lidar stop-on-obstacle guard (min-range threshold → zero cmd_vel)
- [ ] Unit tests for controller math (no sim needed)
- [ ] `launch_testing` scenario `test_goal_reach.py`: headless sim + nav, assert goal reached < 20 s sim time, obstacle clearance ≥ 0.3 m, conditional waits only
- [ ] rosbag record of scenario topics; upload as CI artifact on failure
- [ ] Verify artifact path: break controller gain in a branch, download the failure bag from the run, revert

## Phase 4 — Release Engineering
- [ ] CHANGELOG.md (keep-a-changelog format), v0.1.0 entry
- [ ] Tag `v0.1.0`, cut `release/v0.1` branch
- [ ] Release workflow: on tag push, build runner image, push to GHCR with semver + sha tags
- [ ] README badges: CI status, latest release

## Phase 5 — Interview Drill
- [ ] Rehearse 3-minute repo tour: PR → checks → queue → tag → GHCR image → failure-bag artifact
- [ ] Rehearse Q&A anchors in architecture.md §7 out loud (EN)
- [ ] Prepare one open PR sitting in the merge queue for live demo
- [ ] Write 5 talking-point bullets mapping repo features to job-description lines (PR lifecycle, merge queues, repo hygiene, Docker/K8s, middleware)
