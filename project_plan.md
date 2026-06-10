# Project Plan: OtoNav-CI

**Goal:** A working end-to-end ROS 2 + MuJoCo Software-in-the-Loop (SIL) CI/CD platform on GitHub, built as live interview evidence for a Robotics Software Integration Engineer role (MOTOR Ai, Berlin). Interview: Friday. Available time: ~2 working days.

**Strategic priority:** The role is an *integration* role, not a robotics development role. The job description weights PR lifecycle management, merge queues, repo hygiene, CI/CD, Docker, and middleware troubleshooting far above navigation algorithms. Therefore:

> Keep the robot deliberately simple. Make the integration infrastructure production-grade.

A trivial differential-drive robot with a bulletproof CI pipeline, merge queue, and release process beats a sophisticated robot with a hand-wavy pipeline.

---

## Scope Decisions (locked)

| Decision | Choice | Rationale |
|---|---|---|
| Robot | 2-wheel differential drive, MJCF model | Minimal physics, well-understood kinematics, fast to validate |
| Sim | MuJoCo 3.3.x (pin exact version) | Job-relevant, headless-friendly, no GPU needed for physics |
| ROS | ROS 2 Humble on Ubuntu 22.04 | LTS, matches doc baseline |
| Bridge language | C++ (rclcpp) | Job requires solid C++; this is the showcase code |
| Nav logic | Go-to-goal + stop-on-obstacle only | Enough for a meaningful SIL test scenario, nothing more |
| Headless strategy | Runtime flag (`gui:=false`), NOT compile-time ifdef | Single binary for dev and CI; physics loop needs no GL at all |
| HAL story | Abstract `RobotHardwareInterface` class; MuJoCo impl now, real-HW impl is a stub | Demonstrates sim-to-real architecture without ros2_control setup overhead |
| DDS in CI | `ROS_LOCALHOST_ONLY=1` first; FastDDS unicast-peers profile as fallback | Simplest thing that works on GitHub runners; document the fallback |
| CI | GitHub Actions + native Merge Queue + branch protection | Directly mirrors the job's responsibilities |

## Phases

### Phase 0 — Repo & Hygiene Scaffold (~1h) — DO THIS FIRST
The infrastructure the interviewer cares about most. All of it works before any robot code exists.
- New GitHub repo, trunk-based layout (`main` protected)
- Branch protection rules: required CI checks, no direct pushes, 1 approval (self-review via second account or note the limitation)
- Native GitHub Merge Queue enabled on `main`
- `.github/PULL_REQUEST_TEMPLATE.md`, `CODEOWNERS`, issue templates
- `pre-commit` config: clang-format, ament linters hooks
- Conventional commit convention documented
- **Definition of done:** an empty-ish PR goes through template → CI stub → merge queue → merge.

### Phase 1 — Minimal Sim Bridge (~4h)
- `otonav_description`: MJCF diff-drive model (2 actuated wheels, caster, 1 lidar site, IMU site)
- `otonav_mujoco_bridge` (C++): loads MJCF, runs `mj_step` loop on sim clock, publishes `/clock`, `/odom`, `/scan`, `/imu`; subscribes `/cmd_vel`
- `RobotHardwareInterface` abstraction between control logic and actuation backend
- Optional viewer behind `gui:=true` launch arg
- **Definition of done:** teleop the robot in MuJoCo viewer via `/cmd_vel`; same launch runs headless with `gui:=false`.

### Phase 2 — CI Pipeline (~4h)
- Multi-stage Dockerfile (builder → slim runner), correct entrypoint (`source` + `exec "$@"`)
- Workflow stages: pre-commit/lint → colcon build (ccache + actions/cache) → unit tests (gtest) → headless SIL smoke test
- JUnit XML test reports surfaced in PR checks
- `ROS_LOCALHOST_ONLY=1` for DDS; keep fallback FastDDS profile in `config/` with explanation
- **Definition of done:** a PR that breaks a unit test is blocked; a green PR enters the merge queue automatically.

### Phase 3 — SIL Integration Test (~3h)
- `otonav_nav`: go-to-goal P-controller + lidar stop-on-obstacle
- `launch_testing` scenario: spawn sim headless, command goal, assert goal reached within sim-time budget (use `use_sim_time`, never wall-clock sleeps)
- On failure: record rosbag of the run, upload as CI artifact
- **Definition of done:** integration test passes in CI; a deliberately broken controller produces a downloadable rosbag artifact.

### Phase 4 — Release Engineering (~1.5h)
- Tag `v0.1.0` (semver), generate CHANGELOG, cut a `release/v0.1` branch
- Release workflow: on tag, build and push the runtime Docker image to GHCR
- **Definition of done:** GHCR shows a versioned image traceable to a tag and changelog.

### Phase 5 — Interview Drill (remaining time)
- Walk through `architecture.md` out loud; rehearse the Q&A section
- Prepare a 3-minute repo tour: PR → checks → merge queue → release tag → GHCR image
- Have one open PR live in the queue during the interview if screen-sharing is possible

## Cut Lines (if time runs out)
Cut in this order: Phase 4 GHCR push → rosbag-on-failure artifact → obstacle avoidance (keep go-to-goal only). **Never cut Phase 0 or Phase 2** — they are the job.

## Stretch (post-interview)
- Migrate bridge to `ros2_control` + `mujoco_ros2_control` hardware plugin
- industrial_ci adoption for standardized ROS CI
- Dependabot + rosdep automation, multi-arch image builds
