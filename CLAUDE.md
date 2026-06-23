# CLAUDE.md — OtoNav-CI

Operating manual for Claude Code working in this repository. Read `project_plan.md` for phase scope and `architecture.md` for design decisions before writing code. Update `todo.md` as tasks complete.

> **Machine-wide standards** (settings, Obsidian vault / Second Brain integration, workflow orchestration, task management, core principles) live in the vault at `[VAULT_ROOT]/claude-code-files/BASE_CLAUDE.md`. Read it once per session; the project-specific rules below take precedence where they conflict.

## Project Context
End-to-end ROS 2 Humble + MuJoCo SIL CI/CD platform, built as a portfolio project demonstrating Robotics Software Integration capabilities. Priority order: **repo hygiene & CI infrastructure > sim bridge correctness > navigation sophistication.** When in doubt, simplify the robot, never the pipeline.

## Current State (read before assuming code exists)
The repo is **pre-implementation**. Only `README.md`, `LICENSE`, `.gitignore`, and the four planning docs (`CLAUDE.md`, `architecture.md`, `project_plan.md`, `todo.md`) exist. **None of the `otonav_*` packages in `architecture.md §2` are written yet** — don't search for them, build them. Work the phases in `project_plan.md` / `todo.md` top-to-bottom; Phase 0 (repo hygiene) and Phase 2 (CI) are non-negotiable, everything else has cut lines.

## Architecture at a Glance
Read `architecture.md` for the full picture; the load-bearing model is one data loop:
`otonav_nav` → `/cmd_vel` → `otonav_control` (diff-drive kinematics) → `RobotHardwareInterface` → MuJoCo `mj_step` → sensor data (`/odom`, `/scan`, `/imu`) → back to nav. The bridge owns `/clock`; **all nodes run `use_sim_time:=true`** (ADR-2). The `RobotHardwareInterface` is the sim-to-real seam — see Hard Rule #4 and ADR-4. The whole project's thesis is that the *pipeline* (CI, merge queue, release) is production-grade while the *robot* is deliberately trivial.

## Environment & Stack
- Ubuntu 22.04, ROS 2 Humble, MuJoCo 3.3.x (pin the exact version in one place: `docker/Dockerfile` ARG)
- C++17 with rclcpp for all nodes; Python only for launch files and launch_testing
- Build: colcon. Containers: Docker multi-stage. CI: GitHub Actions + native Merge Queue.

## Commands
```
# Build (always from workspace root /ws or repo root)
source /opt/ros/humble/setup.bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Unit tests
colcon test --packages-select otonav_control otonav_nav && colcon test-result --verbose

# Run a single gtest case (after build): filter by ctest name or run the test binary directly
colcon test --packages-select otonav_control --ctest-args -R <test_name>
./build/otonav_control/<test_binary> --gtest_filter='Suite.Case'

# Run a single launch_testing integration test
ROS_LOCALHOST_ONLY=1 launch_test otonav_bringup/test/<test_file>.py

# Run sim with viewer (local dev)
ros2 launch otonav_bringup sim.launch.py gui:=true

# Run headless (what CI runs)
ros2 launch otonav_bringup sim.launch.py gui:=false

# SIL integration test (CI parity)
ROS_LOCALHOST_ONLY=1 launch_test otonav_bringup/test/test_goal_reach.py

# Lint locally before any commit
pre-commit run --all-files
```

## Hard Rules
1. **Never commit directly to `main`.** Every change: branch → PR → CI → merge queue. Branch naming: `feature/<issue>-<slug>`, `fix/<issue>-<slug>`.
2. **Conventional commits**: `feat:`, `fix:`, `ci:`, `docs:`, `test:`, `refactor:`. Scope optional: `feat(bridge): ...`
3. **No wall-clock sleeps in tests.** Use sim time (`use_sim_time:=true`) and conditional waits. Any `sleep()` in test code is a bug.
4. **No MuJoCo types outside `otonav_mujoco_bridge`.** Control/nav code talks only to ROS messages and `RobotHardwareInterface`. This boundary is the whole sim-to-real story — protect it.
5. **No binary/large files in git** (no rosbags, no meshes over ~1 MB). Test artifacts are CI artifacts, not repo content.
6. **Single source of truth for versions**: MuJoCo version, image tags, and package versions are defined once and referenced, never duplicated.
7. **Headless is the default.** The viewer is opt-in via `gui:=true`. Never make rendering a precondition for physics or tests.
8. **CI must stay under ~10 min per PR.** Use ccache + actions/cache; if a step blows the budget, split or cache it before adding features.

## Code Conventions
- clang-format (config in repo root, version-pinned to match CI), enforced by pre-commit
- ament linters (`ament_cmake_lint_cmake`, `ament_cmake_xmllint`, cpplint via ament) wired into colcon test
- Every node: parameters declared with descriptors, no magic numbers; topic names defined in a single header/yaml
- gtest for unit tests; launch_testing for integration. New logic without a test does not merge.

## Definition of Done (any task)
- [ ] Code + test on a feature branch
- [ ] `pre-commit run --all-files` clean
- [ ] `colcon build` and `colcon test` pass locally
- [ ] PR uses the template, references the todo item, passes CI, lands via merge queue
- [ ] `todo.md` checkbox ticked in the same PR

## What NOT to Do
- Do not add Nav2, SLAM, or any heavyweight nav stack — out of scope, see project_plan.md cut lines
- Do not introduce ros2_control yet (documented stretch path, ADR-4)
- Do not write a custom DDS profile unless `ROS_LOCALHOST_ONLY=1` demonstrably fails in CI (ADR-3)
- Do not optimize the MJCF model visually; physics correctness only
