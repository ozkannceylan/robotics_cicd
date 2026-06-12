# OtoNav-CI

[![CI](https://github.com/ozkanceylan-dev/robotics_cicd/actions/workflows/ci.yml/badge.svg)](https://github.com/ozkanceylan-dev/robotics_cicd/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/ozkanceylan-dev/robotics_cicd?sort=semver)](https://github.com/ozkanceylan-dev/robotics_cicd/releases)
[![GHCR image](https://img.shields.io/badge/ghcr.io-runtime%20image-blue?logo=docker)](https://github.com/ozkanceylan-dev/robotics_cicd/pkgs/container/robotics_cicd)

End-to-end **ROS 2 Humble + MuJoCo** Software-in-the-Loop (SIL) **CI/CD platform**.
A deliberately simple differential-drive robot wrapped in a production-grade pipeline:
protected `main`, PR templates, required checks, native **merge queue**, and a versioned
release image. Built as interview evidence for a Robotics Software Integration Engineer role.

> Read [`project_plan.md`](project_plan.md) for phase scope, [`architecture.md`](architecture.md)
> for design decisions (ADRs), [`todo.md`](todo.md) for live task state, and
> [`CLAUDE.md`](CLAUDE.md) for the operating manual.

## Status

Phase 0 (repo & CI hygiene scaffold) is in place. Robot packages (`otonav_*`) are not
written yet — work proceeds phase-by-phase per `project_plan.md`.

## Contributing flow

Every change goes branch → PR → CI → **merge queue**. Never commit to `main`.

```bash
# 1. branch
git switch -c feature/<issue>-<slug>

# 2. install + run the local quality gate (mirrors CI)
pipx install pre-commit        # or: pip install --user pre-commit
pre-commit install
pre-commit run --all-files

# 3. open a PR (the template + CODEOWNERS review request appear automatically)
gh pr create --fill
```

Conventional commit prefixes: `feat:`, `fix:`, `ci:`, `docs:`, `test:`, `refactor:`.

## Build & test (once packages exist)

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select otonav_control otonav_nav && colcon test-result --verbose

# Run the sim headless (CI parity) / with viewer (local dev)
ros2 launch otonav_bringup sim.launch.py gui:=false
ros2 launch otonav_bringup sim.launch.py gui:=true
```

## CI / DDS in CI

CI builds and tests inside ROS 2 Humble + MuJoCo (MuJoCo cached, C++ via ccache),
runs `colcon test` (ament lint + gtest + a headless SIL launch test), and uploads
JUnit reports. DDS uses `ROS_LOCALHOST_ONLY=1` with `rmw_fastrtps_cpp` (ADR-3).
If loopback discovery ever fails on a runner, fall back to the documented profile:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_ci_profile.xml
```

## Releases

Pushing a semver tag (`vX.Y.Z`) builds the slim runner image and pushes it to GHCR
(`.github/workflows/release.yml`). See [`CHANGELOG.md`](CHANGELOG.md).

```bash
docker pull ghcr.io/ozkanceylan-dev/robotics_cicd:latest
docker run --rm ghcr.io/ozkanceylan-dev/robotics_cicd:latest \
  ros2 launch otonav_bringup sim.launch.py gui:=false
```

## License

See [`LICENSE`](LICENSE).
