# OtoNav-CI

[![CI](https://github.com/ozkannceylan/robotics_cicd/actions/workflows/ci.yml/badge.svg)](https://github.com/ozkannceylan/robotics_cicd/actions/workflows/ci.yml)

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

## License

See [`LICENSE`](LICENSE).
