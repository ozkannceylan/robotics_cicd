<!-- Conventional-commit title required: feat|fix|ci|docs|test|refactor(scope): summary -->

## What & why
<!-- One or two sentences. What does this PR change and why now? -->

## Linked work
- Closes #
- todo.md item: <!-- quote the checkbox line this PR ticks -->

## Test evidence
<!-- Paste the relevant output. CI parity required before merge. -->
- [ ] `pre-commit run --all-files` clean
- [ ] `colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release` passes
- [ ] `colcon test` + `colcon test-result --verbose` pass
- [ ] (if applicable) headless SIL test passes: `launch_test ...`

## Safety / regression checklist
- [ ] No direct-to-`main` commits; branch follows `feature/<issue>-<slug>` or `fix/<issue>-<slug>`
- [ ] No wall-clock `sleep()` in tests; sim time + conditional waits only (ADR-2)
- [ ] No MuJoCo types leaked outside `otonav_mujoco_bridge` (Hard Rule #4)
- [ ] No binaries / large files added (Hard Rule #5)
- [ ] Versions referenced from the single source of truth, not duplicated (Hard Rule #6)
- [ ] `todo.md` checkbox ticked in this PR
