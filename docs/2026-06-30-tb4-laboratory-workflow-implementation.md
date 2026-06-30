# TB4 Laboratory Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the recovered TB4 laboratory workflow design into a sequence
of small, reviewable branches that take the project from the current simulated
and bag-oriented implementation to a reproducible real-lab TurtleBot4 flow.

**Architecture:** Treat the work as a gated integration program, not a single
large feature. Mechanical renames happen first, then shared platform contracts,
then sensor correctness, then real TB4 support for Parte B and Parte C, and
only then the final operator runbook.

**Tech Stack:** ROS 2, Python launch files, Python ROS nodes, custom ROS
messages, TurtleBot3 simulation, TurtleBot4 real/bag topics, ArUco, Graph SLAM,
MCL, A*, RViz, colcon, pytest.

---

## Source Design and Baseline

Source design:

- `docs/2026-06-30-tb4-laboratory-workflow-design.md`

Current baseline:

- The current branch is `doc-laboratory-runbook`.
- The design document is already restored in `/docs`.
- This file is a planning artifact only. It must not change code, launch files,
  tests, package manifests, or installed resources.

Global invariants for all future branches:

- Parte A and Parte B are launched separately; do not force their ROS contracts
  into a single runtime shape.
- `graph_slam_node` and `mcl_localization` publish `map -> odom` in different
  stages, never as simultaneous producers in the same profile.
- `state_machine` remains the only normal `/cmd_vel` producer in Parte B/C.
- Renaming branches are mechanical only: no algorithm, parameter, or behavior
  changes.
- Algorithmic corrections require diagnostic evidence before tuning.
- `VisualObservability` is kept unless a future branch proves it is not used at
  runtime. At the current baseline it is connected to Graph SLAM diagnostics,
  so it is not a test-only message.

## Branch Sequence

Implement in this order:

1. `doc-laboratory-runbook`
2. `chore/rename-tp-interfaces`
3. `chore/rename-parte-a-package`
4. `feat/platform-profiles`
5. `fix/lidar-tf-projection`
6. `feat/tb4-namespace-support`
7. `feat/parte-b-real`
8. `feat/parte-c-real-lab`
9. `docs/lab-final-runbook`

Do not merge later branches before earlier gates pass. If a branch exposes a
diagnostic failure that changes the design, stop and update the relevant plan or
runbook before continuing.

## Common Validation Commands

Run these from the repository root before completing any branch unless the
branch-specific section narrows the scope:

```bash
python3 tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -v
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_a_slam_aruco tp_final_ws/src/tp_c_mission
git diff --check
```

When ROS 2 is available, run:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
python3 -m pytest src/tp_a_slam_aruco/test src/tp_b_navigation/test src/tp_c_mission/test -q
```

For rename branches, update the package names in the ROS validation command to
the names produced by that branch.

## Task 1: `doc-laboratory-runbook`

**Objective:** Create an early operator-facing laboratory guide from the design,
before changing code.

**Scope:**

- Add a first-pass lab runbook in `/docs` that explains the real-lab sequence:
  notebook setup, robot selection, preflight, onboard bag recording, bag copy,
  Parte A pass 1, Parte A pass 2, map inspection, Parte B real gates, Parte C
  real gates, shutdown, and artifact retention.
- Use explicit execution locations in command blocks: `[TB4 por SSH]`,
  `[Notebook laboratorio]`, `[RViz]`, and `[Accion fisica]`.
- Keep simulation commands out of the real-lab path unless placed in a clearly
  separate reference section.

**Out of scope:**

- Code changes.
- Launch changes.
- Package renames.
- New algorithms or parameter tuning.

**Risks:**

- The guide can accidentally imply that Parte A and Parte B run together.
- Commands can become too specific to `tb4_0` and fail for `tb4_1`.
- Safety instructions can be buried behind optional diagnostics.

**Validation:**

```bash
git diff --check
```

Manual review checklist:

- The guide starts from a clean lab session.
- Safety and stop procedures appear before autonomous movement.
- The bag contract accepts a selected namespace instead of hardcoding `tb4_0`.
- The guide preserves artifacts under a run-specific directory, not under
  package `share/` or personal paths.

**Ready when:**

- A human can follow the guide from robot identification through artifact
  retention without reading the design document.
- All commands that depend on the robot namespace show the namespace as a
  chosen input.

**Expected commit:**

```bash
git commit -m "docs: add TB4 laboratory runbook draft"
```

## Task 2: `chore/rename-tp-interfaces`

**Objective:** Rename the legacy shared interface package to `tp_interfaces`
mechanically.

**Scope:**

- Rename the package directory, package manifest, CMake project, resource
  marker, import references, dependency names, launch references, tests, and
  documentation references.
- Keep message definitions unchanged unless a reference proves a message is
  unreachable at runtime.
- Preserve `VisualObservability` by default because current diagnostics import
  and use it through Graph SLAM diagnostic code.

**Out of scope:**

- Message schema changes.
- Behavior changes in Parte A, B, or C.
- Removing `VisualObservability` without a runtime reference audit.

**Risks:**

- Python imports can pass locally from stale build artifacts while package
  manifests are wrong.
- Generated ROS interfaces can retain old names in an unclean install space.
- Mechanical search/replace can rewrite prose examples incorrectly.

**Validation:**

```bash
rg -n "tp_slam"_interfaces tp_final_ws docs README.md
python3 -m compileall -q tp_final_ws/src/tp_a_slam_aruco tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
python3 -m pytest src/tp_a_slam_aruco/test src/tp_b_navigation/test src/tp_c_mission/test -q
```

**Ready when:**

- `rg -n "tp_slam"_interfaces` returns no live code or package references.
- The renamed interface package builds from a clean install.
- All tests that import custom messages pass.

**Expected commit:**

```bash
git commit -m "chore: rename shared interfaces package"
```

## Task 3: `chore/rename-parte-a-package`

**Objective:** Rename the legacy Parte A SLAM package to `tp_a_slam_aruco`
mechanically.

**Scope:**

- Rename the package directory, Python module, setup metadata, resource marker,
  launch package references, executable references, tests, and documentation
  references.
- Update commands in docs to use the new package name.
- Keep the launch names `parte_a_slam.launch.py` and `parte_a_mapa.launch.py`
  unless a later design explicitly renames launch files.

**Out of scope:**

- Graph SLAM behavior changes.
- LIDAR projection changes.
- Topic contract changes.

**Risks:**

- Entry points can continue pointing at the old Python module.
- Tests can import from stale `__pycache__` or old install artifacts.
- Documentation may mix old package names with new command examples.

**Validation:**

```bash
rg -n "tp_slam"_aruco tp_final_ws docs README.md
python3 -m compileall -q tp_final_ws/src/tp_a_slam_aruco tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
python3 -m pytest src/tp_a_slam_aruco/test src/tp_b_navigation/test src/tp_c_mission/test -q
```

**Ready when:**

- `ros2 launch tp_a_slam_aruco parte_a_slam.launch.py --show-args` works.
- `ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py --show-args` works.
- No live code imports or launch references use the legacy package name.

**Expected commit:**

```bash
git commit -m "chore: rename Parte A SLAM package"
```

## Task 4: `feat/platform-profiles`

**Objective:** Introduce explicit platform profiles:
`simulation_tb3`, `bag_tb4`, and `real_tb4`.

**Scope:**

- Add a shared profile contract that resolves topics, frames, clock mode,
  landmark source, camera topics, scan topic, odometry topics, and command
  topic.
- Keep algorithms shared; profiles only select configuration and enabled nodes.
- Ensure `simulation_tb3` preserves the current Gazebo behavior.
- Ensure `bag_tb4` uses simulated time and bag topics.
- Ensure `real_tb4` uses wall time and real robot topics.

**Out of scope:**

- TB4 namespace robustness beyond the profile input shape.
- LIDAR projection fixes.
- Real navigation behavior changes.

**Risks:**

- A profile can silently change simulation defaults.
- `real_tb4` can accidentally launch simulation-only nodes such as
  `landmark_sensor`.
- `bag_tb4` can fail if nodes do not consistently use `use_sim_time=true`.

**Validation:**

```bash
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
python3 tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -v
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
ros2 launch tp_b_navigation parte_b.launch.py profile:=simulation_tb3 --show-args
ros2 launch tp_c_mission parte_c_real.launch.py profile:=real_tb4 --show-args
python3 -m pytest src/tp_b_navigation/test src/tp_c_mission/test -q
```

**Ready when:**

- Launch argument inspection shows the three profiles and their resolved
  defaults.
- The simulation profile starts the same logical seven Parte B nodes as before.
- The real profile does not start virtual landmark publishers or sensors.

**Expected commit:**

```bash
git commit -m "feat: add platform execution profiles"
```

## Task 5: `fix/lidar-tf-projection`

**Objective:** Make every LIDAR consumer respect scan metadata and TF instead
of assuming fixed TB4 extrinsics or forward angle conventions.

**Scope:**

- Update Parte A occupancy-grid projection to use `scan.header.frame_id`,
  `angle_min`, `angle_max`, `angle_increment`, finite range checks, and TF from
  LIDAR frame to base frame at the scan timestamp.
- Keep `lidar_tx=-0.04` and `lidar_yaw=pi/2` only as an explicit fallback for
  known bag/profile cases where TF is unavailable.
- Audit Parte B obstacle monitoring and any scan projection utilities for the
  same frame and angle assumptions.
- Add diagnostics that report whether TF or fallback extrinsics were used.

**Out of scope:**

- Occupancy log-odds tuning.
- Global dilation to preserve thin objects.
- MCL noise tuning.

**Risks:**

- TF lookup failures can stop map generation if fallback behavior is not
  explicit.
- Fixing scan geometry can change existing map appearance; this is expected but
  must be measured.
- Using one pose per scan can still produce distortion during fast rotations;
  treat that as a later diagnostic unless evidence demands it here.

**Validation:**

```bash
python3 -m compileall -q tp_final_ws/src/tp_a_slam_aruco tp_final_ws/src/tp_b_navigation
git diff --check
```

With ROS 2 and a known bag:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation
source install/setup.bash
python3 -m pytest src/tp_a_slam_aruco/test src/tp_b_navigation/test -q
ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:=/tmp/trayectoria.json \
  map_output:=/tmp/tb4-map-check
```

**Ready when:**

- Unit tests cover scan angle handling and invalid ranges.
- Diagnostics identify TF versus fallback extrinsics.
- A map generated from the same bag no longer depends on hardcoded scan yaw as
  the primary path.

**Expected commit:**

```bash
git commit -m "fix: project lidar scans through TF"
```

## Task 6: `feat/tb4-namespace-support`

**Objective:** Make the real and bag workflows robust for both `tb4_0` and
`tb4_1`.

**Scope:**

- Select `robot_namespace` once per run and feed it to launch files,
  validators, diagnostics, and run artifacts.
- Derive default topics from that namespace while allowing explicit remaps.
- Validate that critical topics do not point to the other TB4 namespace.
- Record resolved topics, frames, and fallback sources under the run artifact
  directory.

**Out of scope:**

- Supporting multiple robots in the same run.
- Running both `tb4_0` and `tb4_1` simultaneously.
- Changing algorithm parameters for one robot unless documented as a fallback
  calibration source.

**Risks:**

- Textual replacement of `/tb4_0` with `/tb4_1` can miss frame IDs or camera
  topics.
- Frame names may not be namespace-prefixed even when topics are.
- Output files can collide if the namespace is not included in `run_id` or
  result paths.

**Validation:**

```bash
rg -n "/tb4_0|tb4_0|/tb4_1|tb4_1" tp_final_ws/src docs README.md
python3 -m compileall -q tp_final_ws/src/tp_a_slam_aruco tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
ros2 launch tp_a_slam_aruco parte_a_slam.launch.py robot_namespace:=tb4_0 --show-args
ros2 launch tp_a_slam_aruco parte_a_slam.launch.py robot_namespace:=tb4_1 --show-args
ros2 launch tp_c_mission parte_c_real.launch.py robot_namespace:=tb4_0 --show-args
ros2 launch tp_c_mission parte_c_real.launch.py robot_namespace:=tb4_1 --show-args
```

**Ready when:**

- Both TB4 namespaces resolve to consistent topic sets.
- Validation fails loudly when a critical topic is assigned to the wrong robot.
- Run artifacts include namespace and resolved platform configuration.

**Expected commit:**

```bash
git commit -m "feat: support selectable TB4 namespace"
```

## Task 7: `feat/parte-b-real`

**Objective:** Run Parte B on the real TB4 using the map and ArUco landmarks
produced by Parte A.

**Scope:**

- Add or update a real Parte B launch path that starts `map_loader`,
  `mcl_localization`, `global_planner`, `obstacle_monitor`, `state_machine`,
  ArUco detection, and the ArUco-to-MCL observation adapter.
- Ensure the real profile does not start `landmark_publisher`,
  `landmark_sensor`, `/calc_odom`, duplicate TB4 drivers, or Graph SLAM as a
  concurrent `map -> odom` producer.
- Add safety gates for stale MCL pose, excessive covariance, stale scan, and
  missing TF before global obstacle insertion or autonomous movement.

**Out of scope:**

- Parte C cone mission behavior.
- MCL algorithm redesign.
- Changing planner clearance weights without measured evidence.

**Risks:**

- MCL currently does not use the occupancy grid to reject particles inside
  walls; document this limit in diagnostics and safety criteria.
- Real ArUco observations can be sparse, causing multimodal localization.
- Obstacle projection can create false positives if pose or TF is stale.

**Validation:**

```bash
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
python3 -m pytest src/tp_b_navigation/test src/tp_c_mission/test -q
ros2 launch tp_b_navigation parte_b.launch.py profile:=real_tb4 --show-args
```

Lab gates:

- B1: localization without autonomous movement.
- B2: basic navigation to close, clear goals at reduced speed.
- B3: dynamic obstacle detection, visualization, replanning, and recovery in
  incremental steps.

**Ready when:**

- MCL is the only `map -> odom` producer in the real Parte B profile.
- `state_machine` is the only normal velocity command producer.
- The robot reaches multiple operator goals in the real map at reduced speed
  without collision.

**Expected commit:**

```bash
git commit -m "feat: run Parte B on real TB4 map"
```

## Task 8: `feat/parte-c-real-lab`

**Objective:** Run the red-cone mission on the real TB4 workflow.

**Scope:**

- Use the same map and ArUco landmarks accepted by Parte A/B.
- Use real RGB, aligned metric depth, `CameraInfo`, camera TF, MCL pose, and A*
  reachability checks.
- Confirm red cone detections temporally and publish cone pose in `map`.
- Treat non-red cones as dynamic obstacles when appropriate.
- Ensure `tp_c_mission` sends `/mission_goal` to `state_machine` and never
  publishes `/cmd_vel`.

**Out of scope:**

- Dynamic costmap beyond the existing obstacle layer.
- `custom_casa_obs2`.
- Direct visual servoing to the cone without A* validation.

**Risks:**

- RGB and depth streams can be misaligned or stale.
- Seeing the cone does not imply a reachable approach pose.
- Mission exploration can repeat exhausted goals if failure state is not
  tracked.

**Validation:**

```bash
python3 -m compileall -q tp_final_ws/src/tp_c_mission tp_final_ws/src/tp_b_navigation
git diff --check
```

With ROS 2:

```bash
cd tp_final_ws
colcon build --packages-select tp_interfaces tp_a_slam_aruco tp_b_navigation tp_c_mission
source install/setup.bash
python3 -m pytest src/tp_c_mission/test src/tp_b_navigation/test -q
ros2 launch tp_c_mission parte_c_real.launch.py profile:=real_tb4 --show-args
```

Lab gates:

- C1: perception with robot stopped.
- C2: approach pose validation with A* and safe final distance.
- C3: full mission with distractors, cancellation, and safe stops on stale
  localization, scan, or vision.

**Ready when:**

- The mission publishes `FOUND` for the red cone and ignores distractors.
- The robot does not cross walls or assume free space from camera visibility.
- Manual cancel and emergency stop procedures are documented and tested.

**Expected commit:**

```bash
git commit -m "feat: run Parte C mission in real lab"
```

## Task 9: `docs/lab-final-runbook`

**Objective:** Replace the draft runbook with the final step-by-step lab guide
after the implementation branches have passed.

**Scope:**

- Update the root README or a clearly linked `/docs` guide as the executable
  day-of-lab procedure.
- Include terminal layout, execution locations, selected namespace, run ID,
  artifact paths, preflight checks, safety checks, commands, acceptance gates,
  troubleshooting, and cleanup.
- Keep deeper rationale and diagnostic plots in specific docs under `docs/`.

**Out of scope:**

- New code changes.
- Algorithm tuning.
- Changing contracts discovered during implementation without a matching code
  branch.

**Risks:**

- The final guide can drift from actual launch arguments.
- It can mix simulation shortcuts with real-lab commands.
- It can omit the stop procedure because it was already described in an earlier
  draft.

**Validation:**

```bash
git diff --check
```

Manual review checklist:

- Every command names where it runs.
- The guide works for `tb4_0` and `tb4_1`.
- Safety and stop procedures are visible before autonomous commands.
- The final artifact checklist includes bag, resolved platform YAML,
  trajectory JSON, map YAML/PGM, landmarks, diagnostics, and logs.

**Ready when:**

- A fresh lab notebook can follow the guide without reading implementation
  notes.
- All gates A1/A2/A3, B1/B2/B3, and C1/C2/C3 are represented as explicit
  checkpoints.

**Expected commit:**

```bash
git commit -m "docs: add final TB4 lab runbook"
```

## Integrated Definition of Done

The full program is done when one TB4 in the lab can complete this sequence:

- Record one onboard acquisition.
- Validate and replay the bag on the notebook.
- Produce Graph SLAM trajectory, ArUco landmarks, and a navigable map.
- Localize the real robot with MCL in that map.
- Reach multiple operator goals without collision.
- Detect and route around a newly added obstacle without losing localization.
- Find the red cone among distractors.
- Stop safely on stale localization, scan, vision, TF, or manual cancel.
- Preserve all commands, diagnostics, and artifacts for the run.

## Plan Maintenance Rules

- If a branch discovers a contract mismatch, update the design or this plan
  before continuing.
- If a branch requires algorithm tuning, capture the diagnostic evidence in
  `/docs` before changing parameters.
- If a validation command changes after package renames, update this document
  and the runbook in the same branch that changes the command.
- Keep every branch reviewable on its own; do not batch unrelated fixes into a
  later integration branch.
