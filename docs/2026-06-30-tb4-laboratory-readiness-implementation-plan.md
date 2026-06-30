# TB4 Laboratory Readiness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the remaining runtime gaps between the current TB3/TB4
implementation and a safe, reproducible end-to-end laboratory run on a physical
TurtleBot4.

**Architecture:** Preserve the existing three-profile architecture.
`simulation_tb3` continues using virtual landmarks and Gazebo truth;
`bag_tb4` processes namespaced recorded data with simulated time; `real_tb4`
uses physical sensors, ArUco observations and wall time. Correct TF routing and
MCL temporal behavior first, then fix Parte A scan synchronization, enforce the
real RGB-D contract, preserve per-stage artifacts, add runtime smoke tests, and
finish with gated bag and physical-robot validation.

**Tech Stack:** ROS 2 Humble, Python, ROS 2 launch, tf2, rosbag2, NumPy, GTSAM,
OpenCV ArUco, pytest, colcon, Miniforge/RoboStack, TurtleBot3 simulation and
TurtleBot4 hardware.

---

## Source documents and baseline

Read these before changing code:

- `docs/2026-06-30-tb4-laboratory-workflow-design.md`
- `docs/2026-06-30-tb4-laboratory-workflow-implementation.md`
- `docs/2026-06-30-tb4-laboratory-runbook.md`
- `AGENTS.md`

Expected baseline:

- branch containing commit `0d91d24`;
- package names `tp_interfaces` and `tp_a_slam_aruco`;
- shared profiles in `tp_platform`;
- current portable test result: `171 passed, 2 skipped`;
- current bag contract passes for `tp_final_ws/bags/laberinto`;
- no tracked local changes before starting a task.

Before Task 1, push the baseline branch and integrate it into `main` without
squashing its existing task-oriented commits. Re-run the common verification
on the resulting `main`, then create `fix/real-tb4-tf-routing` from that commit.
Do not begin the corrective branches from the older `7b5fb4f` main baseline.

The profiles must retain these landmark sources:

| Profile | Landmark source | Nodes |
|---|---|---|
| `simulation_tb3` | virtual | `landmark_publisher` + `landmark_sensor` |
| `bag_tb4` | ArUco from bag | detector + identified-observation adapter |
| `real_tb4` | physical ArUco | detector + identified-observation adapter |

Do not remove the virtual simulation path while fixing the real path.

## Branch and merge order

Create each branch from an updated `main`, merge it only after its gate passes,
then create the next branch:

1. `fix/real-tb4-tf-routing`
2. `fix/mcl-prediction-publication`
3. `fix/parte-a-scan-odom-bracketing`
4. `fix/real-sensor-time-depth-contract`
5. `fix/run-artifacts-and-rviz`
6. `test/tb4-runtime-smoke`
7. `docs/tb4-lab-readiness`

Do not combine these branches. Each commit must be independently revertible.
Parameter tuning for MCL, A*, clearance, log-odds or obstacle TTL remains out of
scope unless a recorded diagnostic run demonstrates the need.

## Common environment and verification

On the current macOS development machine:

```bash
source "${HOME}/miniforge3/etc/profile.d/conda.sh"
conda activate rosenv_mf
```

After package renames, remove only stale renamed-package products:

```bash
rm -rf \
  tp_final_ws/build/tp_slam_interfaces \
  tp_final_ws/build/tp_slam_aruco \
  tp_final_ws/install/tp_slam_interfaces \
  tp_final_ws/install/tp_slam_aruco
```

Portable checks for every branch:

```bash
python3 tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -v
python3 -m compileall -q \
  tp_final_ws/src/tp_platform \
  tp_final_ws/src/tp_a_slam_aruco \
  tp_final_ws/src/tp_b_navigation \
  tp_final_ws/src/tp_c_mission
git diff --check
```

ROS build and tests:

```bash
cd tp_final_ws
PYTHON_EXECUTABLE="$(command -v python3)"
colcon build --packages-select \
  tp_platform tp_interfaces tp_a_slam_aruco tp_b_navigation \
  tp_c_mission turtlebot3_custom_simulation \
  --cmake-args \
  -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}" \
  -DPYTHON_EXECUTABLE="${PYTHON_EXECUTABLE}"
source install/setup.bash
python3 -m pytest \
  src/tp_platform/test \
  src/tp_a_slam_aruco/test \
  src/tp_b_navigation/test \
  src/tp_c_mission/test -q
```

## Task 1: Route namespaced TF into Parte B and Parte C

**Branch:** `fix/real-tb4-tf-routing`

**Files:**

- Modify: `tp_final_ws/src/tp_b_navigation/launch/parte_b.launch.py`
- Modify: `tp_final_ws/src/tp_c_mission/launch/parte_c_real.launch.py`
- Modify: `tp_final_ws/src/tp_platform/tp_platform/platform_profiles.py`
- Test: `tp_final_ws/src/tp_b_navigation/test/test_platform_profiles.py`
- Test: `tp_final_ws/src/tp_b_navigation/test/test_simulation_contracts.py`
- Test: `tp_final_ws/src/tp_c_mission/test/test_parte_c_contracts.py`

- [ ] **Step 1: Add failing launch-contract tests**

Assert that TB4 profiles route both TF topics while simulation keeps the
standard global topics:

```python
def test_real_profile_exposes_namespaced_tf_topics():
    profile = resolve_profile('real_tb4', robot_namespace='tb4_1')
    assert profile.tf_topic == '/tb4_1/tf'
    assert profile.tf_static_topic == '/tb4_1/tf_static'


def test_simulation_profile_keeps_global_tf_topics():
    profile = resolve_profile('simulation_tb3')
    assert profile.tf_topic == '/tf'
    assert profile.tf_static_topic == '/tf_static'
```

Add launch-source assertions requiring `('/tf', tf_topic)` and
`('/tf_static', tf_static_topic)` in the real launch paths.

- [ ] **Step 2: Verify the new tests fail**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_b_navigation/test/test_platform_profiles.py \
  tp_final_ws/src/tp_b_navigation/test/test_simulation_contracts.py \
  tp_final_ws/src/tp_c_mission/test/test_parte_c_contracts.py -q
```

Expected: failures because simulation currently has empty TF topics and B/C do
not route namespaced TF.

- [ ] **Step 3: Make the platform TF contract explicit**

Set simulation defaults to `/tf` and `/tf_static`. Resolve the selected TB4
namespace exactly once for `bag_tb4` and `real_tb4`.

- [ ] **Step 4: Route TF for every node containing a TransformListener**

Extend the launch remaps:

```python
tf_remaps = [
    ('/tf', tf_topic),
    ('/tf_static', tf_static_topic),
]
sensor_remaps = [
    ('/odom', odom_topic),
    ('/scan', scan_topic),
    ('/cmd_vel', cmd_vel_topic),
    *tf_remaps,
]
```

Apply TF remaps to MCL, planner, obstacle monitor, state machine, ArUco adapter,
cone detector and any RViz instance that needs the selected robot TF. Do not
launch a second driver stack.

- [ ] **Step 5: Test both namespaces and simulation**

```bash
ros2 launch tp_b_navigation parte_b.launch.py --show-args
ros2 launch tp_c_mission parte_c_real.launch.py --show-args
python3 -m pytest tp_final_ws/src/tp_b_navigation/test \
  tp_final_ws/src/tp_c_mission/test -q
```

Manually inspect expanded launch logging for `tb4_0` and `tb4_1`. Confirm
`simulation_tb3` still launches both virtual-landmark nodes.

- [ ] **Step 6: Commit**

```bash
git add tp_final_ws/src/tp_platform \
  tp_final_ws/src/tp_b_navigation \
  tp_final_ws/src/tp_c_mission
git commit -m "fix: route namespaced TB4 transforms"
```

## Task 2: Publish MCL prediction continuously

**Branch:** `fix/mcl-prediction-publication`

**Files:**

- Modify: `tp_final_ws/src/tp_b_navigation/tp_b_navigation/mcl_localization.py`
- Modify: `tp_final_ws/src/tp_b_navigation/tp_b_navigation/safety_gates.py`
- Test: `tp_final_ws/src/tp_b_navigation/test/test_mcl_prediction.py`
- Test: `tp_final_ws/src/tp_b_navigation/test/test_safety_gates.py`

- [ ] **Step 1: Extract a testable prediction-publication decision**

Add tests proving that a valid odometry prediction:

- changes the weighted estimate;
- produces a new MCL pose timestamp;
- allows `map -> odom` to remain approximately stable during pure odometry;
- does not require an ArUco observation for every update.

Use deterministic particles and zero motion noise:

```python
particles = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
weights = np.array([0.5, 0.5])
predicted = predict_particles(
    particles, weights,
    previous_odom=(0.0, 0.0, 0.0),
    current_odom=(0.2, 0.0, 0.0),
    alphas=(0.0, 0.0, 0.0, 0.0),
    rng=np.random.default_rng(7),
)
assert np.allclose(predicted[:, 0], [0.2, 0.3])
```

- [ ] **Step 2: Verify the prediction tests fail**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_b_navigation/test/test_mcl_prediction.py -q
```

- [ ] **Step 3: Separate prediction from ROS callbacks**

Create a small pure helper for particle propagation or adapt the existing
method so it can be tested deterministically. Keep the current Thrun odometry
model and parameters unchanged.

- [ ] **Step 4: Refresh the estimate and outputs after prediction**

After a successful `_predict`:

```python
self._update_estimate()
self.publish_particles()
self.publish_pose()
```

Do not resample during prediction. Corrections and resampling remain driven by
landmark measurements.

- [ ] **Step 5: Do not consume accumulated motion on an empty correction**

Make `_correct_measurements()` return the number of observations actually used:

```python
used = self._correct_measurements(measurements)
if used:
    self.accum_d = 0.0
    self.accum_a = 0.0
```

Add tests for empty, unknown-ID and valid-ID observation arrays.

- [ ] **Step 6: Clarify stationary correction behavior**

The real B1 gate must be able to validate ArUco consistency while stopped.
Choose one explicit behavior and test it:

- identified ArUco observations may correct after `/initialpose` even without
  motion; or
- B1 is documented as observation validation only and does not claim a filter
  correction while stationary.

Recommended: permit a first identified correction after `/initialpose`, then
retain the movement threshold for repeated static measurements.

- [ ] **Step 7: Run regression tests**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_b_navigation/test/test_mcl_prediction.py \
  tp_final_ws/src/tp_b_navigation/test/test_safety_gates.py \
  tp_final_ws/src/tp_b_navigation/test/test_simulation_contracts.py -q
```

Confirm simulation still uses `/calc_odom` for prediction and `/odom` for its
virtual landmark truth.

- [ ] **Step 8: Commit**

```bash
git add tp_final_ws/src/tp_b_navigation
git commit -m "fix: publish MCL pose after odometry prediction"
```

## Task 3: Require bracketing odometry for Parte A scans

**Branch:** `fix/parte-a-scan-odom-bracketing`

**Files:**

- Modify: `tp_final_ws/src/tp_a_slam_aruco/tp_a_slam_aruco/occupancy_grid_node.py`
- Create: `tp_final_ws/src/tp_a_slam_aruco/tp_a_slam_aruco/scan_odom_buffer.py`
- Test: `tp_final_ws/src/tp_a_slam_aruco/test/test_scan_odom_buffer.py`
- Modify: `tp_final_ws/src/tp_a_slam_aruco/test/test_parte_a_contracts.py`

- [ ] **Step 1: Write failing buffer tests**

Cover:

```python
def test_scan_waits_until_odom_brackets_timestamp():
    buffer = ScanOdomBuffer()
    buffer.add_odom(10.00, (0.0, 0.0, 0.0))
    buffer.add_scan(FakeScan(stamp=10.03))
    assert buffer.pop_ready() == []
    buffer.add_odom(10.05, (0.1, 0.0, 0.0))
    ready = buffer.pop_ready()
    assert len(ready) == 1


def test_interpolation_does_not_clamp_to_latest_odom():
    assert interpolate_bracketed(
        [(10.00, (0.0, 0.0, 0.0))], 10.03
    ) is None
```

Also test out-of-order messages, duplicate odometry timestamps, bounded memory
and end-of-bag flushing policy.

- [ ] **Step 2: Verify the tests fail**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_a_slam_aruco/test/test_scan_odom_buffer.py -q
```

- [ ] **Step 3: Implement bounded scan/odom synchronization**

Queue scans until odometry samples exist on both sides. Resolve the base pose
with dense odometry plus interpolated SLAM correction only after the bracket is
available. Never silently clamp a scan newer than the last odometry sample.

Expose counters:

- scans integrated with a valid bracket;
- scans waiting;
- scans dropped at end-of-run;
- scans dropped for excessive wait;
- interpolation gap in milliseconds.

- [ ] **Step 4: Preserve the LIDAR TF/fallback contract**

After resolving the bracketed base pose, continue resolving
`scan.header.frame_id -> base_frame` at the scan timestamp. Keep
`lidar_tx=-0.04`, `lidar_yaw=pi/2` only as the explicit fallback.

- [ ] **Step 5: Compare the same bag deterministically**

Generate two maps from `tp_final_ws/bags/laberinto`:

```bash
ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
  robot_namespace:=tb4_0 \
  trajectory_file:=/ruta/trajectory.json \
  map_output:=/tmp/tb4-map-bracketed \
  max_angular_velocity:=0.0

ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
  robot_namespace:=tb4_0 \
  trajectory_file:=/ruta/trajectory.json \
  map_output:=/tmp/tb4-map-low-turn-rate \
  max_angular_velocity:=0.35
```

Record wall thickness, openings, double-wall count, scan-drop percentage and
fine-object retention. Do not change log-odds in this branch.

- [ ] **Step 6: Commit code and diagnostic evidence separately**

```bash
git add tp_final_ws/src/tp_a_slam_aruco
git commit -m "fix: bracket Parte A scans with dense odometry"

git add docs/parte_a
git commit -m "docs: compare bracketed Parte A maps"
```

## Task 4: Enforce measurement timestamps and the real RGB-D contract

**Branch:** `fix/real-sensor-time-depth-contract`

**Files:**

- Modify: `tp_final_ws/src/tp_b_navigation/tp_b_navigation/aruco_mcl_adapter.py`
- Modify: `tp_final_ws/src/tp_c_mission/tp_c_mission/cone_detector_node.py`
- Modify: `tp_final_ws/src/tp_c_mission/tp_c_mission/mission_manager_node.py`
- Modify: `tp_final_ws/src/tp_c_mission/config/parte_c.yaml`
- Test: `tp_final_ws/src/tp_b_navigation/test/test_aruco_mcl_adapter.py`
- Test: `tp_final_ws/src/tp_c_mission/test/test_cone_runtime_contract.py`

- [ ] **Step 1: Add timestamp tests**

Mock the TF buffer and assert that lookups receive the marker/image timestamp,
not `rclpy.time.Time()`:

```python
stamp = Time(sec=12, nanosec=300_000_000)
adapter.transform_marker(marker_with_stamp(stamp))
assert tf_buffer.requested_time == rclpy.time.Time.from_msg(stamp)
```

- [ ] **Step 2: Add RGB-D readiness tests**

Real vision readiness requires:

- fresh RGB;
- valid `CameraInfo`;
- fresh depth;
- matching image dimensions;
- depth timestamp within `max_depth_age`;
- TF available at the RGB timestamp.

Test every missing condition independently.

- [ ] **Step 3: Use measurement timestamps in both adapters**

Convert message stamps using:

```python
measurement_time = rclpy.time.Time.from_msg(msg.header.stamp)
transform = self.tf_buffer.lookup_transform(
    target_frame,
    msg.header.frame_id,
    measurement_time,
)
```

Keep latest-TF behavior only behind an explicit diagnostic fallback parameter,
disabled by default in `real_tb4`.

- [ ] **Step 4: Split monocular and real RGB-D policies**

Add a parameter such as:

```yaml
require_aligned_depth: true
```

Set it to `true` in `parte_c_real.launch.py` and `false` only for simulation or
explicit bag calibration. When required depth becomes stale, publish
`vision_ready=false`; mission manager then cancels through the existing safety
path.

- [ ] **Step 5: Run tests and commit**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_b_navigation/test/test_aruco_mcl_adapter.py \
  tp_final_ws/src/tp_c_mission/test/test_cone_runtime_contract.py \
  tp_final_ws/src/tp_c_mission/test/test_parte_c_contracts.py -q

git add tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git commit -m "fix: enforce timestamped RGB-D observations"
```

## Task 5: Preserve stage artifacts and make RViz namespace-safe

**Branch:** `fix/run-artifacts-and-rviz`

**Files:**

- Modify: `tp_final_ws/src/tp_platform/tp_platform/platform_profiles.py`
- Modify: `tp_final_ws/src/tp_a_slam_aruco/launch/parte_a_slam.launch.py`
- Modify: `tp_final_ws/src/tp_a_slam_aruco/launch/parte_a_mapa.launch.py`
- Modify: `tp_final_ws/src/tp_b_navigation/launch/parte_b.launch.py`
- Modify: `tp_final_ws/src/tp_c_mission/launch/parte_c_real.launch.py`
- Modify: `tp_final_ws/src/tp_a_slam_aruco/config/rviz_config.rviz`
- Modify: `tp_final_ws/src/tp_b_navigation/config/parte_b.rviz`
- Test: `tp_final_ws/src/tp_platform/test/test_tp_platform_profiles.py`

- [ ] **Step 1: Add artifact non-overwrite tests**

Resolve stage-specific files:

```text
config/platform-parte-a-slam.yaml
config/platform-parte-a-mapa.yaml
config/platform-parte-b.yaml
config/platform-parte-c.yaml
```

Test that writing B does not change the bytes of the A files and that every
file includes `run_id`, profile, namespace, topics, frames and artifact paths.

- [ ] **Step 2: Extend the writer API**

Use an explicit stage:

```python
write_resolved_platform(
    artifact_dir,
    profile,
    stage='parte-b',
    run_id=run_id,
    topics=topics,
    frames=frames,
    artifacts=artifacts,
)
```

Reject empty or path-containing stage names. Do not merge YAML by parsing a
possibly half-written previous file.

- [ ] **Step 3: Stop defaulting Parte A outputs into package share**

Use `/tmp/tp_final_rob/...` defaults for trajectory and map output. The runbook
continues passing explicit run paths.

- [ ] **Step 4: Remap RViz topics**

Prefer launch remaps for `/scan`, `/odom`, `/tf` and `/tf_static` instead of
maintaining one RViz file per robot. Verify `tb4_1` displays its scan and
odometry without editing the RViz configuration.

- [ ] **Step 5: Test and commit**

```bash
python3 -m pytest \
  tp_final_ws/src/tp_platform/test \
  tp_final_ws/src/tp_b_navigation/test/test_simulation_contracts.py \
  tp_final_ws/src/tp_a_slam_aruco/test/test_aruco_launch_config.py -q

git add tp_final_ws/src/tp_platform \
  tp_final_ws/src/tp_a_slam_aruco \
  tp_final_ws/src/tp_b_navigation \
  tp_final_ws/src/tp_c_mission
git commit -m "fix: preserve per-stage run artifacts"
```

## Task 6: Add executable runtime smoke coverage

**Branch:** `test/tb4-runtime-smoke`

**Files:**

- Modify: `tp_final_ws/src/tp_a_slam_aruco/test/test_ros_smoke.py`
- Create: `tp_final_ws/src/tp_b_navigation/test/test_real_profile_smoke.py`
- Create: `tp_final_ws/src/tp_c_mission/test/test_real_mission_smoke.py`
- Modify: `README.md`

- [ ] **Step 1: Replace the nonexistent smoke-test bag path**

The current test references `bags/aruco_estimation`, which is absent. Accept a
bag path from `TP_TB4_TEST_BAG`; otherwise use `bags/laberinto` only when it
exists locally.

- [ ] **Step 2: Bound every smoke test**

Use `ros2 bag play --start-offset ... --duration ... --clock`, isolated
`ROS_DOMAIN_ID`, a temporary `ROS_LOG_DIR`, and process groups that always
receive SIGINT and then SIGTERM on timeout.

- [ ] **Step 3: Smoke-test Parte A contracts**

Assert:

- TF bridge receives namespaced TF;
- ArUco detector receives RGB and CameraInfo;
- occupancy node reports TF or explicit fallback source;
- output JSON/YAML files are non-empty;
- logs contain no traceback.

- [ ] **Step 4: Smoke-test B without moving hardware**

Run `profile:=bag_tb4`, disable velocity output through a test-only launch
switch or remap it to `/test/cmd_vel`, publish `/initialpose`, and assert:

- MCL pose continues updating from odometry;
- `map -> odom` exists;
- obstacle monitor reports healthy when TF/map/scan are valid;
- no virtual landmark nodes are present.

Run a separate simulation contract asserting the two virtual landmark nodes
are present in `simulation_tb3`.

- [ ] **Step 5: Smoke-test C readiness**

With a short cone-bag segment, assert that `vision_ready` reflects RGB-D and TF
freshness, and that removing depth causes readiness to become false without
publishing velocity.

- [ ] **Step 6: Run and commit**

```bash
RUN_ROS_SMOKE=1 \
TP_TB4_TEST_BAG="$(pwd)/tp_final_ws/bags/laberinto" \
python3 -m pytest \
  tp_final_ws/src/tp_a_slam_aruco/test/test_ros_smoke.py \
  tp_final_ws/src/tp_b_navigation/test/test_real_profile_smoke.py \
  tp_final_ws/src/tp_c_mission/test/test_real_mission_smoke.py -q

git add tp_final_ws/src/*/test README.md
git commit -m "test: add TB4 runtime smoke workflows"
```

## Task 7: Final diagnostics, runbook and laboratory gates

**Branch:** `docs/tb4-lab-readiness`

**Files:**

- Modify: `docs/2026-06-30-tb4-laboratory-runbook.md`
- Modify: `README.md`
- Create: `docs/parte_b/tb4-mcl-obstacle-diagnostic.md`
- Create: `docs/parte_a/tb4-map-comparison.md`

- [ ] **Step 1: Document clean build after package renames**

Add the targeted stale-artifact cleanup from the common environment section.
Do not recommend deleting bags or run artifacts.

- [ ] **Step 2: Correct the runbook commands**

Include:

- Miniforge activation on the notebook;
- explicit `run_id`;
- per-stage platform YAML files;
- TF preflight for the selected namespace;
- aligned-depth verification;
- `tb4_0`/`tb4_1` RViz checks;
- safe stop procedure before every autonomous gate.

- [ ] **Step 3: Execute the MCL/obstacle diagnostic before tuning**

Record in one reproducible run:

```text
/odom
/mcl_pose
/particlecloud
/observed_landmark_ids
/obstacle_detected
/obstacle_monitor_healthy
/dynamic_obstacles
/nav_state
/cmd_vel
/tf
/tf_static
```

Determine whether localization divergence or false obstacle insertion occurs
first. Do not change MCL noise, TTL, inflation and avoidance simultaneously.

- [ ] **Step 4: Complete gates A1-A3 on the known bag**

Preserve trajectory comparison, correction smoothness, TF/fallback counts,
scan-bracketing statistics, baseline map and bracketed map.

- [ ] **Step 5: Complete physical gates incrementally**

On one selected TB4:

1. B1 localization while autonomous motion is disabled.
2. B2 one short clear goal at reduced speed.
3. B2 multiple goals.
4. B3 obstacle detection while stopped.
5. B3 replan without motion.
6. B3 reduced-speed avoidance and localization recovery.
7. C1 RGB-D perception while stopped.
8. C2 reachable approach-pose validation.
9. C3 full red-cone mission with distractors.
10. Cancel, stale-data and emergency-stop tests.

- [ ] **Step 6: Record outcomes and commit documentation**

```bash
git add README.md docs
git commit -m "docs: finalize verified TB4 laboratory workflow"
```

## Integrated definition of done

Do not call the real workflow complete until all conditions are evidenced:

- all portable and ROS tests pass from a clean build;
- runtime smoke tests use an existing bag and pass;
- simulation still launches virtual landmarks;
- real/bag profiles never launch virtual landmarks;
- B/C receive selected namespaced TF;
- MCL advances with odometry between ArUco observations;
- safety gates remain healthy during valid motion and stop on stale inputs;
- each Parte A scan is integrated only with bracketed dense odometry;
- the accepted map has no severe double walls or false connections;
- real Parte C refuses missing/stale/misaligned depth;
- stage configurations do not overwrite each other;
- `tb4_0` and `tb4_1` require only namespace selection or documented overrides;
- B1-B3 and C1-C3 are recorded on physical hardware;
- the final runbook matches the actual launch arguments.

## Handoff to the next chat

Start with Task 1 only. Use one worktree or branch per task, TDD for every
behavioral correction, and request review before merging each branch. If a bag
or laboratory observation contradicts this plan, update this document before
changing algorithm parameters.
