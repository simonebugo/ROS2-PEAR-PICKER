# Commands and demonstration guide

This guide targets ROS 2 Jazzy on Linux or inside the existing Docker
container. Run Bash blocks in Linux/container terminals and PowerShell blocks
on Windows. Use system Python (`/usr/bin/python3`) for ROS and OpenCV; activate
no YOLO environment. Close previous launches with Ctrl+C before switching
between the independent modes below.

## 1. Workspace setup

The current container layout is:

```text
/root/sr_ws10/
  src/rbt08pkg/
  src/rbt10pkg_mit/
  vision/
  README.md
  COMMANDS.md
```

In **every new Bash terminal**, set these paths and source ROS:

```bash
export WS=/root/sr_ws10
export REPO="$WS"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
```

If using a native Linux Git checkout, set `WS` to your colcon workspace and
`REPO` to the repository root (for example `$WS/src/ROS2-PEAR-PICKER`). The
two package directories must be under `$WS/src`; tests live at `$REPO/vision`.
Skip sourcing `install/setup.bash` until the first build has completed.

For a fresh machine with ROS Jazzy already installed, resolve the declared
dependencies from the workspace. Use `sudo` as a regular user; omit it in a
root container:

```bash
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
# Only if rosdep has never been initialized on this machine:
sudo rosdep init
rosdep update
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select rbt08pkg rbt10pkg_mit --symlink-install
source "$WS/install/setup.bash"
/usr/bin/python3 -c 'import rclpy, cv2, numpy, yaml; print("ROS/OpenCV runtime imports OK")'
```

The camera sensor requires Ogre2 rendering on the Gazebo server. `render_engine:=ogre`
selects a more compatible GUI renderer; it does not remove the sensor's
Ogre2/OpenGL requirement.

### Updating the current Windows-to-Docker setup

The personal helper is optional and is not distributed through Git. On the
owner's Windows checkout it copies maintained sources and this guide:

```powershell
cd D:\Matteo\Desktop\sr_ws10
powershell -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Then rebuild both packages in the container with the build commands above.
The helper merges files and does not delete obsolete files already in the
container or its install tree. Use the maintained entry points below; a fresh
checkout/workspace will contain only maintained files. Do not copy a Windows
virtual environment into Linux. Collaborators can update their Linux checkout
through Git instead of using the personal helper.

## 2. Clean start and repeat a run

1. Stop the active launch with Ctrl+C and wait for its processes to exit.
2. Close separate RViz and diagnostic processes from the previous run.
3. Source the environment in each terminal and start exactly one mode below.
4. Wait for controllers and perception to become ready before triggering.

For a new pear or another seed, restart the full simulation. A trigger alone
does not put a released pear back on the branch or restore the robot's initial
pose. Do not launch `demo.launch.py` alongside the Gazebo integrated launch:
they would create competing controller/state sources.

## 3. Complete vision pick-and-place (recommended demo)

Terminal A — randomized **wide** scene with a reproducible seed:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py \
  pose_source:=vision randomize:=true seed:=42 \
  grasp_orientation:=branch \
  randomization_config:="$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/orchard_randomization_wide.json"
```

Change `42` to another integer for a different repeatable scene, or use
`seed:=auto` and record the seed printed by the launch. Always restart the
launch between seeds. The same seed under different profiles can produce
different scenes.

Terminal B — readiness and preflight **before grasping**:

```bash
ros2 control list_controllers
ros2 topic info /harvest/pear_pose
ros2 topic echo /orchard/vision/center_status --once
ros2 topic echo /harvest/pear_pose --once
```

Expect `arm_controller`, `gripper_controller` and `joint_state_broadcaster`
to be active, one publisher for `/harvest/pear_pose`, and a valid, recent
center. Copy the exact path from the new launch's `ORCHARD scenario=...` line:

```bash
# Replace the example path with the exact path printed by THIS launch.
export SCENARIO=/tmp/pear_orchard_42_REPLACE_ME/scenario.json
/usr/bin/python3 "$REPO/vision/check_randomized_runtime.py" "$SCENARIO"
```

Expect `SCENE ALIGNMENT OK`, `MOVEIT IK OK`, `VISION OK`, `GRASP ORIENTATION OK` and
`RANDOMIZED PREFLIGHT OK`. The alignment check expects the intentional 15 mm
radial branch clearance in MoveIt. This test does not command motion. If it
fails, keep its message and manifest and resolve the failure before triggering.
The IK check tries the preceding MoveIt solution and the manifest's numerical
seed. If collision-aware IK fails, diagnostic-only queries report whether pure
IK exists and list contacts from `/check_state_validity`. These queries never
authorize motion or count as a passed preflight. An error such as `-31` alone
does not establish whether collision rejection or IK search caused the failure.

Trigger exactly one cycle:

```bash
ros2 topic pub --once /harvest/trigger std_msgs/msg/Empty '{}'
```

Expect `PICK AND PLACE COMPLETED SUCCESSFULLY`, physical release into the
basket and upward retreat. Observe the full cycle in Gazebo: a passed preflight
is not a full-path or physical-grasp guarantee.

### Other scene profiles

Standard randomization (the default bounds):

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision randomize:=true seed:=42
```

Fixed edge case: height 1.56 m, yaw +20 deg, elevation +5 deg, distance 0.56 m:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py \
  pose_source:=vision randomize:=true seed:=42 \
  randomization_config:="$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/orchard_randomization_edge.json"
```

All edge intervals have equal endpoints: another seed keeps the same geometry.
Use its newly printed manifest for preflight. Returning to wide only requires
selecting the wide file at the next clean launch; no rebuild is needed.

Fixed original world with a vision-derived target:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision
```

Fixed original world with a known target, to isolate perception from motion:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=static
```

For the two fixed-world modes, no random manifest is generated. Use readiness
checks and the trigger, not the randomized preflight. Randomization deliberately
requires `pose_source:=vision`.

### Branch-based grasp orientation

The integrated launch defaults to `grasp_orientation:=branch`. It rotates the
pear's local frame by the sampled branch yaw and elevation (pitch, at most 5 degrees
in the supplied profiles). `/harvest/pear_pose` has a camera-estimated position
and a scenario-supplied orientation; the camera does not estimate that rotation.
The manager projects the robot-to-pear vector into the pear's local XY plane
and computes a variable offset with atan2(local_y, local_x). It composes
R_pear * Rz(offset) * Ry(90 degrees): gripper +Z lies in pear XY and faces
the fruit from the robot side. The approach can therefore tilt vertically.
It does not force the gripper perpendicular to the branch. The camera supplies
the target position; branch orientation is known simulation information.
The branch mode first retreats 25 cm and then lifts 10 cm along base +Z.
Radial mode retains its previous 20 cm lift.

Only the scenario's orientation is added to the vision-based pear pose; the
manifest's pear position is never published as the robot target. Inspect
`/harvest/pear_pose` and `/harvest/grasp_pose` to see the two quaternions.
The manager transforms the pear pose to `base_link` and uses gripper +Z
for both the approach and the first withdrawal. The lift follows withdrawal.

The generator screens IK with the same oriented targets. Some combinations
previously accepted in radial mode can fail with branch mode; the generator
rejects those samples and tries another one. Thus a seed can produce different
geometry between modes or software versions. Numerical IK is not a collision
or full-trajectory guarantee; run the live preflight before the first trigger.
New branch manifests carry `grasp_offset_policy: robot_position`; old manifests
with fixed-offset IK seeds are rejected. Restart the launch to regenerate them.

Offline regression tests (no running simulation needed):

```bash
cd "$WS"
/usr/bin/python3 "$REPO/vision/test_randomization.py" -v
/usr/bin/python3 "$REPO/vision/test_pick_target.py" -v
/usr/bin/python3 "$REPO/vision/test_motion_checks.py" -v
```

Inspect the mode and targets while the integrated launch is running:

```bash
ros2 param get /pick_place_manager grasp_orientation
ros2 topic echo /harvest/pear_pose --once
ros2 topic echo /harvest/grasp_pose --once
ros2 topic echo /harvest/pre_grasp_pose --once
```

In RViz add a `Pose` display on `/harvest/grasp_pose` and choose `Axes` to see
the gripper frame; +Z is its approach axis. The existing target markers also
show the approach direction. With different seeds, compare the grasp frame's
orientation with the pear frame. A change in measured position also changes
the offset; fruit yaw and pitch still come from the scenario. The former
`approach_yaw_world` and `nominal_grasp_yaw_world` manager parameters are removed.

To return to the previous approach, stop the launch and restart with:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py \
  pose_source:=vision randomize:=true seed:=42 grasp_orientation:=radial \
  randomization_config:="$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/orchard_randomization_wide.json"
```

No rebuild is needed when switching modes. A standalone `pick_place_manager.py`
still defaults to radial mode for the separate MoveIt/RViz demonstration.

## 4. Visualization during the integrated demo

Additional terminal — RGB, depth, masks, TF and 3D perception markers:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/camera.rviz" --ros-args -p use_sim_time:=true
```

Additional terminal — MoveIt robot, obstacles and planned trajectories:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix rbt10pkg_mit)/share/rbt10pkg_mit/config/moveit.rviz" --ros-args -p use_sim_time:=true
```

Use `world` as Fixed Frame. The depth panel uses an unnormalized 0.1–5 m
range. Enable these displays as needed:

| Display | Topic / observation |
| --- | --- |
| Image: RGB | `/orchard/camera/rgb/image_raw` |
| Image: Depth | `/orchard/camera/depth/image_raw`, encoding `32FC1` |
| Image: HSV overlay | `/orchard/vision/image_annotated` |
| Image: all segmented pixels | `/orchard/vision/mask` |
| Image: selected target | `/orchard/vision/target_mask` |
| Marker: observed surface | `/orchard/vision/surface_marker`, magenta |
| Marker: world surface | `/orchard/vision/surface_marker_world`, cyan, should overlap the magenta point |
| Marker: estimated center | `/orchard/vision/center_marker_world`, orange, behind the observed surface |
| MarkerArray: task targets | `/harvest/target_markers`; arrows illustrate goals, not the executed path |
| Planning scene | `/monitored_planning_scene`; enable scene geometry and robot collision geometry in MoveIt |

The integrated launch starts perception but does not start the camera transport
checker. To add its optical-axis ray and `/orchard/camera/status`, run:

```bash
ros2 run rbt08pkg camera_pipeline_check.py --ros-args -p use_sim_time:=true
```

Do not start a second checker if using the camera-only or standalone vision
launches below: those already start it.

## 5. Independent teaching modes

Start each mode with the other launches stopped.

Camera only: Gazebo, stable robot/controllers, RGB-D bridge, checker and RViz;
no task execution:

```bash
ros2 launch rbt08pkg camera_test.launch.py render_engine:=ogre
```

Vision only: camera plus segmentation, surface/center estimates and RViz;
no pick-and-place manager and no `/harvest/pear_pose` publisher by default:

```bash
ros2 launch rbt08pkg vision_test.launch.py render_engine:=ogre
```

For only 2D segmentation, add `estimate_3d:=false transform_world:=false`.
For camera-frame 3D only, add `transform_world:=false`. To hide RViz, add
`rviz:=false`. When a camera is already running and no detector is active:

```bash
ros2 launch rbt08pkg vision_test.launch.py start_camera:=false
```

Robot inspection with joint sliders, no Gazebo or MoveIt:

```bash
ros2 launch rbt08pkg rviz.launch.py
```

MoveIt-only demonstration with fake hardware — run each line in a separate
sourced terminal, in this order:

```bash
ros2 launch rbt10pkg_mit demo.launch.py
```
```bash
ros2 run rbt08pkg spawn_planning_scene.py
```
```bash
ros2 run rbt08pkg pear_pose_source.py
```
```bash
ros2 run rbt08pkg pick_place_manager.py
```
```bash
ros2 topic pub --once /harvest/trigger std_msgs/msg/Empty '{}'
```

The fake-hardware demo animates the robot and MoveIt attachment but cannot
simulate a falling pear. Restart these processes for a repeat from the initial
state. Do not enable simulation time in this mode: there is no Gazebo clock.

Gazebo/MoveIt motion smoke test — from a clean start:

```bash
ros2 launch rbt10pkg_mit gazebo_moveit_test.launch.py
```

After controllers are active, in another terminal (this **moves the robot**):

```bash
ros2 run rbt08pkg moveit_smoke_test.py --ros-args -p use_sim_time:=true
```

## 6. Tests without physical motion

Offline tests, with system OpenCV and NumPy installed:

```bash
/usr/bin/python3 -m unittest discover -s "$REPO/vision" -p 'test_*.py' -v
```

ROS runtime preflights use isolated topics and synthetic input. Source ROS
and the built workspace first; Gazebo is not required:

```bash
/usr/bin/python3 "$REPO/vision/check_runtime.py"
/usr/bin/python3 "$REPO/vision/check_depth_runtime.py"
/usr/bin/python3 "$REPO/vision/check_world_runtime.py"
/usr/bin/python3 "$REPO/vision/check_pick_target_runtime.py"
```

They cover segmentation/headers, invalid inputs, depth deprojection, TF and
clock reset, center estimation, target freshness and the trigger snapshot.
The detector check deliberately sends a malformed image; an error log during
that check is expected. Each script must finish with its `PREFLIGHT OK` message.
The randomized live-scene preflight is described in section 3.

## 7. Live diagnostics and frame capture

Run continuous `echo`, `hz` or TF commands separately; Ctrl+C stops each one.

```bash
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 action list
ros2 run tf2_ros tf2_echo world table_camera_link_optical
```

Camera input and frequency:

```bash
ros2 topic echo /orchard/camera/camera_info --once --qos-reliability best_effort
ros2 topic echo /orchard/camera/rgb/image_raw --field encoding --once --qos-reliability best_effort
ros2 topic echo /orchard/camera/depth/image_raw --field encoding --once --qos-reliability best_effort
ros2 topic hz /orchard/camera/rgb/image_raw
ros2 topic hz /orchard/camera/depth/image_raw
ros2 topic hz /orchard/vision/image_annotated
```

Do not add `--qos-reliability` to `topic hz`: the installed Jazzy CLI used for
this project rejects it. A low wall-clock rate can reflect slow rendering or
simulation; the detector also limits its processing rate to 5 Hz.

Perception and grasp state:

```bash
ros2 topic echo /orchard/vision/detections --once --qos-reliability best_effort
ros2 topic echo /orchard/vision/status --once
ros2 topic echo /orchard/vision/depth_status --once
ros2 topic echo /orchard/vision/world_status --once
ros2 topic echo /orchard/vision/center_status --once
ros2 topic echo /orchard/vision/surface_point --once
ros2 topic echo /orchard/vision/surface_point_world --once
ros2 topic echo /orchard/vision/center_point_world --once
ros2 topic echo /harvest/state
```

Capture one RGB image and save its masks, overlay and JSON report:

```bash
/usr/bin/python3 "$REPO/vision/inspect_segmentation.py" \
  --config "$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/pear_segmentation.yaml"
```

The tool prints its timestamped output directory under `vision/runs/hsv_debug`.
It loads the YAML settings, not any temporary changes made to the live node.
To reprocess a saved image:

```bash
/usr/bin/python3 "$REPO/vision/inspect_segmentation.py" \
  --image /absolute/path/to/rgb.png \
  --config "$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/pear_segmentation.yaml"
```

Inspect or temporarily tune segmentation (restart to restore YAML values):

```bash
ros2 param get /pear_detector hsv_lower
ros2 param set /pear_detector hsv_lower '[18, 80, 35]'
```

## 8. Common checks when a run stops

| Symptom | Check |
| --- | --- |
| No camera image | `gz topic -l`, `ros2 node info /camera_bridge`, sensor/OpenGL errors |
| Empty detections | RGB view, HSV masks, fruit visibility and thresholds |
| `WAITING_TF` | `tf2_echo`, frame IDs, clock and original measurement timestamp |
| `STREAM_TIMEOUT` | New input arrival and whether Gazebo is paused; watchdogs use wall time |
| Trigger rejected | Target freshness, finite coordinates and matching valid `center_status` |
| Incomplete Cartesian path / IK jump | Inspect tree, fruit and robot collision geometry; keep the failing seed and log |
| Start-state mismatch | Check `/joint_states` freshness and physical drift; do not bypass it by widening tolerances |
| Grasp/release timeout | Inspect `/harvest/state`, contacts and detachable-joint bridge messages |

If the world was accidentally paused, the integrated physical-grasp bridge
exposes this unpause service:

```bash
ros2 service call /world/pear_orchard/control ros_gz_interfaces/srv/ControlWorld \
  '{world_control: {pause: false}}'
```

If execution times out, a controller may still be finishing its goal. Stop
and restart the simulation before a new demonstration cycle. Record the
profile, seed, manifest, center error and failing step when reporting a problem.
