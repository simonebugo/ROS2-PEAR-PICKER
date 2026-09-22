# ROS2 Pear Picker

ROS2 Pear Picker is a simulated fruit-harvesting project built with ROS 2 Jazzy, Gazebo and MoveIt 2. A fixed robotic arm finds a pear on a tree, picks it and places it in a basket. The project is intended as a demonstrator for perception, motion planning and simulated grasping.

The current vision system uses a fixed RGB-D camera and OpenCV color segmentation. It does not require YOLO or a trained model.

> **Image placeholder:** overview of the robot, tree, pear, camera and basket in Gazebo.

## How it works

1. The camera provides a color image and depth measurements. The vision nodes identify the pear, estimate its 3D position and publish it in the `world` frame.
2. When `/harvest/trigger` is sent, the pick-and-place manager checks that a recent, valid target is available.
3. MoveIt plans the approach and the subsequent motions. Gazebo controllers execute them on the simulated robot.
4. The grasp manager detaches the pear from the branch and attaches it to the gripper. After moving to the basket, the gripper releases it.

Gazebo handles the physical attachment and release; MoveIt's Planning Scene represents the same objects for collision-aware planning. The camera-derived target is passed through `/harvest/pear_pose`, so the motion sequence does not depend on a hard-coded pear position.

> **Image placeholder:** RGB image with pear detection and the estimated target shown in RViz.

## Main components

| Location | Purpose |
| --- | --- |
| `rbt08pkg/` | Robot and world models, camera pipeline, grasp and pick-and-place nodes |
| `rbt10pkg_mit/` | MoveIt configuration and launch files |
| `vision/` | Vision checks and test utilities |

The project provides separate launches for camera/vision inspection, a MoveIt-only demonstration and the integrated Gazebo pick-and-place simulation. RViz configurations are retained to inspect the robot, RGB/depth images, detected pear and target markers.

## Reproducible scenarios

The pear and its supporting branch can be placed in different positions when the world starts. Randomization is bounded to keep the scenarios useful for the robot and camera, and a seed reproduces a chosen initial scene. Standard, wider and fixed edge-case profiles are available. The robot, basket and camera remain fixed.

> **Image placeholder:** examples of the same scene with different randomization seeds, followed by a grasp and basket release.

## Running the project

See [COMMANDS.md](COMMANDS.md) for setup, clean starts, demonstrations, tests, RViz/depth displays and troubleshooting commands. Run the documented modes separately to avoid duplicate robot publishers or controllers.
