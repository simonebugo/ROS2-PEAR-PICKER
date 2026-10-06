#!/usr/bin/env python3

import time
import threading
import math
import json
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Empty
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK, GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, AllowedCollisionEntry
from trajectory_checks import cartesian_problem
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def multiply_quaternions(a, b):
    """Compose rotations: apply b in the pear frame, then a in the parent frame."""
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


class PickPlaceManager(Node):
    """
    Coordinate target poses, MoveIt trajectories and the physical grasp lifecycle.
    """

    # Wall-time allowance for slow Gazebo execution on a VM.
    EXECUTION_TIMEOUT_S = 120.0

    def __init__(self):
        super().__init__('pick_place_manager')
        self.declare_parameter('require_vision_status', False)
        self.declare_parameter('max_pose_age_s', 5.0)
        self.declare_parameter('grasp_orientation', 'radial')
        self.declare_parameter('approach_yaw_world', 0.0)
        self.declare_parameter('nominal_grasp_yaw_world', 0.0)
        self._grasp_orientation = self.get_parameter('grasp_orientation').value
        self._approach_yaw_world = float(self.get_parameter('approach_yaw_world').value)
        self._nominal_grasp_yaw_world = float(self.get_parameter('nominal_grasp_yaw_world').value)
        if (self._grasp_orientation not in ('branch', 'radial') or
                not all(math.isfinite(v) for v in
                        (self._approach_yaw_world, self._nominal_grasp_yaw_world))):
            raise ValueError('Invalid grasp orientation mode or world yaw.')
        self._require_vision_status = bool(self.get_parameter('require_vision_status').value)
        self._max_pose_age = float(self.get_parameter('max_pose_age_s').value)
        if not math.isfinite(self._max_pose_age) or self._max_pose_age <= 0:
            raise ValueError('max_pose_age_s must be positive and finite.')
        self.latest_poses = None
        self._pear_msg = None
        self._pear_received_at = None
        self._center_status = {}
        self._center_status_at = None
        if self._require_vision_status:
            self.sub_center_status = self.create_subscription(
                String, '/orchard/vision/center_status', self.center_status_callback, 10)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_pear_pose = self.create_subscription(
            PoseStamped,
            '/harvest/pear_pose',
            self.pear_pose_callback,
            10
        )

        self.sub_grasp_state = self.create_subscription(
            String,
            '/harvest/state',
            self.grasp_state_callback,
            10
        )

        self.sub_trigger = self.create_subscription(
            Empty,
            '/harvest/trigger',
            self.trigger_callback,
            10
        )

        # Target poses remain visible to RViz and external diagnostics.
        self.pub_pre_grasp = self.create_publisher(PoseStamped, '/harvest/pre_grasp_pose', 10)
        self.pub_grasp = self.create_publisher(PoseStamped, '/harvest/grasp_pose', 10)
        self.pub_lift = self.create_publisher(PoseStamped, '/harvest/lift_pose', 10)

        self.pub_pre_place = self.create_publisher(PoseStamped, '/harvest/pre_place_pose', 10)
        self.pub_place = self.create_publisher(PoseStamped, '/harvest/place_pose', 10)
        self.pub_retreat = self.create_publisher(PoseStamped, '/harvest/retreat_pose', 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/harvest/target_markers', 10)
        self.pub_planning_scene = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.pub_gripper_close_complete = self.create_publisher(
            Empty, '/harvest/gripper/close_complete', 10)

        self.current_grasp_state = 'UNKNOWN'
        self._grasp_state_condition = threading.Condition()
        self._sequence_lock = threading.Lock()
        self._joint_lock = threading.Lock()
        self._joint_state = None
        self._joint_received_at = 0.0
        self._joint_count = 0
        self._saved_acm = None
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile_sensor_data)

        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.015)
        self.declare_parameter('grasp_state_timeout_s', 8.0)
        # False keeps the standalone MoveIt/RViz demo usable. The integrated
        # Gazebo launch enables physical attachment confirmation.
        self.declare_parameter('require_physical_grasp', False)
        
        # Show basket targets even before a pear measurement arrives.
        self.create_timer(1.0, self.publish_basket_poses)

        self.get_logger().info('Pick-and-place manager initialized.')
        self.get_logger().info(
            f'Grasp orientation: {self._grasp_orientation}; '
            f'configured world yaw={math.degrees(self._approach_yaw_world):.2f} deg '
            '(used only in branch mode). Target position comes from /harvest/pear_pose.')


        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self._apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        self.get_logger().info('MoveIt 2 action and service clients initialized.')

    def _spin_future(self, future, timeout_sec=10.0):
        """Wait from the sequence thread while the main executor handles ROS."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def compute_ik_for_pose(self, target_pose: PoseStamped):
        """Return an indicative collision-unchecked IK solution, or None."""
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'grasp_frame'
        req.ik_request.pose_stamped = target_pose
        req.ik_request.avoid_collisions = False  # Diagnostic only; motion planning checks collisions.
        req.ik_request.timeout.sec = 5

        future = self._ik_client.call_async(req)
        self._spin_future(future, timeout_sec=8.0)
        res = future.result()

        if res and res.error_code.val == 1:  # 1 = SUCCESS
            self.get_logger().info("IK: kinematic solution found.")
            return res.solution.joint_state

        self.get_logger().error(
            f"IK: no kinematic solution found for "
            f"({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})"
        )
        return None

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in ('j1', 'j2', 'j3', 'j4', 'j5', 'j6')):
            return
        with self._joint_lock:
            self._joint_state = copy.deepcopy(msg)
            self._joint_received_at = time.monotonic()
            self._joint_count += 1

    def wait_for_stable_arm(self):
        """Require fresh, stationary joint samples before planning the next motion."""
        deadline = time.monotonic() + 8.0
        reference, since, first_count = None, None, 0
        while rclpy.ok() and time.monotonic() < deadline:
            with self._joint_lock:
                state = copy.deepcopy(self._joint_state)
                received, count = self._joint_received_at, self._joint_count
            now = time.monotonic()
            if state is None or now-received > 1.0:
                reference = None
            else:
                positions = dict(zip(state.name, state.position))
                q = [positions[f'j{i}'] for i in range(1, 7)]
                if reference is None or max(abs(a-b) for a, b in zip(q, reference)) > .002:
                    reference, since, first_count = q, now, count
                elif now-since >= .4 and count-first_count >= 3:
                    return state
            time.sleep(.02)
        self.get_logger().error('Arm is moving or /joint_states is stale: planning stopped.')
        return None

    def read_scene(self, components):
        if not self._get_scene_client.wait_for_service(timeout_sec=3.):
            return None
        req = GetPlanningScene.Request()
        req.components.components = components
        future = self._get_scene_client.call_async(req)
        if not self._spin_future(future, timeout_sec=5.):
            return None
        return future.result().scene if future.result() else None

    def apply_acm(self, matrix):
        if not self._apply_scene_client.wait_for_service(timeout_sec=3.):
            return False
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.allowed_collision_matrix = matrix
        future = self._apply_scene_client.call_async(req)
        return bool(self._spin_future(future, timeout_sec=5.) and future.result() and future.result().success)

    def allow_fruit_contact(self):
        scene = self.read_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        if scene is None:
            self.get_logger().error('Planning scene unavailable for pear-gripper contact.')
            return False
        matrix = scene.allowed_collision_matrix
        self._saved_acm = copy.deepcopy(matrix)
        fingers = ['left_finger_link', 'right_finger_link', 'gripper_base_link']
        for name in ['pear'] + fingers:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for entry in matrix.entry_values:
                    entry.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        pear_index = matrix.entry_names.index('pear')
        for name in fingers:
            i = matrix.entry_names.index(name)
            matrix.entry_values[pear_index].enabled[i] = True
            matrix.entry_values[i].enabled[pear_index] = True
        if not self.apply_acm(matrix):
            self.get_logger().error('Cannot allow pear-gripper contact; sequence stopped.')
            return False
        return True

    def wait_for_scene_attachment(self, attached):
        deadline = time.monotonic()+8.
        while rclpy.ok() and time.monotonic() < deadline:
            scene = self.read_scene(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            if scene is not None:
                objects = [obj for obj in scene.robot_state.attached_collision_objects if obj.object.id == 'pear']
                if (attached and any(obj.link_name == 'grasp_frame' for obj in objects)) or (not attached and not objects):
                    return True
            time.sleep(.1)
        self.get_logger().error('MoveIt did not confirm the pear attachment update.')
        return False


    def plan_and_move_to_pose(self, target_pose: PoseStamped) -> bool:
        """
        Plan and execute a collision-checked free-space motion to the target pose.
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server /move_action unavailable!")
            return False

        self.compute_ik_for_pose(target_pose)

        state = self.wait_for_stable_arm()
        if state is None:
            return False

        self.get_logger().info("Sending pose goal to MoveGroup for planning...")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.4
        goal_msg.request.max_acceleration_scaling_factor = 0.4
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.start_state.joint_state = state

        # Pose constraints let OMPL choose among collision-free IK configurations.
        constraints = Constraints()
        constraints.name = "pose_target"

        # Position tolerance: a sphere of radius 3 cm.
        pos_constraint = PositionConstraint()
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.header.frame_id = target_pose.header.frame_id
        pos_constraint.link_name = 'grasp_frame'

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]
        bv.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        sphere_pose.orientation.w = 1.0
        bv.primitive_poses.append(sphere_pose)

        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        # Orientation tolerance: 0.20 rad on each axis.
        orient_constraint = OrientationConstraint()
        orient_constraint.header.stamp = self.get_clock().now().to_msg()
        orient_constraint.header.frame_id = target_pose.header.frame_id
        orient_constraint.link_name = 'grasp_frame'
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.20
        orient_constraint.absolute_y_axis_tolerance = 0.20
        orient_constraint.absolute_z_axis_tolerance = 0.20
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        # The pear remains a world CollisionObject during pre-grasp planning.
        # It is attached to grasp_frame only after GraspManager confirms
        # ON_GRIPPER in attach_pear_in_moveit().

        self.get_logger().info(
            f"Planning trajectory to ({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})..."
        )

        goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._spin_future(goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the planning request.")
            return False

        self.get_logger().info("Goal accepted; planning...")

        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, timeout_sec=20.0)
        result = result_future.result()

        if not result or result.result.error_code.val != 1:
            code = result.result.error_code.val if (result and hasattr(result.result, 'error_code')) else -1
            plan_time = getattr(result.result, 'planning_time', 0.0) if result else 0.0
            self.get_logger().error(f"Planning failed: MoveIt code {code} (time: {plan_time:.2f}s)")
            return False

        # Execute only after MoveIt accepts the planned trajectory.
        pts_count = len(result.result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(f"Planning succeeded ({pts_count} waypoints); executing...")

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.result.planned_trajectory

        exec_future = self._execute_trajectory_client.send_goal_async(exec_goal)
        self._spin_future(exec_future, timeout_sec=10.0)
        exec_handle = exec_future.result()

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error("ExecuteTrajectory rejected the trajectory.")
            return False

        exec_res_future = exec_handle.get_result_async()
        if not self._spin_future(exec_res_future, timeout_sec=self.EXECUTION_TIMEOUT_S):
            self.get_logger().error(
                f'MoveIt execution timed out after {self.EXECUTION_TIMEOUT_S:.0f} s; '
                'the controller may still complete the goal.')
            return False
        exec_res = exec_res_future.result()

        if exec_res and exec_res.result.error_code.val == 1:
            self.get_logger().info("Motion completed successfully.")
            return True
        else:
            code = exec_res.result.error_code.val if exec_res else -1
            self.get_logger().error(f"Controller trajectory execution failed: code {code}")
            return False

    def execute_cartesian_path(self, waypoints: list) -> bool:
        """
        Plan and execute a collision-checked Cartesian path through the waypoints.
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /compute_cartesian_path unavailable!")
            return False

        state = self.wait_for_stable_arm()
        if state is None:
            return False
        req = GetCartesianPath.Request()
        req.start_state.is_diff = True
        req.start_state.joint_state = state
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.link_name = 'grasp_frame'
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 5.0
        if hasattr(req, 'revolute_jump_threshold'):
            req.revolute_jump_threshold = 0.35
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4

        self.get_logger().info(f"Computing Cartesian trajectory through {len(waypoints)} waypoints...")
        future = self._cartesian_client.call_async(req)
        self._spin_future(future, timeout_sec=10.0)
        res = future.result()

        if res is None:
            self.get_logger().error('No response to Cartesian planning request.')
            return False
        problem = cartesian_problem(res, state)
        if problem:
            self.get_logger().error(problem)
            return False

        self.get_logger().info(f"Cartesian path complete ({res.fraction * 100:.1f}%); sending to /execute_trajectory...")

        if not self._execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/execute_trajectory unavailable!")
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        exec_future = self._execute_trajectory_client.send_goal_async(exec_goal)
        self._spin_future(exec_future, timeout_sec=10.0)
        exec_handle = exec_future.result()

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error("Controller rejected the Cartesian trajectory.")
            return False

        res_future = exec_handle.get_result_async()
        if not self._spin_future(res_future, timeout_sec=self.EXECUTION_TIMEOUT_S):
            self.get_logger().error(
                f'Cartesian execution timed out after {self.EXECUTION_TIMEOUT_S:.0f} s; '
                'the controller may still complete the goal.')
            return False
        exec_result = res_future.result()
        if exec_result and exec_result.result.error_code.val == 1:
            self.get_logger().info("Cartesian trajectory executed successfully.")
            return True

        error_code = exec_result.result.error_code.val if exec_result else -1
        self.get_logger().error(
            f"Controller Cartesian execution failed: code {error_code}")
        return False

    def grasp_state_callback(self, msg: String):
        state_text = msg.data.split(':')[0].strip()
        with self._grasp_state_condition:
            state_changed = state_text != self.current_grasp_state
            self.current_grasp_state = state_text

        if state_changed:
            self.get_logger().info(f'GraspManager state updated: {self.current_grasp_state}')

            if self.current_grasp_state == 'ON_GRIPPER':
                self.attach_pear_in_moveit()
            elif self.current_grasp_state == 'RELEASED':
                self.detach_pear_in_moveit()

        # Notify only after the corresponding Planning Scene update has been
        # published, so the sequence cannot start its next plan too early.
        with self._grasp_state_condition:
            self._grasp_state_condition.notify_all()

    def command_gripper(self, position: float, movement_time_s: float = 2.0) -> bool:
        """Command both fingers to the same joint position."""
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Action server /gripper_controller/follow_joint_trajectory unavailable!')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = int(movement_time_s)
        point.time_from_start.nanosec = int(
            (movement_time_s - int(movement_time_s)) * 1_000_000_000)
        goal.trajectory.points = [point]

        goal_future = self._gripper_client.send_goal_async(goal)
        self._spin_future(goal_future, timeout_sec=8.0)
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Controller rejected the gripper command.')
            return False

        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, timeout_sec=15.0)
        result = result_future.result()
        if (result is None or
                result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            error_code = result.result.error_code if result else -1
            self.get_logger().error(
                f'Gripper motion failed: code {error_code}')
            return False

        self.get_logger().info(
            f'Gripper reached position {position:.3f} m.')
        return True

    def wait_for_grasp_state(self, expected_state: str) -> bool:
        """Wait for physical grasp confirmation without blocking ROS callbacks."""
        timeout_s = float(self.get_parameter('grasp_state_timeout_s').value)
        deadline = time.monotonic() + timeout_s
        with self._grasp_state_condition:
            while self.current_grasp_state != expected_state:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.get_logger().error(
                        f'Timeout waiting for GraspManager state {expected_state}.')
                    return False
                self._grasp_state_condition.wait(timeout=remaining)
        return True

    def attach_pear_in_moveit(self):
        """Attach the existing pear collision object to grasp_frame in MoveIt."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        # An AttachedCollisionObject changes RobotState, not only the world.
        # MoveIt ignores the update unless this nested diff flag is set too.
        scene_msg.robot_state.is_diff = True

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = 'grasp_frame'
        attached_obj.object.id = 'pear'
        attached_obj.object.operation = CollisionObject.ADD
        attached_obj.touch_links = ['left_finger_link', 'right_finger_link', 'gripper_base_link']

        scene_msg.robot_state.attached_collision_objects.append(attached_obj)
        self.pub_planning_scene.publish(scene_msg)
        self.get_logger().info('MoveIt attachment update published: pear -> grasp_frame.')

    def detach_pear_in_moveit(self):
        """Detach the pear collision object from grasp_frame in MoveIt."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = 'grasp_frame'
        attached_obj.object.id = 'pear'
        attached_obj.object.operation = CollisionObject.REMOVE

        scene_msg.robot_state.attached_collision_objects.append(attached_obj)
        self.pub_planning_scene.publish(scene_msg)
        self.get_logger().info('MoveIt detachment update published for the pear.')

    def add_pear_to_world(self, pose: Pose) -> None:
        """Restore the released pear as a static obstacle in the MoveIt-only demo."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        pear = CollisionObject()
        pear.header.frame_id = 'base_link'
        pear.id = 'pear'
        pear.operation = CollisionObject.ADD

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.075]
        pear.primitives.append(sphere)
        pear.primitive_poses.append(pose)
        scene_msg.world.collision_objects.append(pear)

        self.pub_planning_scene.publish(scene_msg)
        self.get_logger().info(
            'MoveIt world update published: pear at the placement pose.')

    def publish_basket_poses(self):
        """Publish basket target poses in base_link."""
        basket_world_x = 0.65
        basket_world_y = -0.55

        basket_yaw = math.atan2(basket_world_y, basket_world_x)
        basket_orientation = euler_to_quaternion(-math.pi, 0.0, basket_yaw)

        pre_place = PoseStamped()
        pre_place.header.stamp = self.get_clock().now().to_msg()
        pre_place.header.frame_id = 'base_link'
        pre_place.pose.position = Point(x=basket_world_x, y=basket_world_y, z=0.35)
        pre_place.pose.orientation = basket_orientation

        place = PoseStamped()
        place.header = pre_place.header
        place.pose.position = Point(x=basket_world_x, y=basket_world_y, z=0.18)
        place.pose.orientation = basket_orientation

        retreat = PoseStamped()
        retreat.header = pre_place.header
        retreat.pose.position = Point(x=basket_world_x, y=basket_world_y, z=0.40)
        retreat.pose.orientation = basket_orientation

        self.pub_pre_place.publish(pre_place)
        self.pub_place.publish(place)
        self.pub_retreat.publish(retreat)

    def pear_pose_callback(self, msg: PoseStamped):
        """Transform the pear pose to base_link and compute the cycle targets."""
        self.latest_poses = None
        self._pear_msg = msg
        self._pear_received_at = time.monotonic()
        if not all(math.isfinite(v) for v in (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)):
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pear_in_base = do_transform_pose_stamped(msg, transform)
        except Exception as e:
            self.get_logger().warn(f'TF2 error transforming the pear pose: {e}')
            return

        px = pear_in_base.pose.position.x
        py = pear_in_base.pose.position.y
        pz = pear_in_base.pose.position.z
        if not all(math.isfinite(v) for v in (px, py, pz)):
            return

        # Standalone/legacy mode: radial approach in the base XY plane.
        approach_yaw = math.atan2(py, px)

        # Grasp-frame +Z points horizontally toward the fruit; +Y keeps
        # the fingers side by side. The wrist stays behind the grasp frame.
        grasp_orientation = euler_to_quaternion(0.0, math.pi / 2.0, approach_yaw)
        if self._grasp_orientation == 'branch':
            # The camera supplies centre position; the known scenario sets
            # pear orientation in that PoseStamped. The fixed pear->gripper
            # rotation is the successful nominal grasp in the original world.
            pear_q = pear_in_base.pose.orientation
            norm = math.sqrt(sum(v*v for v in (pear_q.x, pear_q.y, pear_q.z, pear_q.w)))
            if not math.isfinite(norm) or abs(norm-1.0) > 1e-3:
                self.get_logger().warn('Invalid pear orientation; grasp target unavailable.')
                return
            pear_to_grasp = euler_to_quaternion(0.0, math.pi / 2.0, self._nominal_grasp_yaw_world)
            grasp_orientation = multiply_quaternions(pear_q, pear_to_grasp)

        # Local +Z is the approach axis. Pre-grasp and retreat must follow the
        # chosen attitude, including when world and base_link differ.
        q = grasp_orientation
        dir_x = 2.0 * (q.x*q.z + q.w*q.y)
        dir_y = 2.0 * (q.y*q.z - q.w*q.x)
        dir_z = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)

        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position = Point(x=px, y=py, z=pz)
        grasp_pose.pose.orientation = grasp_orientation

        # Approach horizontally from 25 cm behind the fruit.
        offset_pre = 0.25
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header = grasp_pose.header
        pre_grasp_pose.pose.position = Point(
            x=px - dir_x * offset_pre,
            y=py - dir_y * offset_pre,
            z=pz - dir_z * offset_pre
        )
        pre_grasp_pose.pose.orientation = grasp_orientation

        # The branch-oriented wrist has less vertical clearance from the fruit
        # branch. MoveIt rejected the old 20 cm endpoint (left finger versus
        # branch); its 10 cm intermediate target passed the live IK check.
        # Preserve the previous 20 cm lift in standalone/radial mode.
        offset_lift = 0.10 if self._grasp_orientation == 'branch' else 0.20
        lift_pose = PoseStamped()
        lift_pose.header = grasp_pose.header
        lift_pose.pose.position = Point(
            x=pre_grasp_pose.pose.position.x,
            y=pre_grasp_pose.pose.position.y,
            z=pre_grasp_pose.pose.position.z + offset_lift,
        )
        lift_pose.pose.orientation = grasp_orientation

        self.pub_pre_grasp.publish(pre_grasp_pose)
        self.pub_grasp.publish(grasp_pose)
        self.pub_lift.publish(lift_pose)

        basket_yaw = math.atan2(-0.55, 0.65)
        basket_orient = euler_to_quaternion(-math.pi, 0.0, basket_yaw)
        
        pre_place_p = Pose(position=Point(x=0.65, y=-0.55, z=0.35), orientation=basket_orient)
        place_p = Pose(position=Point(x=0.65, y=-0.55, z=0.18), orientation=basket_orient)
        retreat_p = Pose(position=Point(x=0.65, y=-0.55, z=0.40), orientation=basket_orient)

        self.publish_all_markers(
            pre_grasp_pose.pose, grasp_pose.pose, lift_pose.pose,
            pre_place_p, place_p, retreat_p
        )

        pre_place_pose = PoseStamped(header=grasp_pose.header, pose=pre_place_p)
        place_pose = PoseStamped(header=grasp_pose.header, pose=place_p)
        retreat_pose = PoseStamped(header=grasp_pose.header, pose=retreat_p)

        self.latest_poses = (pre_grasp_pose, grasp_pose, lift_pose, pre_place_pose, place_pose, retreat_pose)

    def center_status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
            self._center_status = status if isinstance(status, dict) else {}
        except (ValueError, TypeError):
            self._center_status = {}
        self._center_status_at = time.monotonic()

    def target_rejection_reason(self):
        """Four trigger checks: presence, finite coordinates, validity, freshness."""
        if self._pear_msg is None:
            return 'pear pose not available yet'
        p = self._pear_msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return 'nonfinite pear coordinates'
        if self.latest_poses is None:
            return 'pose unavailable in base_link (check TF)'
        stamp = self._pear_msg.header.stamp
        now = time.monotonic()
        ages = [now - self._pear_received_at,
                (self.get_clock().now().nanoseconds -
                 (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9]
        if self._require_vision_status:
            status = self._center_status
            if (status.get('valid') is not True or
                    status.get('stamp') != {'sec': stamp.sec, 'nanosec': stamp.nanosec}):
                return 'center_status is invalid for this measurement'
            try:
                ages.append(float(status['received_age_s']) + now - self._center_status_at)
            except (KeyError, TypeError, ValueError):
                return 'vision measurement age unavailable'
        if any(not math.isfinite(age) or age < 0 or age > self._max_pose_age for age in ages):
            return f'stale pose or inconsistent timestamp (limit {self._max_pose_age:.1f} s)'
        return None

    def trigger_callback(self, msg: Empty):
        """Validate the target and capture the cycle poses on a manual trigger."""
        reason = self.target_rejection_reason()
        if reason is None:
            self.get_logger().info('Manual trigger received; starting the sequence worker...')
            thread = threading.Thread(target=self.start_pick_and_place_sequence, args=self.latest_poses)
            thread.start()
        else:
            self.get_logger().warn(f'Trigger rejected: {reason}.')

    def start_pick_and_place_sequence(self, pre_g: PoseStamped, grasp: PoseStamped, lift: PoseStamped, pre_p: PoseStamped, place: PoseStamped, retreat: PoseStamped):
        if not self._sequence_lock.acquire(blocking=False):
            self.get_logger().warn('Sequence already running: trigger ignored.')
            return
        try:
            self._run_pick_and_place(pre_g, grasp, lift, pre_p, place, retreat)
        finally:
            try:
                if self._saved_acm is not None and not self.apply_acm(self._saved_acm):
                    self.get_logger().error('Failed to restore MoveIt contact permissions; restart before another cycle.')
                self._saved_acm = None
            finally:
                self._sequence_lock.release()

    def _run_pick_and_place(self, pre_g, grasp, lift, pre_p, place, retreat):
        """
        Execute the eight-step pick-and-place cycle.
        """
        self.get_logger().info("=== STARTING PICK AND PLACE ===")

        self.get_logger().info("[STEP 1/8] Moving to pre-grasp...")
        if not self.plan_and_move_to_pose(pre_g):
            self.get_logger().error("STEP 1 FAILED: cannot reach pre-grasp. Sequence stopped.")
            return

        # The final approach intentionally permits fingertip-to-pear contact.
        self.get_logger().info("[STEP 2/8] Cartesian approach to the pear...")
        if not self.allow_fruit_contact():
            return
        if not self.execute_cartesian_path([grasp.pose]):
            self.get_logger().error("STEP 2 FAILED: grasp approach failed. Sequence stopped.")
            return

        self.get_logger().info("[STEP 3/8] Closing gripper and awaiting GraspManager confirmation...")
        closed_position = float(self.get_parameter('gripper_closed_position').value)
        if not self.command_gripper(closed_position):
            self.get_logger().error("STEP 3 FAILED: gripper closure failed.")
            return
        # GraspManager uses this event after the controller has completed the
        # deliberate close command at the known grasp pose.  This also covers
        # Gazebo configurations where collision blocks the finger joints but
        # the ContactSensor stream is unavailable.
        self.pub_gripper_close_complete.publish(Empty())
        require_physical_grasp = bool(
            self.get_parameter('require_physical_grasp').value)
        if (require_physical_grasp and
                not self.wait_for_grasp_state('ON_GRIPPER')):
            self.get_logger().error(
                "STEP 4 FAILED: physical grasp was not confirmed.")
            return
        if not require_physical_grasp:
            # In the standalone MoveIt/RViz demo there is no Gazebo
            # GraspManager event to perform this transition.  Make the same
            # logical attachment before planning the retreat; otherwise the
            # gripper starts phase 5 in collision with the pear still present
            # as a world CollisionObject.
            self.attach_pear_in_moveit()
            time.sleep(0.2)

        # Move away from the branch before lifting. The first waypoint
        # retraces the safe horizontal approach; the second raises the pear in
        # the cleared area, away from the trunk and fruit branch.
        self.get_logger().info("[STEP 5/8] Retreating from the branch and lifting the pear...")
        if not self.wait_for_scene_attachment(True):
            return
        if not self.execute_cartesian_path([pre_g.pose, lift.pose]):
            self.get_logger().error("STEP 5 FAILED: branch retreat/lift failed. Sequence stopped.")
            return

        self.get_logger().info("[STEP 6/8] Transporting the pear to the basket...")
        if not self.plan_and_move_to_pose(pre_p):
            self.get_logger().error("STEP 6 FAILED: basket transport failed. Sequence stopped.")
            return

        self.get_logger().info("[STEP 7/8] Descending into the basket...")
        if not self.execute_cartesian_path([place.pose]):
            self.get_logger().error("STEP 7 FAILED: basket descent failed. Sequence stopped.")
            return

        self.get_logger().info("[STEP 8/8] Releasing the pear and retreating...")
        open_position = float(self.get_parameter('gripper_open_position').value)
        if not self.command_gripper(open_position):
            self.get_logger().error("STEP 8 FAILED: gripper opening failed.")
            return
        if (require_physical_grasp and
                not self.wait_for_grasp_state('RELEASED')):
            self.get_logger().error(
                "STEP 8 FAILED: pear release was not confirmed.")
            return
        if not require_physical_grasp:
            self.detach_pear_in_moveit()
            time.sleep(0.2)
        if not self.wait_for_scene_attachment(False):
            return
        if not self.execute_cartesian_path([retreat.pose]):
            self.get_logger().error("STEP 8 FAILED: retreat failed.")
            return
        if not require_physical_grasp:
            # Add it back only once the TCP has cleared the fruit; adding it
            # before the retreat would make the Cartesian path start inside
            # the freshly released collision object.
            self.add_pear_to_world(place.pose)

        self.get_logger().info("=== PICK AND PLACE COMPLETED SUCCESSFULLY ===")

    def publish_all_markers(self, pre_g, grasp, lift, pre_p, place, retreat):
        """Publish six RViz target arrows; these illustrate goals, not the planned joint path."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        def create_arrow_points_marker(marker_id, name, p_start, p_end, r, g, b):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'base_link'
            m.ns = name
            m.id = marker_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points = [p_start, p_end]
            m.scale.x = 0.02  # Shaft diameter.
            m.scale.y = 0.04  # Head diameter.
            m.scale.z = 0.06  # Head length.
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.95
            return m

        # Horizontal direction toward the pear.
        dx = grasp.position.x - pre_g.position.x
        dy = grasp.position.y - pre_g.position.y
        d_len = math.sqrt(dx * dx + dy * dy)
        ux = dx / d_len if d_len > 0 else 1.0
        uy = dy / d_len if d_len > 0 else 0.0

        # Pre-grasp: 15 cm arrow along the approach direction.
        p_pre_end = Point(x=pre_g.position.x + ux * 0.15, y=pre_g.position.y + uy * 0.15, z=pre_g.position.z)
        m_pre_g = create_arrow_points_marker(1, 'pre_grasp', pre_g.position, p_pre_end, 0.0, 0.5, 1.0)

        # Grasp: 10 cm arrow along the grasp axis.
        p_grasp_end = Point(x=grasp.position.x + ux * 0.10, y=grasp.position.y + uy * 0.10, z=grasp.position.z)
        m_grasp = create_arrow_points_marker(2, 'grasp', grasp.position, p_grasp_end, 0.0, 1.0, 0.2)

        # This arrow connects targets; the actual path retreats before lifting.
        m_lift = create_arrow_points_marker(3, 'lift', grasp.position, lift.position, 1.0, 0.8, 0.0)

        # Pre-place: descent into the basket.
        m_pre_p = create_arrow_points_marker(4, 'pre_place', pre_p.position, place.position, 0.6, 0.2, 0.8)

        # Place: 8 cm arrow toward the basket bottom.
        p_place_end = Point(x=place.position.x, y=place.position.y, z=place.position.z - 0.08)
        m_place = create_arrow_points_marker(5, 'place', place.position, p_place_end, 1.0, 0.0, 0.4)

        # Retreat: rise 22 cm after release.
        m_ret = create_arrow_points_marker(6, 'retreat', place.position, retreat.position, 0.0, 0.9, 0.9)

        marker_array.markers.extend([m_pre_g, m_grasp, m_lift, m_pre_p, m_place, m_ret])
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceManager()
    try:
        # The main executor handles subscriptions, services and action results;
        # the pick-and-place sequence waits for them from its worker thread.
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
