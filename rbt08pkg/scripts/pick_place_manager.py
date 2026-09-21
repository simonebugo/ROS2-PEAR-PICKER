#!/usr/bin/env python3

import time
import threading
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Empty
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PositionIKRequest,
    JointConstraint,
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


class PickPlaceManager(Node):
    """
    Coordinatore centrale per il Pick and Place della Pera.
    Gestisce TF2, calcolo pose relative di Pick & Place, MoveIt Attach/Detach
    e la pubblicazione delle pose operative per la traiettoria.
    """

    # Il trasporto al cestino ha richiesto circa 67 s sul controller Gazebo.
    # Lasciare margine per la simulazione lenta senza interrompere un goal valido.
    EXECUTION_TIMEOUT_S = 120.0

    def __init__(self):
        super().__init__('pick_place_manager')
        self.declare_parameter('require_vision_status', False)
        self.declare_parameter('max_pose_age_s', 5.0)
        self._require_vision_status = bool(self.get_parameter('require_vision_status').value)
        self._max_pose_age = float(self.get_parameter('max_pose_age_s').value)
        if not math.isfinite(self._max_pose_age) or self._max_pose_age <= 0:
            raise ValueError('max_pose_age_s deve essere positivo e finito.')
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

        # Subscriber per la posa della pera
        self.sub_pear_pose = self.create_subscription(
            PoseStamped,
            '/harvest/pear_pose',
            self.pear_pose_callback,
            10
        )

        # Subscriber per lo stato del GraspManager
        self.sub_grasp_state = self.create_subscription(
            String,
            '/harvest/state',
            self.grasp_state_callback,
            10
        )

        # Subscriber per il trigger manuale di avvio da terminale
        self.sub_trigger = self.create_subscription(
            Empty,
            '/harvest/trigger',
            self.trigger_callback,
            10
        )

        # Publisher Pose Operative Pick (Pera)
        self.pub_pre_grasp = self.create_publisher(PoseStamped, '/harvest/pre_grasp_pose', 10)
        self.pub_grasp = self.create_publisher(PoseStamped, '/harvest/grasp_pose', 10)
        self.pub_lift = self.create_publisher(PoseStamped, '/harvest/lift_pose', 10)

        # Publisher Pose Operative Place (Cestino - FASE 6)
        self.pub_pre_place = self.create_publisher(PoseStamped, '/harvest/pre_place_pose', 10)
        self.pub_place = self.create_publisher(PoseStamped, '/harvest/place_pose', 10)
        self.pub_retreat = self.create_publisher(PoseStamped, '/harvest/retreat_pose', 10)

        # Publisher Marker 3D & Planning Scene
        self.pub_markers = self.create_publisher(MarkerArray, '/harvest/target_markers', 10)
        self.pub_planning_scene = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.pub_gripper_close_complete = self.create_publisher(
            Empty, '/harvest/gripper/close_complete', 10)

        self.current_grasp_state = 'UNKNOWN'
        self._grasp_state_condition = threading.Condition()
        self.sequence_executed = False

        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.015)
        self.declare_parameter('grasp_state_timeout_s', 8.0)
        # False keeps the standalone MoveIt/RViz demo usable. The integrated
        # Gazebo launch enables it so contact and detachable joints are mandatory.
        self.declare_parameter('require_physical_grasp', False)
        
        # Timer periodico per pubblicare le pose del cestino anche prima di ricevere la posa pera
        self.create_timer(1.0, self.publish_basket_poses)

        self.get_logger().info('PickPlaceManager FASE 6: Coordinatore Pick & Place completo con pose Cestino.')


        # Action Client per movimento libero e traiettorie
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

        # Service Client per movimento rettilineo e solutore IK
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info('Client MoveIt 2 (/move_action, /execute_trajectory, /compute_cartesian_path, /compute_ik) pronti.')

    def _spin_future(self, future, timeout_sec=10.0):
        """Wait from the sequence thread while the main executor handles ROS."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def compute_ik_for_pose(self, target_pose: PoseStamped):
        """Calcola l'IK per una posa target. Restituisce la soluzione articolare o None se irraggiungibile."""
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'grasp_frame'
        req.ik_request.pose_stamped = target_pose
        req.ik_request.avoid_collisions = False  # Cinematica pura senza controllo collisioni
        req.ik_request.timeout.sec = 5

        future = self._ik_client.call_async(req)
        self._spin_future(future, timeout_sec=8.0)
        res = future.result()

        if res and res.error_code.val == 1:  # 1 = SUCCESS
            self.get_logger().info("IK: Soluzione cinematica pura trovata.")
            return res.solution.joint_state

        self.get_logger().error(
            f"IK: Posa irraggiungibile cinematicamente "
            f"({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})"
        )
        return None


    def plan_and_move_to_pose(self, target_pose: PoseStamped) -> bool:
        """
        Pianifica ed esegue un movimento nello spazio libero verso una posa target usando ComputeIK e MoveGroup.
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server /move_action non disponibile!")
            return False

        # Verifica indicativa raggiungibilità
        self.compute_ik_for_pose(target_pose)

        self.get_logger().info("Invio Pose Goal a MoveGroup (plan + execute)...")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        goal_msg.request.start_state.is_diff = True

        # Invece di vincolare tutti i 6 giunti a una singola soluzione IK rigida,
        # inviamo a MoveGroup una Pose Goal (PositionConstraint + OrientationConstraint).
        # In questo modo il planner OMPL campiona liberamente diverse soluzioni IK
        # trovando un percorso privo di collisioni.
        constraints = Constraints()
        constraints.name = "pose_target"

        # 1. Position Constraint (tolleranza sferica di 2 cm)
        pos_constraint = PositionConstraint()
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = 'grasp_frame'

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]  # raggio 3 cm di tolleranza
        bv.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        sphere_pose.orientation.w = 1.0
        bv.primitive_poses.append(sphere_pose)

        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        # 2. Orientation Constraint (tolleranza angolare di ~11 gradi)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.stamp = self.get_clock().now().to_msg()
        orient_constraint.header.frame_id = 'world'
        orient_constraint.link_name = 'grasp_frame'
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.20
        orient_constraint.absolute_y_axis_tolerance = 0.20
        orient_constraint.absolute_z_axis_tolerance = 0.20
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True  # Calcola prima la traiettoria pulita
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        # The pear remains a world CollisionObject during pre-grasp planning.
        # It is attached to grasp_frame only after GraspManager confirms
        # ON_GRIPPER in attach_pear_in_moveit().

        self.get_logger().info(
            f"Pianificazione traiettoria verso ({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})..."
        )

        # 1. Invia richiesta di pianificazione a MoveGroup
        goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._spin_future(goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Richiesta di pianificazione rifiutata da MoveIt!")
            return False

        self.get_logger().info("Goal accettato. Pianificazione in corso...")

        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, timeout_sec=20.0)
        result = result_future.result()

        if not result or result.result.error_code.val != 1:
            code = result.result.error_code.val if (result and hasattr(result.result, 'error_code')) else -1
            plan_time = getattr(result.result, 'planning_time', 0.0) if result else 0.0
            self.get_logger().error(f"Pianificazione fallita! Codice MoveIt: {code} (Tempo: {plan_time:.2f}s)")
            return False

        # 2. Pianificazione riuscita: invia la traiettoria al controller tramite ExecuteTrajectory
        pts_count = len(result.result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(f"Pianificazione riuscita ({pts_count} waypoint)! Esecuzione sul controller...")

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.result.planned_trajectory

        exec_future = self._execute_trajectory_client.send_goal_async(exec_goal)
        self._spin_future(exec_future, timeout_sec=10.0)
        exec_handle = exec_future.result()

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error("Traiettoria rifiutata da ExecuteTrajectory!")
            return False

        exec_res_future = exec_handle.get_result_async()
        if not self._spin_future(exec_res_future, timeout_sec=self.EXECUTION_TIMEOUT_S):
            self.get_logger().error(
                f'Timeout esecuzione MoveIt dopo {self.EXECUTION_TIMEOUT_S:.0f} s; '
                'il controller potrebbe ancora completare il goal.')
            return False
        exec_res = exec_res_future.result()

        if exec_res and exec_res.result.error_code.val == 1:
            self.get_logger().info("Movimento completato con successo! Robot si è spostato.")
            return True
        else:
            code = exec_res.result.error_code.val if exec_res else -1
            self.get_logger().error(f"Esecuzione traiettoria fallita sul controller! Codice: {code}")
            return False

    def execute_cartesian_path(self, waypoints: list, avoid_collisions: bool = True) -> bool:
        """
        Calcola ed esegue una traiettoria cartesiana rettilinea passando per i waypoints.
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servizio /compute_cartesian_path non disponibile!")
            return False

        req = GetCartesianPath.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.link_name = 'grasp_frame'
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions

        self.get_logger().info(f"Calcolo traiettoria cartesiana per {len(waypoints)} waypoint...")
        future = self._cartesian_client.call_async(req)
        self._spin_future(future, timeout_sec=10.0)
        res = future.result()

        if res is None or res.fraction < 0.5:
            fraction = res.fraction * 100 if res else 0
            self.get_logger().warn(f"Traiettoria cartesiana insufficiente ({fraction:.1f}% coperto). Salto.")
            return False

        self.get_logger().info(f"Traiettoria calcolata ({res.fraction * 100:.1f}%). Invio a /execute_trajectory...")

        if not self._execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/execute_trajectory non disponibile!")
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        exec_future = self._execute_trajectory_client.send_goal_async(exec_goal)
        self._spin_future(exec_future, timeout_sec=10.0)
        exec_handle = exec_future.result()

        if not exec_handle or not exec_handle.accepted:
            self.get_logger().error("Traiettoria cartesiana rifiutata dal controller!")
            return False

        res_future = exec_handle.get_result_async()
        if not self._spin_future(res_future, timeout_sec=self.EXECUTION_TIMEOUT_S):
            self.get_logger().error(
                f'Timeout esecuzione cartesiana dopo {self.EXECUTION_TIMEOUT_S:.0f} s; '
                'il controller potrebbe ancora completare il goal.')
            return False
        exec_result = res_future.result()
        if exec_result and exec_result.result.error_code.val == 1:
            self.get_logger().info("Traiettoria cartesiana eseguita con successo.")
            return True

        error_code = exec_result.result.error_code.val if exec_result else -1
        self.get_logger().error(
            f"Esecuzione cartesiana fallita sul controller! Codice: {error_code}")
        return False

    def grasp_state_callback(self, msg: String):
        state_text = msg.data.split(':')[0].strip()
        with self._grasp_state_condition:
            state_changed = state_text != self.current_grasp_state
            self.current_grasp_state = state_text

        if state_changed:
            self.get_logger().info(f'Stato GraspManager aggiornato: {self.current_grasp_state}')

            if self.current_grasp_state == 'ON_GRIPPER':
                self.attach_pear_in_moveit()
            elif self.current_grasp_state == 'RELEASED':
                self.detach_pear_in_moveit()

        # Notify only after the corresponding Planning Scene update has been
        # published, so the sequence cannot start its next plan too early.
        with self._grasp_state_condition:
            self._grasp_state_condition.notify_all()

    def command_gripper(self, position: float, movement_time_s: float = 2.0) -> bool:
        """Invia una posizione simmetrica al controller fisico del gripper."""
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Action server /gripper_controller/follow_joint_trajectory non disponibile!')
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
            self.get_logger().error('Comando del gripper rifiutato dal controller!')
            return False

        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, timeout_sec=15.0)
        result = result_future.result()
        if (result is None or
                result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            error_code = result.result.error_code if result else -1
            self.get_logger().error(
                f'Movimento del gripper fallito! Codice: {error_code}')
            return False

        self.get_logger().info(
            f'Gripper portato alla posizione {position:.3f} m.')
        return True

    def wait_for_grasp_state(self, expected_state: str) -> bool:
        """Attende la conferma fisica del GraspManager senza bloccare i callback ROS."""
        timeout_s = float(self.get_parameter('grasp_state_timeout_s').value)
        deadline = time.monotonic() + timeout_s
        with self._grasp_state_condition:
            while self.current_grasp_state != expected_state:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.get_logger().error(
                        f'Timeout: stato {expected_state} non ricevuto dal GraspManager.')
                    return False
                self._grasp_state_condition.wait(timeout=remaining)
        return True

    def attach_pear_in_moveit(self):
        """Aggancia la pera a grasp_frame in MoveIt per evitare falsi allarmi di collisione con le dita."""
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
        self.get_logger().info('MoveIt Planning Scene: Pera AGGANCIATA a grasp_frame (AttachedCollisionObject).')

    def detach_pear_in_moveit(self):
        """Stacca la pera da grasp_frame in MoveIt."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = 'grasp_frame'
        attached_obj.object.id = 'pear'
        attached_obj.object.operation = CollisionObject.REMOVE

        scene_msg.robot_state.attached_collision_objects.append(attached_obj)
        self.pub_planning_scene.publish(scene_msg)
        self.get_logger().info('MoveIt Planning Scene: Pera SGANCIATA dal robot (Detached).')

    def add_pear_to_world(self, pose: Pose) -> None:
        """Reinserisce la pera come ostacolo statico nel demo MoveIt/RViz."""
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
            'MoveIt Planning Scene: Pera reinserita nel world alla posa di deposito.')

    def publish_basket_poses(self):
        """FASE 6: Definizione e pubblicazione delle pose operative per il cestino (frame base_link)."""
        basket_world_x = 0.65
        basket_world_y = -0.55
        basket_world_z = 0.0

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
        """Trasforma la posa della pera in base_link e calcola pre_grasp, grasp, lift."""
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
            self.get_logger().warn(f'Errore TF2 nella trasformazione della posa pera: {e}')
            return

        px = pear_in_base.pose.position.x
        py = pear_in_base.pose.position.y
        pz = pear_in_base.pose.position.z
        if not all(math.isfinite(v) for v in (px, py, pz)):
            return

        # Angolo di approccio nel piano XY (direzione orizzontale verso la pera)
        approach_yaw = math.atan2(py, px)

        # ---------------------------------------------------------------
        # Orientamento del gripper per la PERA (approccio ORIZZONTALE):
        #
        # Geometria gripper (dal URDF):
        #   - Asse Z di grasp_frame = direzione di presa (punta FUORI dal gripper)
        #   - Asse Y di grasp_frame = direzione apertura dita (left/right finger)
        #
        # Obiettivo:
        #   - Z punti ORIZZONTALMENTE verso la pera → pitch = +π/2
        #   - Y rimanga ORIZZONTALE → dita affiancate sinistra-destra (parallele al suolo)
        #   - yaw = approach_yaw → il gripper punta nella direzione della pera nel piano XY
        #
        # Risultato visivo: il gripper arriva LATERALMENTE alla pera,
        # con le due dita aperte a sinistra e destra (non una sopra e una sotto).
        # ---------------------------------------------------------------
        # With the pre-grasp pose behind the fruit, +Z must point from the
        # robot towards the pear. This keeps link5 on the robot side of the
        # grasp frame instead of driving the wrist through the fruit.
        grasp_orientation = euler_to_quaternion(0.0, math.pi / 2.0, approach_yaw)

        dist_xy = math.sqrt(px * px + py * py)
        dir_x = px / dist_xy if dist_xy > 0 else 1.0
        dir_y = py / dist_xy if dist_xy > 0 else 0.0

        # Posa di presa: esattamente sulla pera
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position = Point(x=px, y=py, z=pz)
        grasp_pose.pose.orientation = grasp_orientation

        # Pre-grasp: 25 cm INDIETRO rispetto alla pera nel piano XY (stesso Z della pera).
        # Maggiore distanza per dare ampio respiro al robot prima dell'avvicinamento rettilineo.
        offset_pre = 0.25
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header = grasp_pose.header
        pre_grasp_pose.pose.position = Point(
            x=px - dir_x * offset_pre,
            y=py - dir_y * offset_pre,
            z=pz  # stesso Z della pera: approccio perfettamente orizzontale
        )
        pre_grasp_pose.pose.orientation = grasp_orientation

        # Lift: after retreating to the pre-grasp clearance, raise the pear by
        # 20 cm. Raising it directly at the grasp position would cross the
        # fruit branch above it.
        offset_lift = 0.20
        lift_pose = PoseStamped()
        lift_pose.header = grasp_pose.header
        lift_pose.pose.position = Point(
            x=pre_grasp_pose.pose.position.x,
            y=pre_grasp_pose.pose.position.y,
            z=pz + offset_lift,
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

        # PER AVVIO AUTOMATICO IMMEDIATO (es. con la telecamera): scommentare la riga sottostante
        # self.start_pick_and_place_sequence(pre_grasp_pose, grasp_pose, lift_pose, pre_place_pose, place_pose, retreat_pose)

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
            return 'posa della pera non ancora disponibile'
        p = self._pear_msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return 'coordinate della pera non finite'
        if self.latest_poses is None:
            return 'posa non disponibile in base_link (verificare TF)'
        stamp = self._pear_msg.header.stamp
        now = time.monotonic()
        ages = [now - self._pear_received_at,
                (self.get_clock().now().nanoseconds -
                 (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9]
        if self._require_vision_status:
            status = self._center_status
            if (status.get('valid') is not True or
                    status.get('stamp') != {'sec': stamp.sec, 'nanosec': stamp.nanosec}):
                return 'center_status non valido per questa misura'
            try:
                ages.append(float(status['received_age_s']) + now - self._center_status_at)
            except (KeyError, TypeError, ValueError):
                return 'età della misura vision non disponibile'
        if any(not math.isfinite(age) or age < 0 or age > self._max_pose_age for age in ages):
            return f'posa scaduta o timestamp non coerente (limite {self._max_pose_age:.1f} s)'
        return None

    def trigger_callback(self, msg: Empty):
        """Chiamato quando l'utente invia il comando manuale di avvio su /harvest/trigger."""
        reason = self.target_rejection_reason()
        if reason is None:
            self.get_logger().info('TRIGGER MANUALE RICEVUTO! Avvio della sequenza in un thread dedicato...')
            thread = threading.Thread(target=self.start_pick_and_place_sequence, args=self.latest_poses)
            thread.start()
        else:
            self.get_logger().warn(f'Trigger rifiutato: {reason}.')

    def start_pick_and_place_sequence(self, pre_g: PoseStamped, grasp: PoseStamped, lift: PoseStamped, pre_p: PoseStamped, place: PoseStamped, retreat: PoseStamped):
        """
        Esegue l'intera sequenza automatica Pick and Place in 8 fasi.
        """
        self.get_logger().info("=== AVVIO SEQUENZA PICK AND PLACE ===")

        # FASE 1: Movimento nello spazio libero verso la posa pre_grasp
        self.get_logger().info("[FASE 1/8] Movimento a pre_grasp...")
        if not self.plan_and_move_to_pose(pre_g):
            self.get_logger().error("FASE 1 FALLITA: Impossibile raggiungere pre_grasp. Sequenza interrotta.")
            return

        # FASE 2: Avvicinamento cartesiano rettilineo sulla pera (grasp).
        # The final approach intentionally permits fingertip-to-pear contact.
        self.get_logger().info("[FASE 2/8] Avvicinamento rettilineo alla pera...")
        if not self.execute_cartesian_path([grasp.pose], avoid_collisions=False):
            self.get_logger().error("FASE 2 FALLITA: Traiettoria verso grasp fallita. Sequenza interrotta.")
            return

        # FASE 3 & 4: Chiusura dita ed attesa aggancio fisico (GraspManager ON_GRIPPER)
        self.get_logger().info("[FASE 3/8] Presa pera in corso (attesa conferma GraspManager)...")
        closed_position = float(self.get_parameter('gripper_closed_position').value)
        if not self.command_gripper(closed_position):
            self.get_logger().error("FASE 3 FALLITA: Chiusura del gripper fallita.")
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
                "FASE 4 FALLITA: presa non confermata dai contatti/detachable joint.")
            return
        if not require_physical_grasp:
            # In the standalone MoveIt/RViz demo there is no Gazebo
            # GraspManager event to perform this transition.  Make the same
            # logical attachment before planning the retreat; otherwise the
            # gripper starts phase 5 in collision with the pear still present
            # as a world CollisionObject.
            self.attach_pear_in_moveit()
            time.sleep(0.2)

        # FASE 5: Move away from the branch before lifting. The first waypoint
        # retraces the safe horizontal approach; the second raises the pear in
        # the cleared area, away from the trunk and fruit branch.
        self.get_logger().info("[FASE 5/8] Arretramento dal ramo e sollevamento pera...")
        # The start state intentionally contains the newly grasped pear close
        # to the fingers and branch.  Permit the known outward escape path;
        # collision checking resumes for the free-space transport afterwards.
        if not self.execute_cartesian_path(
                [pre_g.pose, lift.pose], avoid_collisions=False):
            self.get_logger().error("FASE 5 FALLITA: Arretramento/lift dal ramo fallito. Sequenza interrotta.")
            return

        # FASE 6: Trasporto nello spazio libero verso il cestino (pre_place)
        self.get_logger().info("[FASE 6/8] Trasporto della pera al cestino...")
        if not self.plan_and_move_to_pose(pre_p):
            self.get_logger().error("FASE 6 FALLITA: Trasporto verso il cestino fallito. Sequenza interrotta.")
            return

        # FASE 7: Discesa cartesiana rettilinea nel cestino (place +18 cm)
        self.get_logger().info("[FASE 7/8] Discesa nel cestino...")
        if not self.execute_cartesian_path([place.pose]):
            self.get_logger().error("FASE 7 FALLITA: Discesa nel cestino fallita. Sequenza interrotta.")
            return

        # FASE 8: apertura, distacco fisico e risalita di disimpegno.
        self.get_logger().info("[FASE 8/8] Rilascio pera e risalita di sicurezza...")
        open_position = float(self.get_parameter('gripper_open_position').value)
        if not self.command_gripper(open_position):
            self.get_logger().error("FASE 8 FALLITA: Apertura del gripper fallita.")
            return
        if (require_physical_grasp and
                not self.wait_for_grasp_state('RELEASED')):
            self.get_logger().error(
                "FASE 8 FALLITA: distacco della pera non confermato.")
            return
        if not require_physical_grasp:
            self.detach_pear_in_moveit()
            time.sleep(0.2)
        if not self.execute_cartesian_path([retreat.pose]):
            self.get_logger().error("FASE 8 FALLITA: Risalita di sicurezza fallita.")
            return
        if not require_physical_grasp:
            # Add it back only once the TCP has cleared the fruit; adding it
            # before the retreat would make the Cartesian path start inside
            # the freshly released collision object.
            self.add_pear_to_world(place.pose)

        self.get_logger().info("=== SEQUENZA PICK AND PLACE COMPLETATA CON SUCCESSO! ===")

    def publish_all_markers(self, pre_g, grasp, lift, pre_p, place, retreat):
        """Pubblica tutti e 6 i marker 3D (frecce) per la visualizzazione completa dell'algoritmo in RViz."""
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
            m.scale.x = 0.02  # Diametro asta
            m.scale.y = 0.04  # Diametro punta
            m.scale.z = 0.06  # Lunghezza punta
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.95
            return m

        # Direzione orizzontale verso la pera
        dx = grasp.position.x - pre_g.position.x
        dy = grasp.position.y - pre_g.position.y
        d_len = math.sqrt(dx * dx + dy * dy)
        ux = dx / d_len if d_len > 0 else 1.0
        uy = dy / d_len if d_len > 0 else 0.0

        # 1. Pre-grasp: freccia orizzontale che punta verso la pera (lunghezza 15 cm)
        p_pre_end = Point(x=pre_g.position.x + ux * 0.15, y=pre_g.position.y + uy * 0.15, z=pre_g.position.z)
        m_pre_g = create_arrow_points_marker(1, 'pre_grasp', pre_g.position, p_pre_end, 0.0, 0.5, 1.0)   # Blu

        # 2. Grasp: freccia orizzontale sulla pera (lunghezza 10 cm lungo l'asse di presa)
        p_grasp_end = Point(x=grasp.position.x + ux * 0.10, y=grasp.position.y + uy * 0.10, z=grasp.position.z)
        m_grasp = create_arrow_points_marker(2, 'grasp', grasp.position, p_grasp_end, 0.0, 1.0, 0.2)       # Verde

        # 3. Lift: freccia verticale che va dalla posa grasp alla posa lift (+20 cm in alto)
        m_lift = create_arrow_points_marker(3, 'lift', grasp.position, lift.position, 1.0, 0.8, 0.0)         # Giallo

        # 4. Pre-place: freccia verticale verso il basso da pre_place a place
        m_pre_p = create_arrow_points_marker(4, 'pre_place', pre_p.position, place.position, 0.6, 0.2, 0.8)   # Viola

        # 5. Place: freccia verso il fondo del cestino (8 cm verso il basso)
        p_place_end = Point(x=place.position.x, y=place.position.y, z=place.position.z - 0.08)
        m_place = create_arrow_points_marker(5, 'place', place.position, p_place_end, 1.0, 0.0, 0.4)        # Magenta/Rosso

        # 6. Retreat: freccia verso l'alto da place a retreat (+22 cm verso l'alto)
        m_ret = create_arrow_points_marker(6, 'retreat', place.position, retreat.position, 0.0, 0.9, 0.9)     # Ciano

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
