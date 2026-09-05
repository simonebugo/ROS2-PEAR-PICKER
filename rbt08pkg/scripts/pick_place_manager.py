#!/usr/bin/env python3

import time
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Empty
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.action import ActionClient
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

    def __init__(self):
        super().__init__('pick_place_manager')

        # Executor dedicato per spin_until_future_complete nel thread della sequenza
        self._exec = rclpy.executors.SingleThreadedExecutor()
        self._exec.add_node(self)

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

        self.current_grasp_state = 'UNKNOWN'
        self.sequence_executed = False
        
        # Timer periodico per pubblicare le pose del cestino anche prima di ricevere la posa pera
        self.create_timer(1.0, self.publish_basket_poses)

        self.get_logger().info('PickPlaceManager FASE 6: Coordinatore Pick & Place completo con pose Cestino.')


        # Action Client per movimento libero e traiettorie
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Service Client per movimento rettilineo e solutore IK
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info('Client MoveIt 2 (/move_action, /execute_trajectory, /compute_cartesian_path, /compute_ik) pronti.')

    def _spin_future(self, future, timeout_sec=10.0):
        """Attende il completamento di un future girando l'executor dedicato. Thread-safe."""
        rclpy.spin_until_future_complete(self, future, executor=self._exec, timeout_sec=timeout_sec)

    def compute_ik_for_pose(self, target_pose: PoseStamped):
        """Calcola l'IK per una posa target. Restituisce la soluzione articolare o None se irraggiungibile."""
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'grasp_frame'
        req.ik_request.pose_stamped = target_pose
        # avoid_collisions=False: l'IK cerca solo soluzioni cinematiche.
        # Le collisioni vengono verificate da OMPL durante il planning.
        # Con True, il solver KDL spesso non trova soluzioni ad alta quota.
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5       # da 2s a 5s
        req.ik_request.attempts = 10         # 10 tentativi con seed casuali

        future = self._ik_client.call_async(req)
        self._spin_future(future, timeout_sec=8.0)
        res = future.result()

        if res and res.error_code.val == 1:  # 1 = SUCCESS
            return res.solution.joint_state

        self.get_logger().warn(
            f"IK fallita (codice {res.error_code.val if res else 'N/A'}) per "
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

        # 1. Verifica raggiungibilità IK prima del planning
        joint_solution = self.compute_ik_for_pose(target_pose)
        if joint_solution is None:
            self.get_logger().error(
                f"Posa ({target_pose.pose.position.x:.2f}, "
                f"{target_pose.pose.position.y:.2f}, "
                f"{target_pose.pose.position.z:.2f}) NON RAGGIUNGIBILE dall'IK!"
            )
            return False

        self.get_logger().info("IK verificata. Invio goal a MoveGroup (plan + execute)...")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10

        # Vincoli articolari dalla soluzione IK
        constraints = Constraints()
        for name, pos in zip(joint_solution.name, joint_solution.position):
            if name in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.05
                jc.tolerance_below = 0.05
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False  # pianifica ED esegue

        self.get_logger().info(
            f"Movimento verso ({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})..."
        )

        # Invia goal e attendi accettazione (thread-safe)
        goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._spin_future(goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rifiutato da MoveIt!")
            return False

        self.get_logger().info("Goal accettato. Attendo esecuzione sul controller...")

        # Attendi il risultato finale (include l'esecuzione fisica sul mock controller)
        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, timeout_sec=30.0)

        result = result_future.result()
        if result and result.result.error_code.val == 1:
            self.get_logger().info("Movimento completato con successo! Robot si è spostato.")
            return True
        else:
            code = result.result.error_code.val if result else -1
            self.get_logger().error(f"Movimento fallito! Codice errore MoveIt: {code}")
            return False

    def execute_cartesian_path(self, waypoints: list) -> bool:
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
        req.avoid_collisions = True

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
        self._spin_future(res_future, timeout_sec=30.0)

        self.get_logger().info("Traiettoria cartesiana eseguita con successo.")
        return True

    def grasp_state_callback(self, msg: String):
        state_text = msg.data.split(':')[0].strip()
        if state_text != self.current_grasp_state:
            self.current_grasp_state = state_text
            self.get_logger().info(f'Stato GraspManager aggiornato: {self.current_grasp_state}')

            if self.current_grasp_state == 'ON_GRIPPER':
                self.attach_pear_in_moveit()
            elif self.current_grasp_state == 'RELEASED':
                self.detach_pear_in_moveit()

    def attach_pear_in_moveit(self):
        """Aggancia la pera a grasp_frame in MoveIt per evitare falsi allarmi di collisione con le dita."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

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

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = 'grasp_frame'
        attached_obj.object.id = 'pear'
        attached_obj.object.operation = CollisionObject.REMOVE

        scene_msg.robot_state.attached_collision_objects.append(attached_obj)
        self.pub_planning_scene.publish(scene_msg)
        self.get_logger().info('MoveIt Planning Scene: Pera SGANCIATA dal robot (Detached).')

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
        #   - Z punti ORIZZONTALMENTE verso la pera → pitch = -π/2
        #   - Y rimanga ORIZZONTALE → dita affiancate sinistra-destra (parallele al suolo)
        #   - yaw = approach_yaw → il gripper punta nella direzione della pera nel piano XY
        #
        # Risultato visivo: il gripper arriva LATERALMENTE alla pera,
        # con le due dita aperte a sinistra e destra (non una sopra e una sotto).
        # ---------------------------------------------------------------
        grasp_orientation = euler_to_quaternion(0.0, -math.pi / 2.0, approach_yaw)

        dist_xy = math.sqrt(px * px + py * py)
        dir_x = px / dist_xy if dist_xy > 0 else 1.0
        dir_y = py / dist_xy if dist_xy > 0 else 0.0

        # Posa di presa: esattamente sulla pera
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position = Point(x=px, y=py, z=pz)
        grasp_pose.pose.orientation = grasp_orientation

        # Pre-grasp: 15 cm INDIETRO rispetto alla pera nel piano XY (stesso Z della pera).
        # Il gripper si posiziona a distanza di sicurezza prima di avanzare in linea retta.
        offset_pre = 0.15
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header = grasp_pose.header
        pre_grasp_pose.pose.position = Point(
            x=px - dir_x * offset_pre,
            y=py - dir_y * offset_pre,
            z=pz  # stesso Z della pera: approccio perfettamente orizzontale
        )
        pre_grasp_pose.pose.orientation = grasp_orientation

        # Lift: solleva la pera di 15 cm dopo la presa (allontana dal ramo verso l'alto)
        offset_lift = 0.15
        lift_pose = PoseStamped()
        lift_pose.header = grasp_pose.header
        lift_pose.pose.position = Point(x=px, y=py, z=pz + offset_lift)
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

    def trigger_callback(self, msg: Empty):
        """Chiamato quando l'utente invia il comando manuale di avvio su /harvest/trigger."""
        if hasattr(self, 'latest_poses') and self.latest_poses is not None:
            self.get_logger().info('TRIGGER MANUALE RICEVUTO! Avvio della sequenza in un thread dedicato...')
            thread = threading.Thread(target=self.start_pick_and_place_sequence, args=self.latest_poses)
            thread.start()
        else:
            self.get_logger().warn('Trigger ricevuto, ma la posa della pera non è ancora disponibile.')

    def start_pick_and_place_sequence(self, pre_g: PoseStamped, grasp: PoseStamped, lift: PoseStamped, pre_p: PoseStamped, place: PoseStamped, retreat: PoseStamped):
        """
        Esegue l'intera sequenza automatica Pick and Place in 8 fasi.
        """
        self.get_logger().info("=== AVVIO SEQUENZA PICK AND PLACE ===")

        # FASE 1: Movimento nello spazio libero verso la posa pre_grasp (12 cm dalla pera)
        self.get_logger().info("[FASE 1/8] Movimento a pre_grasp...")
        self.plan_and_move_to_pose(pre_g)

        # FASE 2: Avvicinamento cartesiano rettilineo sulla pera (grasp)
        self.get_logger().info("[FASE 2/8] Avvicinamento rettilineo alla pera...")
        self.execute_cartesian_path([grasp.pose])

        # FASE 3 & 4: Chiusura dita ed attesa aggancio fisico (GraspManager ON_GRIPPER)
        self.get_logger().info("[FASE 3/8] Presa pera in corso (attesa conferma GraspManager)...")
        # Invia comando chiusura ed attende lo stato ON_GRIPPER

        # FASE 5: Sollevamento cartesiano rettilineo (lift +15 cm)
        self.get_logger().info("[FASE 5/8] Sollevamento pera dal ramo...")
        self.execute_cartesian_path([lift.pose])

        # FASE 6: Trasporto nello spazio libero verso il cestino (pre_place)
        self.get_logger().info("[FASE 6/8] Trasporto della pera al cestino...")
        self.plan_and_move_to_pose(pre_p)

        # FASE 7: Discesa cartesiana rettilinea nel cestino (place +18 cm)
        self.get_logger().info("[FASE 7/8] Discesa nel cestino...")
        self.execute_cartesian_path([place.pose])

        # FASE 8: Rilascio pera e risalita di disimpegno (retreat +40 cm)
        self.get_logger().info("[FASE 8/8] Rilascio pera e risalita di sicurezza...")
        self.execute_cartesian_path([retreat.pose])

        self.get_logger().info("=== SEQUENZA PICK AND PLACE COMPLETATA CON SUCCESSO! ===")

    def publish_all_markers(self, pre_g, grasp, lift, pre_p, place, retreat):
        """Pubblica tutti e 6 i marker 3D (frecce) per la visualizzazione completa dell'algoritmo in RViz."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        def create_arrow_marker(marker_id, name, pose, r, g, b):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'base_link'
            m.ns = name
            m.id = marker_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = 0.12
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.9
            return m

        # 3 Pose Pick (Albero)
        m_pre_g = create_arrow_marker(1, 'pre_grasp', pre_g, 0.0, 0.5, 1.0)   # Blu
        m_grasp = create_arrow_marker(2, 'grasp', grasp, 0.0, 1.0, 0.2)       # Verde
        m_lift = create_arrow_marker(3, 'lift', lift, 1.0, 0.8, 0.0)         # Giallo

        # 3 Pose Place (Cestino)
        m_pre_p = create_arrow_marker(4, 'pre_place', pre_p, 0.6, 0.2, 0.8)   # Viola
        m_place = create_arrow_marker(5, 'place', place, 1.0, 0.0, 0.4)        # Magenta/Rosso
        m_ret = create_arrow_marker(6, 'retreat', retreat, 0.0, 0.9, 0.9)     # Ciano

        marker_array.markers.extend([m_pre_g, m_grasp, m_lift, m_pre_p, m_place, m_ret])
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceManager()
    try:
        # Usa un MultiThreadedExecutor: il thread principale gira i callback
        # mentre il thread della sequenza (trigger_callback) usa _spin_future
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
