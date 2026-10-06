#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


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

    return qx, qy, qz, qw


class PlanningSceneSpawner(Node):
    # Keep the planner this far from the physical branch to account for
    # controller tracking error during the retreat and lift.
    BRANCH_CLEARANCE_M = 0.015

    def __init__(self):
        super().__init__('planning_scene_spawner')
        self.declare_parameter('scene_config', '')
        scenario_path = self.get_parameter('scene_config').value
        self.scenario = None
        if scenario_path:
            from orchard_randomization import load_scenario
            self.scenario = load_scenario(scenario_path)
            self.get_logger().info(f'Collision scene from {scenario_path}; seed={self.scenario["seed"]}')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.get_logger().info('Initializing Planning Scene Spawner...')
        # Give publisher time to connect
        time.sleep(1.0)
        self.spawn_scene_objects()

    def spawn_scene_objects(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        objects = []

        # 1. Ground Plane
        ground = CollisionObject()
        ground.header.frame_id = 'world'
        ground.id = 'ground'
        ground.operation = CollisionObject.ADD
        box_ground = SolidPrimitive()
        box_ground.type = SolidPrimitive.BOX
        # Solid forbidden region below z=0; a thin slab permits IK below it.
        box_ground.dimensions = [10.0, 10.0, 5.0]
        pose_ground = Pose()
        pose_ground.position.x = 0.0
        pose_ground.position.y = 0.0
        pose_ground.position.z = -2.5
        pose_ground.orientation.w = 1.0
        ground.primitives.append(box_ground)
        ground.primitive_poses.append(pose_ground)
        objects.append(ground)

        # 2. Tree Trunk (Cylinder: height 2.10m, radius 0.10m at world pose x=1.35, y=0.55, z=1.05)
        trunk = CollisionObject()
        trunk.header.frame_id = 'world'
        trunk.id = 'tree_trunk'
        trunk.operation = CollisionObject.ADD
        cyl_trunk = SolidPrimitive()
        cyl_trunk.type = SolidPrimitive.CYLINDER
        cyl_trunk.dimensions = [2.10, 0.10]  # [height, radius]
        pose_trunk = Pose()
        pose_trunk.position.x = 1.35
        pose_trunk.position.y = 0.55
        pose_trunk.position.z = 1.05
        pose_trunk.orientation.w = 1.0
        trunk.primitives.append(cyl_trunk)
        trunk.primitive_poses.append(pose_trunk)
        objects.append(trunk)

        # 3. Upper branch, matching branch_collision in orchard.world.sdf.
        upper_branch = CollisionObject()
        upper_branch.header.frame_id = 'world'
        upper_branch.id = 'upper_branch'
        upper_branch.operation = CollisionObject.ADD
        cyl_upper_branch = SolidPrimitive()
        cyl_upper_branch.type = SolidPrimitive.CYLINDER
        cyl_upper_branch.dimensions = [0.65, 0.045]
        pose_upper_branch = Pose()
        pose_upper_branch.position.x = 1.08
        pose_upper_branch.position.y = 0.55
        pose_upper_branch.position.z = 1.85
        qx, qy, qz, qw = euler_to_quaternion(0.0, 1.10, 0.0)
        pose_upper_branch.orientation.x = qx
        pose_upper_branch.orientation.y = qy
        pose_upper_branch.orientation.z = qz
        pose_upper_branch.orientation.w = qw
        upper_branch.primitives.append(cyl_upper_branch)
        upper_branch.primitive_poses.append(pose_upper_branch)
        objects.append(upper_branch)

        # 4. Fruit branch (length 0.60m, radius 0.04m, horizontal at z=1.48).
        #    World centre: trunk(1.35) - 0.30 = x=1.05, z=1.48.
        branch = CollisionObject()
        branch.header.frame_id = 'world'
        branch.id = 'fruit_branch'
        branch.operation = CollisionObject.ADD
        cyl_branch = SolidPrimitive()
        cyl_branch.type = SolidPrimitive.CYLINDER
        cyl_branch.dimensions = [0.60, 0.040 + self.BRANCH_CLEARANCE_M]
        pose_branch = Pose()
        pose_branch.position.x = 1.05
        pose_branch.position.y = 0.55
        pose_branch.position.z = 1.48
        qx, qy, qz, qw = euler_to_quaternion(0.0, 1.5708, 0.0)
        pose_branch.orientation.x = qx
        pose_branch.orientation.y = qy
        pose_branch.orientation.z = qz
        pose_branch.orientation.w = qw
        branch.primitives.append(cyl_branch)
        branch.primitive_poses.append(pose_branch)
        objects.append(branch)

        # 5. Collection Basket (Center: x=0.65, y=-0.55, z=0)
        basket_parts = [
            ('basket_bottom', [0.32, 0.32, 0.04], [0.65, -0.55, 0.02]),
            ('basket_left_wall', [0.32, 0.03, 0.22], [0.65, -0.405, 0.13]),
            ('basket_right_wall', [0.32, 0.03, 0.22], [0.65, -0.695, 0.13]),
            ('basket_front_wall', [0.03, 0.26, 0.22], [0.795, -0.55, 0.13]),
            ('basket_back_wall', [0.03, 0.26, 0.22], [0.505, -0.55, 0.13])
        ]

        for part_id, dims, pos in basket_parts:
            basket_obj = CollisionObject()
            basket_obj.header.frame_id = 'world'
            basket_obj.id = part_id
            basket_obj.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dims
            p = Pose()
            p.position.x = pos[0]
            p.position.y = pos[1]
            p.position.z = pos[2]
            p.orientation.w = 1.0
            basket_obj.primitives.append(box)
            basket_obj.primitive_poses.append(p)
            objects.append(basket_obj)

        # 6. Pear body and neck, matching both Gazebo collision spheres.
        pear = CollisionObject()
        pear.header.frame_id = 'world'
        pear.id = 'pear'
        pear.operation = CollisionObject.ADD
        sphere_pear = SolidPrimitive()
        sphere_pear.type = SolidPrimitive.SPHERE
        sphere_pear.dimensions = [0.075]  # [radius]
        pose_pear = Pose()
        pose_pear.position.x = 0.85
        pose_pear.position.y = 0.55
        pose_pear.position.z = 1.30
        pose_pear.orientation.w = 1.0
        pear.primitives.append(sphere_pear)
        pear.primitive_poses.append(pose_pear)

        sphere_pear_neck = SolidPrimitive()
        sphere_pear_neck.type = SolidPrimitive.SPHERE
        sphere_pear_neck.dimensions = [0.052]
        pose_pear_neck = Pose()
        pose_pear_neck.position.x = 0.85
        pose_pear_neck.position.y = 0.55
        pose_pear_neck.position.z = 1.365
        pose_pear_neck.orientation.w = 1.0
        pear.primitives.append(sphere_pear_neck)
        pear.primitive_poses.append(pose_pear_neck)
        objects.append(pear)

        if self.scenario is not None:
            values = self.scenario['branch_pose_world']
            pose_branch.position.x, pose_branch.position.y, pose_branch.position.z = values[:3]
            q = euler_to_quaternion(*values[3:])
            pose_branch.orientation.x, pose_branch.orientation.y, pose_branch.orientation.z, pose_branch.orientation.w = q
            cyl_branch.dimensions = [self.scenario['branch_length'],
                                     self.scenario['branch_radius'] + self.BRANCH_CLEARANCE_M]
            branch.primitives = [cyl_branch]
            branch.primitive_poses = [pose_branch]
            x, y, z = self.scenario['pear_pose_world'][:3]
            pose_pear.position.x, pose_pear.position.y, pose_pear.position.z = x, y, z
            roll, pitch, yaw = self.scenario['pear_pose_world'][3:]
            # Rotate the local neck offset with the same SDF model attitude.
            pose_pear_neck.position.x = x + .065*(math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll))
            pose_pear_neck.position.y = y + .065*(math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll))
            pose_pear_neck.position.z = z + .065*math.cos(pitch)*math.cos(roll)
            pear_q = euler_to_quaternion(*self.scenario['pear_pose_world'][3:])
            for pear_part in (pose_pear, pose_pear_neck):
                pear_part.orientation.x, pear_part.orientation.y = pear_q[:2]
                pear_part.orientation.z, pear_part.orientation.w = pear_q[2:]
            pear.primitive_poses = [pose_pear, pose_pear_neck]

        scene_msg.world.collision_objects = objects

        # Publish multiple times to ensure MoveIt receives the diff
        for _ in range(5):
            self.publisher.publish(scene_msg)
            time.sleep(0.2)

        self.get_logger().info('Successfully published collision objects to /planning_scene!')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneSpawner()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
