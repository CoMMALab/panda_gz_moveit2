#!/usr/bin/env python3
"""
Publishes static and known dynamic collision objects to the MoveIt planning
scene and on /collision_object for the behavior tree.

Static objects (table + legs) go only to /planning_scene.
Dynamic objects (blocks) go to both /collision_object (latched, for the BT)
and /planning_scene (for collision avoidance).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


# Table surface: centre z=0.25, thickness=0.04 → top at 0.27 m
_TABLE_SURFACE_Z = 0.27

BLOCKS = [
    {'id': 'block_0', 'x': 0.50, 'y': -0.10, 'z': _TABLE_SURFACE_Z + 0.025,
     'dx': 0.05, 'dy': 0.05, 'dz': 0.05},
    {'id': 'block_1', 'x': 0.50, 'y':  0.00, 'z': _TABLE_SURFACE_Z + 0.025,
     'dx': 0.05, 'dy': 0.05, 'dz': 0.05},
    {'id': 'block_2', 'x': 0.50, 'y':  0.10, 'z': _TABLE_SURFACE_Z + 0.025,
     'dx': 0.05, 'dy': 0.05, 'dz': 0.05},
]


class ScenePublisher(Node):
    """Publishes the table and known blocks to the MoveIt planning scene."""

    def __init__(self):
        super().__init__('scene_publisher')

        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').value

        latched_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', latched_qos)

        self._published = False
        self.create_timer(3.0, self._publish_once)

    def _publish_once(self):
        if self._published:
            return
        self._published = True
        self._publish_table()
        self._publish_blocks()

    # ------------------------------------------------------------------ table

    def _publish_table(self):
        scene = PlanningScene(is_diff=True)

        table = CollisionObject()
        table.header.frame_id = self.frame_id
        table.header.stamp = self.get_clock().now().to_msg()
        table.id = 'table'
        table.operation = CollisionObject.ADD

        table_top = SolidPrimitive(type=SolidPrimitive.BOX,
                                   dimensions=[0.8, 1.0, 0.04])
        table_top_pose = Pose(
            position=Point(x=0.6, y=0.0, z=0.25),
            orientation=Quaternion(w=1.0),
        )
        table.primitives.append(table_top)
        table.primitive_poses.append(table_top_pose)

        for x, y, z in [(0.95, 0.45, 0.115), (0.95, -0.45, 0.115),
                        (0.25, 0.45, 0.115), (0.25, -0.45, 0.115)]:
            leg = SolidPrimitive(type=SolidPrimitive.BOX,
                                 dimensions=[0.04, 0.04, 0.23])
            leg_pose = Pose(position=Point(x=x, y=y, z=z),
                            orientation=Quaternion(w=1.0))
            table.primitives.append(leg)
            table.primitive_poses.append(leg_pose)

        scene.world.collision_objects.append(table)
        self.scene_pub.publish(scene)
        self.get_logger().info('Published table to planning scene')

    # ----------------------------------------------------------------- blocks

    def _publish_blocks(self):
        for obj in BLOCKS:
            co = self._make_block(obj)

            # Notify the behavior tree (latched)
            self.collision_pub.publish(co)

            # Add to MoveIt planning scene
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects.append(co)
            self.scene_pub.publish(scene)

            self.get_logger().info(
                f'Published "{obj["id"]}" at '
                f'({obj["x"]:.3f}, {obj["y"]:.3f}, {obj["z"]:.3f})'
            )

    def _make_block(self, obj: dict) -> CollisionObject:
        co = CollisionObject()
        co.header.frame_id = self.frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj['id']
        co.operation = CollisionObject.ADD

        prim = SolidPrimitive(type=SolidPrimitive.BOX,
                              dimensions=[obj['dx'], obj['dy'], obj['dz']])
        pose = Pose(
            position=Point(x=obj['x'], y=obj['y'], z=obj['z']),
            orientation=Quaternion(w=1.0),
        )
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        return co


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
