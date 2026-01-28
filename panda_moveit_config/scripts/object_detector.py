#!/usr/bin/env python3
"""
Lab 02: Object Detection from Point Cloud

This node subscribes to point cloud data from an overhead RGBD camera,
detects objects on the table, and publishes them to the MoveIt planning scene.

Students must implement the TODO sections.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene

# Helper to convert PointCloud2 to numpy array
from sensor_msgs_py import point_cloud2


class ObjectDetector(Node):
    """Detects objects from point cloud and publishes to planning scene."""

    def __init__(self):
        super().__init__('object_detector')

        # Publisher for the planning scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Store detected objects for the grasp planner to access
        self.detected_objects = []

        # TODO 1: Create a subscriber to the point cloud topic
        # Hint: The topic name depends on how you configured gz_bridge.yaml
        # The message type is sensor_msgs/msg/PointCloud2
        # The callback should be self.point_cloud_callback
        #
        # self.pc_sub = self.create_subscription(...)

        self.get_logger().info('Object detector initialized')

    def point_cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud data."""

        # Convert PointCloud2 to numpy array of (x, y, z) points
        points = self.pointcloud2_to_xyz(msg)

        if points is None or len(points) == 0:
            return

        # TODO 2: Filter the point cloud
        # Remove points that are:
        # - Too far from the camera (z > some threshold in camera frame)
        # - Part of the table surface (you'll need to identify the table plane)
        # - Outside the region of interest
        #
        # Hint: The camera is at z=1.5 looking down. Points on the table
        # surface are at z≈0.42 in world frame. Objects are above that.
        #
        # filtered_points = ...

        # TODO 3: Cluster the remaining points into separate objects
        # You can use simple spatial clustering or more advanced methods
        # Each cluster represents a potential object
        #
        # clusters = self.cluster_points(filtered_points)

        # TODO 4: For each cluster, compute a bounding box
        # Determine the center position and dimensions of each object
        #
        # objects = []
        # for cluster in clusters:
        #     center, dimensions = self.compute_bounding_box(cluster)
        #     objects.append({'pose': center, 'dimensions': dimensions})

        # TODO 5: Store detected objects and publish to planning scene
        # self.detected_objects = objects
        # self.publish_objects(objects)
        pass

    def pointcloud2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array of XYZ points."""
        try:
            points_list = list(point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ))

            if not points_list:
                return None

            return np.array(points_list)
        except Exception as e:
            self.get_logger().error(f'Error converting point cloud: {e}')
            return None

    def cluster_points(self, points: np.ndarray, distance_threshold: float = 0.05) -> list:
        """
        Cluster points into separate objects.

        Args:
            points: Nx3 numpy array of (x, y, z) points
            distance_threshold: Maximum distance between points in same cluster

        Returns:
            List of numpy arrays, each containing points for one cluster
        """
        # TODO: Implement clustering algorithm
        # Hint: You can use scipy.cluster.hierarchy.fclusterdata
        #
        # from scipy.cluster.hierarchy import fclusterdata
        # labels = fclusterdata(points, t=distance_threshold, criterion='distance')
        # clusters = [points[labels == i] for i in np.unique(labels)]
        raise NotImplementedError("Implement clustering in TODO 3")

    def compute_bounding_box(self, points: np.ndarray) -> tuple:
        """
        Compute axis-aligned bounding box for a cluster of points.

        Args:
            points: Nx3 numpy array of points belonging to one object

        Returns:
            Tuple of (center_pose, dimensions) where:
            - center_pose is a geometry_msgs/Pose
            - dimensions is [size_x, size_y, size_z]
        """
        # TODO: Implement bounding box computation
        # 1. Find min/max for x, y, z coordinates
        # 2. Center = (min + max) / 2 for each axis
        # 3. Dimensions = max - min for each axis
        raise NotImplementedError("Implement bounding box in TODO 4")

    def publish_objects(self, objects: list):
        """Publish detected objects to the MoveIt planning scene."""
        scene = PlanningScene()
        scene.is_diff = True

        for i, obj in enumerate(objects):
            collision_obj = CollisionObject()
            collision_obj.id = f'detected_object_{i}'
            collision_obj.header.frame_id = 'world'
            collision_obj.header.stamp = self.get_clock().now().to_msg()
            collision_obj.operation = CollisionObject.ADD

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(obj['dimensions'])

            collision_obj.primitives.append(box)
            collision_obj.primitive_poses.append(obj['pose'])
            scene.world.collision_objects.append(collision_obj)

        self.scene_pub.publish(scene)
        self.get_logger().info(f'Published {len(objects)} detected objects')

    def get_detected_objects(self) -> list:
        """Return list of detected objects for grasp planning."""
        return self.detected_objects


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
