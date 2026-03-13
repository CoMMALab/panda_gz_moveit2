#!/usr/bin/env python3
"""
Synthetic point cloud publisher for students without GPU rendering support.

Publishes a simulated tabletop scene to /point_cloud in the overhead_camera_link
frame so that object_detector.py can be tested without a working Gazebo RGBD camera.
All four TODOs in object_detector.py are exercised as normal.

Usage (in a separate terminal while the simulation is running):
    ros2 run panda_moveit_config synthetic_point_cloud.py
"""

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import tf_transformations

# ── Known scene geometry (must match tabletop.sdf) ────────────────────────────
CAMERA_POS  = np.array([0.6,   0.0,  1.05])   # world-frame xyz
CAMERA_RPY  = np.array([0.0,   1.5708, 0.0])  # roll pitch yaw (rad)

TABLE_CENTER = np.array([0.6,   0.0,  0.27])   # top-face centre, world frame
TABLE_DIMS   = np.array([0.8,   1.0,  0.0])    # treated as a flat plane

BOX_CENTER   = np.array([0.55, -0.1,  0.375])  # world frame
BOX_DIMS     = np.array([0.06,  0.06, 0.21])   # metres
# ──────────────────────────────────────────────────────────────────────────────

NOISE_STD = 0.002   # metres — matches typical RGBD sensor noise


def _sample_plane(center, dims, n: int) -> np.ndarray:
    """Sample n points uniformly on a horizontal plane."""
    x = np.random.uniform(center[0] - dims[0] / 2, center[0] + dims[0] / 2, n)
    y = np.random.uniform(center[1] - dims[1] / 2, center[1] + dims[1] / 2, n)
    z = np.full(n, center[2])
    return np.column_stack([x, y, z]).astype(np.float32)


def _sample_box_surface(center, dims, n: int) -> np.ndarray:
    """Sample n points uniformly over all six faces of an axis-aligned box."""
    cx, cy, cz = center
    hx, hy, hz = dims / 2
    n6 = max(1, n // 6)
    faces = []
    for axis, sign in [(0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)]:
        xs = np.random.uniform(cx - hx, cx + hx, n6)
        ys = np.random.uniform(cy - hy, cy + hy, n6)
        zs = np.random.uniform(cz - hz, cz + hz, n6)
        if   axis == 0: xs = np.full(n6, cx + sign * hx)
        elif axis == 1: ys = np.full(n6, cy + sign * hy)
        else:           zs = np.full(n6, cz + sign * hz)
        faces.append(np.column_stack([xs, ys, zs]))
    return np.vstack(faces).astype(np.float32)


def _world_to_camera(pts: np.ndarray, pos: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """Transform (N,3) world-frame points into the camera frame."""
    T = tf_transformations.euler_matrix(*rpy, axes='sxyz')
    T[:3, 3] = pos
    T_inv = np.linalg.inv(T)
    ones  = np.ones((len(pts), 1), dtype=np.float32)
    pts_h = np.hstack([pts, ones])           # (N, 4)
    return (T_inv @ pts_h.T).T[:, :3].astype(np.float32)


class SyntheticPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_point_cloud')
        self._pub   = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self._timer = self.create_timer(0.5, self._publish)   # 2 Hz
        self.get_logger().info(
            'Synthetic point cloud publisher running → /point_cloud '
            '(frame: overhead_camera_link, 2 Hz)'
        )

    def _publish(self):
        table_pts = _sample_plane(TABLE_CENTER, TABLE_DIMS, n=2000)
        box_pts   = _sample_box_surface(BOX_CENTER, BOX_DIMS, n=1000)
        pts_world = np.vstack([table_pts, box_pts])

        # Add sensor-like noise
        pts_world += np.random.normal(0.0, NOISE_STD, pts_world.shape).astype(np.float32)

        pts_cam = _world_to_camera(pts_world, CAMERA_POS, CAMERA_RPY)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'overhead_camera_link'

        self._pub.publish(pc2.create_cloud_xyz32(header, pts_cam.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
