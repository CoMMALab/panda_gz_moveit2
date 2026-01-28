#!/usr/bin/env python3
"""
Lab 02: Grasp Planning with MoveIt Pick/Place

This node uses MoveIt's pick/place functionality to grasp detected objects.

Students must implement the TODO sections.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import (
    Grasp, GripperTranslation, PlaceLocation,
    MoveItErrorCodes, PlanningScene, CollisionObject
)
from moveit_msgs.action import MoveGroup, Pickup, Place
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GraspPlanner(Node):
    """Plans and executes pick/place operations using MoveIt."""

    def __init__(self):
        super().__init__('grasp_planner')

        # Action clients for MoveIt
        self.pickup_client = ActionClient(self, Pickup, '/pickup')
        self.place_client = ActionClient(self, Place, '/place')

        # Planning scene publisher (for attaching/detaching objects)
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Gripper joint names for Panda
        self.gripper_joints = ['panda_finger_joint1', 'panda_finger_joint2']
        self.gripper_open_position = [0.04, 0.04]   # Open gripper
        self.gripper_closed_position = [0.0, 0.0]   # Closed gripper

        # Wait for action servers
        self.get_logger().info('Waiting for MoveIt action servers...')
        self.pickup_client.wait_for_server()
        self.place_client.wait_for_server()
        self.get_logger().info('Grasp planner ready')

    def create_grasp(self, object_pose: Pose, object_id: str) -> Grasp:
        """
        Create a Grasp message for picking up an object.

        Args:
            object_pose: Pose of the object to grasp
            object_id: ID of the object in the planning scene

        Returns:
            A configured Grasp message
        """
        grasp = Grasp()
        grasp.id = f'grasp_{object_id}'

        # TODO 1: Set the grasp pose
        # The grasp pose is where the end-effector should be positioned to grasp.
        # For a top-down grasp, the gripper should approach from above.
        #
        # Hint: The grasp pose should be slightly above the object,
        # with the gripper oriented to point downward.
        # The Panda's end-effector frame (panda_hand) has z pointing forward.
        #
        grasp.grasp_pose = PoseStamped()
        grasp.grasp_pose.header.frame_id = 'world'
        # grasp.grasp_pose.pose.position = Point(...)
        # grasp.grasp_pose.pose.orientation = Quaternion(...)

        # TODO 2: Set the pre-grasp approach
        # This defines how the gripper approaches the object before grasping.
        #
        grasp.pre_grasp_approach = GripperTranslation()
        grasp.pre_grasp_approach.direction.header.frame_id = 'world'
        # grasp.pre_grasp_approach.direction.vector.x = ...
        # grasp.pre_grasp_approach.direction.vector.y = ...
        # grasp.pre_grasp_approach.direction.vector.z = ...  # -1 for downward
        # grasp.pre_grasp_approach.min_distance = 0.05
        # grasp.pre_grasp_approach.desired_distance = 0.1

        # TODO 3: Set the post-grasp retreat
        # This defines how the gripper retreats after grasping (typically upward).
        #
        grasp.post_grasp_retreat = GripperTranslation()
        grasp.post_grasp_retreat.direction.header.frame_id = 'world'
        # grasp.post_grasp_retreat.direction.vector.z = 1  # Upward
        # grasp.post_grasp_retreat.min_distance = 0.05
        # grasp.post_grasp_retreat.desired_distance = 0.1

        # Pre-grasp posture (open gripper)
        grasp.pre_grasp_posture = self.create_gripper_trajectory(open=True)

        # Grasp posture (closed gripper)
        grasp.grasp_posture = self.create_gripper_trajectory(open=False)

        return grasp

    def create_gripper_trajectory(self, open: bool) -> JointTrajectory:
        """Create a trajectory for opening or closing the gripper."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = self.gripper_open_position if open else self.gripper_closed_position
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory.points.append(point)
        return trajectory

    def create_place_location(self, place_pose: Pose) -> PlaceLocation:
        """
        Create a PlaceLocation message for placing an object.

        Args:
            place_pose: Where to place the object

        Returns:
            A configured PlaceLocation message
        """
        place = PlaceLocation()
        place.id = 'place_location'

        # TODO 4: Set the place pose
        place.place_pose = PoseStamped()
        place.place_pose.header.frame_id = 'world'
        # place.place_pose.pose = place_pose

        # TODO 5: Set the pre-place approach (typically downward)
        place.pre_place_approach = GripperTranslation()
        place.pre_place_approach.direction.header.frame_id = 'world'
        # place.pre_place_approach.direction.vector.z = -1
        # place.pre_place_approach.min_distance = 0.05
        # place.pre_place_approach.desired_distance = 0.1

        # TODO 6: Set the post-place retreat (typically upward)
        place.post_place_retreat = GripperTranslation()
        place.post_place_retreat.direction.header.frame_id = 'world'
        # place.post_place_retreat.direction.vector.z = 1
        # place.post_place_retreat.min_distance = 0.05
        # place.post_place_retreat.desired_distance = 0.1

        # Post-place posture (open gripper to release)
        place.post_place_posture = self.create_gripper_trajectory(open=True)

        return place

    def pick(self, object_id: str, object_pose: Pose) -> bool:
        """
        Execute a pick operation.

        Args:
            object_id: ID of the object in the planning scene
            object_pose: Current pose of the object

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f'Attempting to pick {object_id}')

        goal = Pickup.Goal()
        goal.target_name = object_id
        goal.group_name = 'arm'
        goal.end_effector = 'gripper'
        goal.allowed_planning_time = 10.0

        # Create grasp
        grasp = self.create_grasp(object_pose, object_id)
        goal.possible_grasps.append(grasp)

        # Send goal
        future = self.pickup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pick goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Successfully picked {object_id}')
            return True
        else:
            self.get_logger().error(f'Pick failed with error code: {result.error_code.val}')
            return False

    def place(self, object_id: str, place_pose: Pose) -> bool:
        """
        Execute a place operation.

        Args:
            object_id: ID of the object being held
            place_pose: Where to place the object

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f'Attempting to place {object_id}')

        goal = Place.Goal()
        goal.attached_object_name = object_id
        goal.group_name = 'arm'
        goal.allowed_planning_time = 10.0

        # Create place location
        place_location = self.create_place_location(place_pose)
        goal.place_locations.append(place_location)

        # Send goal
        future = self.place_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Place goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Successfully placed {object_id}')
            return True
        else:
            self.get_logger().error(f'Place failed with error code: {result.error_code.val}')
            return False

    def pick_and_place(self, object_id: str, object_pose: Pose, place_pose: Pose) -> bool:
        """
        Execute a complete pick and place operation.

        Args:
            object_id: ID of the object to pick
            object_pose: Current pose of the object
            place_pose: Where to place the object

        Returns:
            True if both pick and place succeeded
        """
        if not self.pick(object_id, object_pose):
            return False

        return self.place(object_id, place_pose)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()

    # Example usage: pick detected_object_0 and place it at a new location
    # You would typically get the object pose from the object detector

    # TODO 7: Implement the main logic
    # 1. Wait for objects to be detected (subscribe to planning scene or
    #    communicate with object_detector node)
    # 2. Select an object to pick
    # 3. Define a place location
    # 4. Call pick_and_place

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
