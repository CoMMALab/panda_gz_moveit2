# Lab 02: Perception and Grasping

## Overview

In this lab, you will extend the Panda robot simulation with perception and grasping capabilities. You will:

1. Add an RGBD camera sensor to the Gazebo simulation
2. Configure the ROS-Gazebo bridge to publish point cloud data
3. Implement object detection from point cloud data
4. Implement grasp planning using MoveIt's pick/place functionality

## Prerequisites

Complete Lab 01 and ensure your environment is working. Use the same Docker container.

## Submission Requirements

Submit a PDF document containing:

1. Screenshots of each deliverable as specified
2. Your completed `object_detector.py` code
3. Your completed `grasp_planner.py` code
4. Written answers to all questions

---

## Part 1: Adding the Camera Sensor (15 points)

### Step 1.1: Add an Overhead Camera

Open `panda_description/worlds/tabletop.sdf` and add the following camera model **before** the closing `</world>` tag:

```xml
<!-- Overhead RGBD Camera -->
<model name="overhead_camera">
  <static>true</static>
  <pose>0.5 0 1.5 0 1.5708 0</pose>
  <link name="camera_link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.05 0.1 0.05</size>
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.2 1</ambient>
        <diffuse>0.2 0.2 0.2 1</diffuse>
      </material>
    </visual>
    <sensor name="rgbd_camera" type="rgbd_camera">
      <update_rate>10</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <depth_camera>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </depth_camera>
      </camera>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <topic>overhead_camera</topic>
      <gz_frame_id>overhead_camera_link</gz_frame_id>
    </sensor>
  </link>
</model>
```

### Step 1.2: Understand the Camera Configuration

**Deliverable 1.1:** Answer these questions about the camera configuration:

1. What is the camera's position in world coordinates (x, y, z)?
2. The pose includes orientation `0 1.5708 0` (roll, pitch, yaw in radians). What direction is the camera pointing?
3. What is the camera's horizontal field of view in degrees? (Hint: convert from radians)

### Step 1.3: Verify Camera Topics

Launch the simulation and list Gazebo topics:

```shell
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

In another terminal:

```shell
gz topic -l | grep overhead
```

**Deliverable 1.2:** What Gazebo topics does the camera publish? List all topics containing "overhead".

---

## Part 2: Bridging Camera Data to ROS 2 (15 points)

### Step 2.1: Configure the Bridge

Open `panda_moveit_config/config/gz_bridge.yaml` and add an entry to bridge the point cloud topic.

The bridge entry format is:

```yaml
- ros_topic_name: <ros_topic_name>
  gz_topic_name: <gazebo_topic_name>
  ros_type_name: <ros_message_type>
  gz_type_name: <gazebo_message_type>
  direction: GZ_TO_ROS
```

For the point cloud:
- Gazebo topic: `/overhead_camera/points` (verify with `gz topic -l`)
- ROS message type: `sensor_msgs/msg/PointCloud2`
- Gazebo message type: `gz.msgs.PointCloudPacked`

**Deliverable 2.1:** Show your complete `gz_bridge.yaml` with the new entry.

### Step 2.2: Verify the Bridge

Relaunch the simulation and verify the topic appears in ROS 2:

```shell
ros2 topic list | grep -i point
ros2 topic info <your_topic>
```

**Deliverable 2.2:** Take a screenshot showing `ros2 topic info` output for your point cloud topic.

### Step 2.3: Visualize in RViz

In RViz, add a PointCloud2 display and set the topic to your bridged point cloud.

**Deliverable 2.3:** Take a screenshot of RViz showing the point cloud visualization with the table and objects visible.

---

## Part 3: Implementing Object Detection (35 points)

### Step 3.1: Understand the Skeleton

Open `panda_moveit_config/scripts/object_detector.py`. This skeleton has 5 TODOs:

| TODO | Description |
|------|-------------|
| 1 | Create subscriber to point cloud topic |
| 2 | Filter point cloud (remove table, floor, etc.) |
| 3 | Cluster points into separate objects |
| 4 | Compute bounding boxes for each cluster |
| 5 | Store and publish detected objects |

### Step 3.2: Implement TODO 1 - Subscriber

Create a subscription to your point cloud topic:

```python
self.pc_sub = self.create_subscription(
    PointCloud2,
    '/your_topic_name',  # Use the topic from Part 2
    self.point_cloud_callback,
    10
)
```

### Step 3.3: Implement TODO 2 - Filtering

Filter the point cloud to isolate objects:

1. Points come in the **camera frame** (camera at z=1.5, looking down)
2. Transform or threshold appropriately to:
   - Remove floor points (z ≈ 0 in world frame)
   - Remove table surface (z ≈ 0.42 in world frame)
   - Keep only points above the table (objects)

**Hint:** In camera frame, the table surface is approximately 1.08m from the camera.

### Step 3.4: Implement TODO 3 - Clustering

Use scipy's hierarchical clustering:

```python
from scipy.cluster.hierarchy import fclusterdata

def cluster_points(self, points, distance_threshold=0.05):
    if len(points) < 2:
        return [points]
    labels = fclusterdata(points, t=distance_threshold, criterion='distance')
    return [points[labels == i] for i in np.unique(labels)]
```

### Step 3.5: Implement TODO 4 - Bounding Boxes

Compute axis-aligned bounding boxes:

```python
def compute_bounding_box(self, points):
    min_pt = points.min(axis=0)
    max_pt = points.max(axis=0)

    center = Pose()
    center.position.x = (min_pt[0] + max_pt[0]) / 2
    center.position.y = (min_pt[1] + max_pt[1]) / 2
    center.position.z = (min_pt[2] + max_pt[2]) / 2
    center.orientation.w = 1.0

    dimensions = (max_pt - min_pt).tolist()
    return center, dimensions
```

### Step 3.6: Test Your Detector

Run the detector:

```shell
ros2 run panda_moveit_config object_detector.py
```

**Deliverable 3.1:** Take a screenshot of RViz showing detected objects appearing as collision boxes in the planning scene.

**Deliverable 3.2:** Include your completed `object_detector.py` in your submission.

---

## Part 4: Implementing Grasp Planning (35 points)

### Step 4.1: Understand the Grasp Planner

Open `panda_moveit_config/scripts/grasp_planner.py`. This skeleton has 7 TODOs:

| TODO | Description |
|------|-------------|
| 1 | Set grasp pose (end-effector target position) |
| 2 | Set pre-grasp approach direction |
| 3 | Set post-grasp retreat direction |
| 4 | Set place pose |
| 5 | Set pre-place approach direction |
| 6 | Set post-place retreat direction |
| 7 | Implement main pick-and-place logic |

### Step 4.2: Implement Grasp Pose (TODO 1)

For a top-down grasp, position the gripper above the object:

```python
grasp.grasp_pose.pose.position.x = object_pose.position.x
grasp.grasp_pose.pose.position.y = object_pose.position.y
grasp.grasp_pose.pose.position.z = object_pose.position.z + 0.1  # Above object

# Orientation: gripper pointing down
# Panda hand frame: z points forward (along fingers)
# For top-down: rotate 180° around x-axis
grasp.grasp_pose.pose.orientation.x = 1.0
grasp.grasp_pose.pose.orientation.y = 0.0
grasp.grasp_pose.pose.orientation.z = 0.0
grasp.grasp_pose.pose.orientation.w = 0.0
```

### Step 4.3: Implement Approach/Retreat (TODOs 2, 3)

```python
# Pre-grasp: approach from above (negative z)
grasp.pre_grasp_approach.direction.vector.z = -1.0
grasp.pre_grasp_approach.min_distance = 0.05
grasp.pre_grasp_approach.desired_distance = 0.1

# Post-grasp: retreat upward (positive z)
grasp.post_grasp_retreat.direction.vector.z = 1.0
grasp.post_grasp_retreat.min_distance = 0.05
grasp.post_grasp_retreat.desired_distance = 0.1
```

### Step 4.4: Implement Place Location (TODOs 4, 5, 6)

Similar to grasp, but for placing:

```python
place.place_pose.pose = place_pose

# Pre-place: approach from above
place.pre_place_approach.direction.vector.z = -1.0
place.pre_place_approach.min_distance = 0.05
place.pre_place_approach.desired_distance = 0.1

# Post-place: retreat upward
place.post_place_retreat.direction.vector.z = 1.0
place.post_place_retreat.min_distance = 0.05
place.post_place_retreat.desired_distance = 0.1
```

### Step 4.5: Implement Main Logic (TODO 7)

Create a simple test that picks up a detected object and places it elsewhere:

```python
# Example: Pick detected_object_0 and place at new location
object_pose = Pose()
object_pose.position.x = 0.4  # Get from detector
object_pose.position.y = 0.2
object_pose.position.z = 0.495

place_pose = Pose()
place_pose.position.x = 0.6
place_pose.position.y = 0.0
place_pose.position.z = 0.5

node.pick_and_place('detected_object_0', object_pose, place_pose)
```

### Step 4.6: Test Pick and Place

**Deliverable 4.1:** Take a sequence of screenshots showing:
1. Initial state with detected objects
2. Robot approaching an object
3. Robot grasping the object
4. Robot placing the object at a new location

**Deliverable 4.2:** Include your completed `grasp_planner.py` in your submission.

**Deliverable 4.3:** Answer these questions:
1. Why is the gripper orientation important for successful grasping?
2. What happens if the approach distance is too short? Too long?
3. How would you modify the grasp for a cylindrical object vs a box?

---

## Troubleshooting

### Point cloud not appearing
- Verify `gz_bridge.yaml` syntax
- Check Gazebo topic name matches exactly
- Ensure sensor is publishing: `gz topic -e -t /overhead_camera/points`

### Points in wrong frame
- Camera frame has origin at camera position
- Transform to world frame if needed, or adjust thresholds

### Pick/Place fails
- Check that object is in planning scene
- Verify grasp pose is reachable
- Try adjusting approach distances
- Check gripper orientation

### "No valid grasps found"
- Grasp pose may be in collision
- Try different approach angles
- Ensure gripper can reach the object

---

## Resources

- [MoveIt Pick and Place Tutorial](https://moveit.picknik.ai/main/doc/examples/pick_place/pick_place_tutorial.html)
- [sensor_msgs_py](https://docs.ros.org/en/rolling/p/sensor_msgs_py/)
- [scipy clustering](https://docs.scipy.org/doc/scipy/reference/cluster.hierarchy.html)
