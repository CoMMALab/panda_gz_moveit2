# panda_gz_moveit2

A ROS 2 (Jazzy) simulation environment for the Franka Emika Panda robot arm, integrating Gazebo for physics simulation, MoveIt 2 for motion planning, and an overhead RGBD camera for tabletop perception and grasp planning.

## Building and Running

Build the Docker image:

```bash
.docker/build.bash
```

Start the container:

```bash
.docker/run.bash
```

Launch the simulation (Gazebo + MoveIt 2 + RViz):

```bash
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

## Container Aliases

| Alias | Command | Description |
|-------|---------|-------------|
| `launch_ctrl` | `ros2 launch panda_moveit_config ex_gz_control.launch.py` | Launch Gazebo + MoveIt 2 + RViz |
| `launch_detector` | `ros2 run panda_moveit_config object_detector.py` | Run the tabletop object detector |
| `launch_planner` | `ros2 run panda_moveit_config grasp_planner.py` | Run the grasp planner |
| `build` | `colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"` | Rebuild the ROS workspace |

## Labs

- [Lab 01](labs/LAB01.md) — MoveIt 2 motion planning
