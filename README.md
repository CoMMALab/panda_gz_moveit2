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

| Alias | Command |
|-------|---------|
| `launch_ctrl` | `ros2 launch panda_moveit_config ex_gz_control.launch.py` |
| `build` | `colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"` |

## Labs

- [Lab 01](labs/LAB01.md) — MoveIt 2 motion planning
