# Assignment: MoveIt 2 Motion Planning with the Panda Robot

## Overview

In this assignment, you will use MoveIt 2 to plan and execute motions for a simulated Franka Emika Panda robot arm. You will learn how to:

1. Set up and run a ROS 2 simulation environment using Docker
2. Use MoveIt 2's RViz interface for interactive motion planning
3. Understand the relationship between planning groups, joint states, and motion trajectories
4. Execute planned motions on a simulated robot

### For NVIDIA GPU Users (Optional but Recommended)

If you have an NVIDIA GPU, install the NVIDIA Container Toolkit for hardware-accelerated rendering:

```bash
# Add the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Part 1: Environment Setup (10 points)

### Step 1.1: Clone and Build

Clone this repository and build the Docker image:

```bash
git clone <repository-url> panda_gz_moveit2
cd panda_gz_moveit2
.docker/build.bash
```

### Step 1.2: Run the Container

Start the Docker container:

```bash
.docker/run.bash
```

You should now be inside the container at `/root/ws`.

### Step 1.3: Build the Workspace

Build the ROS 2 workspace inside the container:

```bash
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.bash
```

## Part 2: Running the Motion Planning Demo (15 points)

### Step 2.1: Launch the Planning Interface

Launch the fake controller demo (no physics simulation, instant motion execution):

```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

This will open RViz with the MoveIt Motion Planning plugin.

### Step 2.2: Explore the Interface

In RViz, locate the "MotionPlanning" panel on the left side. You should see:

- **Planning Group:** Currently set to "arm" (the 7-DOF manipulator)
- **Start State:** The current robot configuration (shown in green)
- **Goal State:** The target configuration (shown in orange)

### Step 2.3: Understanding Planning Groups

The Panda robot has two planning groups defined:

1. **arm** - The 7 joints of the manipulator (panda_joint1 through panda_joint7)
2. **gripper** - The 2 finger joints (panda_finger_joint1, panda_finger_joint2)

Use the dropdown in the MotionPlanning panel to switch between groups.

**Deliverable 2.2:** Answer the following questions:
- What is the end-effector link for the arm group? (Hint: check the SRDF file or the TF tree in RViz)

## Part 3: Interactive Motion Planning (25 points)

### Step 3.1: Set a Goal Using the Interactive Marker

1. In RViz, you should see an interactive marker (colored rings and arrows) at the robot's end-effector
2. Drag the marker to move the goal pose
3. The orange "ghost" robot shows where the arm will move to

### Step 3.2: Plan a Motion

1. With a goal pose set, click the **"Plan"** button in the MotionPlanning panel
2. Observe the planned trajectory animation
3. The trajectory shows how the robot will move from start to goal

**Deliverable 3.1:**
- Plan a motion that moves the end-effector of the robot.
- Take a screenshot showing the planned trajectory (the animated path).
- Record the planning time displayed in the panel.

### Step 3.3: Execute the Motion

1. Click **"Execute"** to run the planned trajectory
2. The robot (green) will move to match the goal (orange)
3. Alternatively, use **"Plan & Execute"** to do both steps at once

**Deliverable 3.2:** Take a screenshot showing the robot in its new position after execution.

### Step 3.4: Use Predefined Poses

The SRDF defines several named poses:

- **ready** - A neutral starting pose
- **extended** - Arm fully extended
- **transport** - Compact pose for transport

1. In the MotionPlanning panel, find the "Goal State" dropdown
2. Select one of the predefined poses
3. Plan and execute the motion

**Deliverable 3.3:**
- Define a new named pose in the SRDF of the robot.
- Execute motions to visit all three predefined poses and your new pose (ready → extended → transport → your new pose → ready)
- For each transition, record:
  - Planning time
  - Whether the plan succeeded on the first attempt

## Part 4: Understanding Motion Planning (25 points)

### Step 4.1: Examine the Planning Pipeline

The motion planner uses OMPL (Open Motion Planning Library) with the RRTConnect algorithm by default.

**Deliverable 4.1:** Answer the following questions based on your observations and the configuration files:

1. What planner is configured as the default? (Hint: check `ompl_planning.yaml`)
2. What is the difference between "Plan" failing vs "Execute" failing?

### Step 4.2: Collision Awareness

1. Launch the Gazebo simulation: `ros2 launch panda_moveit_config ex_gz_control.launch.py`
2. In Gazebo, drag one of the tabletop objects (red box or blue cylinder) to a new position
3. Observe that RViz's planning scene updates automatically
4. Position an object between the robot and a goal pose
5. Attempt to plan a motion through/around the obstacle

**Deliverable 4.2:**
- Take a screenshot showing a planned trajectory that avoids the obstacle.
- What happens if you place the obstacle directly on the goal pose?

### Step 4.3: Joint Space vs Cartesian Space

1. In the MotionPlanning panel, find the "Planning" tab
2. Experiment with the "Use Cartesian Path" option.

**Deliverable 4.3:** Compare results from at least 2 different planners:
- Which planner produced shorter paths?
- Which planner was faster?

## Part 5: Gazebo Simulation (25 points)

### Step 5.1: Launch Gazebo Simulation

```bash
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

This launches:
- Gazebo simulator with the Panda robot and a tabletop scene
- Two objects on the table: a red box and a blue cylinder
- MoveIt 2 move_group node
- RViz for motion planning

### Step 5.2: Physics-Based Execution

1. Plan and execute motions as before
2. Observe the difference - motions now follow physics (gravity, inertia, motor limits)

**Deliverable 5.1:**
- Compare execution of the same trajectory in fake control vs Gazebo. What differences do you observe?
- Why might a trajectory succeed in fake control but have issues in Gazebo?


## Submission Requirements

Submit a PDF document containing:

1. All screenshots as specified in the deliverables.
2. Written answers to all questions.
3. A brief reflection (1 paragraph) on what you learned.

## Troubleshooting

### Docker/GPU issues
- Without NVIDIA toolkit, Gazebo will use software rendering (slower)
- If GUI doesn't appear, ensure X11 forwarding is working: `xhost +local:docker`

## Resources

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [OMPL Planners](https://ompl.kavrakilab.org/planners.html)
