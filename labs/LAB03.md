# LAB 03 — Behavior Tree Pick-and-Place

## Overview

You have just joined the autonomy team at ROBOINC. The hardware is ready, the simulation environment is set up, and the component libraries are integrated. What remains is the behavior: a system that can reliably pick objects off a table and sort them into a container, repeatedly, without human intervention.

This is a realistic situation. The spec is intentionally high-level. The infrastructure has rough edges. Some things will not work the first time. Your job is to make engineering decisions, implement the missing pieces, and deliver something that runs end-to-end.

The core abstraction is a behavior tree, a structured way to compose robot behaviors that is widely used in industry precisely because it handles failure gracefully. You will wire the tree, implement the planning nodes, and think carefully about what "reliable" means when the robot operates in an uncertain environment.

> **Tip:** The container includes the following aliases:
>
> | Alias | Description |
> |-------|-------------|
> | `launch_ctrl` | Launch Gazebo + MoveIt 2 + RViz |
> | `launch_bt` | Run the behavior tree |
> | `build` | Rebuild the ROS workspace |

## Behavior Trees

This lab uses py-trees for behavior tree execution and GPD for grasp candidate generation:
- https://py-trees.readthedocs.io/en/devel/introduction.html
- https://github.com/atenpas/gpd

Every node in the tree returns one of three statuses when ticked:

| Status | Meaning |
|--------|---------|
| `SUCCESS` | The node completed its goal |
| `FAILURE` | The node could not complete its goal |
| `RUNNING` | The node is still working |

A **Sequence** ticks its children left to right and returns `SUCCESS` only if all of them succeed. The moment one returns `FAILURE`, the Sequence stops and propagates `FAILURE` upward — equivalent to a chain of `and` conditions. The `RepeatAlways` decorator wraps the root Sequence and re-ticks it on both `SUCCESS` and `FAILURE`, creating the outer loop.

---

## Provided Infrastructure

The following are provided under `panda_gz_moveit2/panda_moveit_config/scripts` and should not be modified:

- `robot_interface.py`:
  - **`RobotInterface`** — ROS 2 node wrapping MoveIt, the gripper controller, and the Gazebo pose bridge
  - **`MoveToHome`** — resets the system on every entry: opens the gripper, detaches all objects from the robot, and moves the arm to the home configuration
  - **`ReadScene`** — waits for all object poses to arrive and stabilize, then adds them to the MoveIt planning scene
  - **`CheckAllObjectsInContainer`** — polls until the current object has settled inside the container bounds, then checks whether all objects are sorted. Returns FAILURE to trigger a restart for the next object, and SUCCESS only when all objects are confirmed placed. This node **must** be the last in your behavior tree.
  - **Primitive Nodes** - Several other provided behaviors at the top of `bt_pick_place.py` that you should read and use in your tree.
- **`gpd.py`** — thin wrapper around the GPD grasp planner library. Handles point cloud I/O and coordinate frame conversion, but not filtering

## Deliverables

**Due: April 10th, 11:59PM**

Submit a single zip file containing:

1. Your written report (including your full name)
2. Your `bt_pick_place.py` file with implementations of: `build_tree()`, `SelectObject`, `ProposeGrasps`, and `ProposeDropPose`.

| Part | Points | Deliverable |
|------|--------|-------------|
| (1) Tree Structure | 20 | `build_tree()` and `SelectObject.update()` in `bt_pick_place.py`; written answers to the Part 1 questions |
| (2) Invariants | 20 | Written invariant analysis for each provided node (one paragraph each) |
| (3) Grasp Proposal | 30 | `ProposeGrasps` in `bt_pick_place.py`; screenshot of the RViz point cloud and grasp markers during a pick; written answers to the Part 3 questions |
| (4) Drop Pose | 30 | `ProposeDropPose` in `bt_pick_place.py`; one paragraph on your drop pose policy and any failure modes observed |

**Point breakdown:** Part 1 (20) + Part 2 (20) + Part 3 (30) + Part 4 (30) = **100 points**

---

## Part 1 — Tree Structure

The skeleton provides the individual behavior nodes but leaves the tree wiring incomplete. Check the file `bt_pick_place.py`.

**Task:** Complete `build_tree()` so that the robot executes the following loop:

1. **Reset** — move to home, read the scene, select the next unsorted object
2. **Grasp** — propose grasp candidates, filter, execute pick sequence
3. **Place** — move to drop zone, release, confirm object is in container

The tree should restart from the beginning on any failure, and terminate cleanly when all objects are sorted. The built tree should use all the nodes in the file.

**Questions to consider:**
- What composite type (Sequence vs Selector) is appropriate for each block?
- Where does the restart-on-failure logic live, and how does it interact with the tree's success condition?
- Trace a failure in the Grasp block: what state is the robot in when it fails? What does `MoveToHome` need to do to recover?

---

## Part 2 — Promises and Proxies

Each node in the tree makes an implicit promise: if it returns SUCCESS, some condition holds that the rest of the tree can rely on. The motion planner is only as reliable as those promises.

Some promises are exact. Others are proxies — the node checks something *related* to the condition it is trying to guarantee, not the condition itself. Proxies can lie. When they do, downstream nodes fail silently, the robot continues on a false assumption, and the failure mode can be very difficult to diagnose.

**Task:** For each provided node, write one paragraph covering:
- What the node is promising on SUCCESS
- What downstream behavior depends on that promise
- Under what conditions the promise could be false, and what the robot would do next.

Nodes to analyze: `MoveToHome`, `ReadScene`, `CheckObjectIsAttached`, `CheckAllObjectsInContainer`

When `ReadScene` succeeds, it writes a snapshot of the scene to the blackboard as `detected_objects` — a `dict[str, DetectedObject]`. Each entry bundles the pose and dimensions of one object:

```python
obj = self.bb.detected_objects['blue_box']
obj.pose   # geometry_msgs/Pose — world-frame pose at the time ReadScene ran
obj.dims   # [dx, dy, dz] in metres
```

This snapshot is frozen. It reflects the scene at the moment `ReadScene` returned SUCCESS, not the live Gazebo state.

---

## Part 3 — Grasp Pose Proposal

`SelectObject` and `ProposeGrasps` are the core planning nodes.
Implement `SelectObject`, it should write the `/target_object_id` to the blackboard and then return SUCCESS, or FAILURE if no object is reachable.
Next, implement `ProposeGrasps` using GPD, provided in `gpd.py`. It should write the `/grasp_proposals` to the blackboard and then return SUCCESS, or FAILURE if no proposals are written.
Grasp candidates are defined as a data class, returned by `detect_grasps`.

```python
@dataclass
class GraspCandidate:
    pos:   np.ndarray  # (3,) grasp point in world frame
    R:     np.ndarray  # (3,3) rotation matrix (columns are unit vectors in world frame)
    score: float       # higher is better, already sorted descending
```

`R` is a special geometric convention used by GPD, encoding the hand.
It has three columns:

| Column | Meaning |
|--------|---------|
| `R[:, 0]` | approach direction — the vector the hand travels along to reach the object |
| `R[:, 1]` | binormal — perpendicular to the other two |
| `R[:, 2]` | hand axis — the direction the fingers close along |

GPD was trained on general grippers and produces candidates in all orientations. Not all are reachable by the Panda. The motion planner targets the TCP (Tool Center Point) — the coordinate frame fixed at the tip of the gripper. `gpd_to_panda_pose` converts a candidate into a TCP pose the planner can consume, but it cannot fix a candidate that is geometrically infeasible to begin with. 

**Task:** Complete the `ProposeGrasps` node. Sample the object surface, run GPD, and filter the candidates down to poses the robot can actually execute — not every candidate GPD produces is reachable. Think about what the approach direction implies for the required joint configuration, and what clearance is needed above the table. If no valid candidates remain after retrying, return `FAILURE`.

**Questions to consider:**
- The score field is already sorted descending; why might the highest-scored candidate still be rejected by your filter?
- The Panda is a table-mounted arm. What approach directions are physically impossible regardless of IK?

---

## Part 4 — Drop Pose Selection

**Task:** Implement `ProposeDropPose` such that the robot chooses a pose within the depth and width of the container and maintains a sufficient clearance above the walls. It should write `/drop_pose` to the blackboard and return SUCCESS after. Finally, execute `CheckAllObjectsInContainer`, which will verify that your implementation successfully organizes all the objects in the container.

**Questions to consider:**
- How would you avoid special-casing the drop-pose based on the specific parameters of the container?
- The drop height is not given directly. Which container fields do you need to combine to derive it, and what does each contribute?

---

## API Reference

### Blackboard

All blackboard state is accessed through `self.bb` after registering the relevant keys.

```python
self.bb.detected_objects   # dict[str, DetectedObject] — snapshot written by ReadScene
self.bb.container          # dict — container geometry (read-only, set at startup):
                           #   'center_xy' : (x, y)  — centre in world frame
                           #   'width'     : float   — inner x dimension
                           #   'depth'     : float   — inner y dimension
                           #   'height'    : float   — wall height above table surface
                           #   'table_z'   : float   — table surface z
self.bb.target_object_id   # str  — written by SelectObject
self.bb.grasp_proposals    # list[Pose] — written by ProposeGrasps
self.bb.drop_pose          # Pose — written by ProposeDropPose
```

### RobotInterface

```python
self.robot.log(msg: str)
# Append to the on-screen log panel and ROS logger.

self.robot.publish_pose_axes(pose: Pose, label: str, scale: float = 0.1)
# Visualise XYZ axes at pose in RViz (x=red, y=green, z=blue).

self.robot.publish_cloud(points: np.ndarray)
# Publish an (N, 3) float32 array as PointCloud2 on /scan_cloud.

RobotInterface.TOP_DOWN_ORIENTATION: Quaternion
# Gripper pointing straight down (180° rotation about x-axis).
```

### gpd.py

```python
from gpd import sample_cuboid_surface, detect_grasps, GraspCandidate

sample_cuboid_surface(
    center: tuple,       # (x, y, z) world frame
    dims: list,          # [dx, dy, dz]
    n_points: int,
    orientation: tuple,  # (qx, qy, qz, qw) object orientation
) -> np.ndarray          # (N, 3) float32 point cloud

detect_grasps(cloud: np.ndarray) -> list[GraspCandidate]
# Run GPD on a point cloud. Returns candidates sorted by score (best first)
# Each candidate has .pos (world frame), .R (rotation matrix), .score

gpd_to_panda_pose(candidate.pos, candidate.R) -> Pose
# Call this function to get a Pose for the motion planner from a GPD candidate
```

---
