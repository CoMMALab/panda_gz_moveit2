# Lab 03: Behavior Trees for Robot Manipulation

## Overview

In Lab 02, you implemented pick-and-place as a monolithic `pick()` function.
When something went wrong (the arm clipped the object, the gripper didn't
close properly) the planner found no path and the function returned `False`,
stopping execution. There was no recovery, no retry, and no way to tell from the
outside which step had failed.

In this lab you will decompose that same sequence into a **Behavior Tree**,
making every step an explicit node with a clear SUCCESS or FAILURE status,
and explore how the engineering trees extend the ability of the planner to work
autonomously across multiple steps while bounding how much errors accumulate.

**Learning objectives:**

- Understand BT node types, status semantics, and tick-based execution
- Recognize how the BT structure maps to the LAB02 `pick()` sequence
- Implement grasp candidate generation, execution, and placement as BT nodes
- Wire recovery behavior (Retry, Sequence) and justify the structural choices

## Prerequisites

Complete Lab 02. Use the same Docker container.

> **Tip:** The container includes the following aliases:
>
> | Alias | Description |
> |-------|-------------|
> | `launch_ctrl` | Launch Gazebo + MoveIt 2 + RViz |
> | `launch_planner` | Run the grasp planner |
> | `build` | Rebuild the ROS workspace |

## Submission Requirements

Submit a PDF document containing:

1. Screenshots of each deliverable as specified
2. Written answers to all questions

And a ZIP file containing:

3. Your completed `behavior_tree.py`

**Point breakdown:** Part 1 (15) + Part 2 (20) + Part 3 (25) + Part 4 (20) + Part 5 (20) = **100 points**

---

## Part 1: Introduction to Behavior Trees (15 points)

### What went wrong in Lab 02

Open `grasp_planner.py` from Lab 02 and find the `pick()` method. It follows
this structure:

```python
def pick(self, object_id, object_pose):
    if not self.move_gripper(open=True):
        return False
    if not self.move_arm_to_pose(pre_grasp):
        return False
    if not self.move_arm_to_pose(grasp_pose, ...):
        return False
    if not self.move_gripper(open=False):
        return False
    self.attach_object(object_id)
    ...
    return True
```

Every step can fail, but failure always means the same thing: stop and return
`False`. There is no retry, no fallback, and no way to know which step failed
without reading the logs. Run the grasp planner a few times and observe how
often it fails silently.

**Deliverable 1.1:** Run `launch_planner` three times. For each run, note
whether pick succeeded or failed, and if it failed, which step produced the
last log message before failure.

### Behavior Tree concepts

A Behavior Tree is a tree of nodes, each of which returns one of three statuses
when *ticked*:

| Status | Meaning |
|--------|---------|
| `SUCCESS` | The node completed its goal |
| `FAILURE` | The node could not complete its goal |
| `RUNNING` | The node is still working (async) |

There are two composite node types used in this lab:

**Sequence** — ticks children left to right. Returns `SUCCESS` only if *all*
children succeed. Returns `FAILURE` as soon as one child fails (short-circuit).
Equivalent to logical AND.

**Selector** — ticks children left to right. Returns `SUCCESS` as soon as one
child succeeds. Returns `FAILURE` only if *all* children fail.
Equivalent to logical OR.

The **Retry** decorator wraps a child and re-ticks it on failure, up to N times.

The **Blackboard** is a shared key-value store that nodes use to pass data
between each other without direct coupling.

### The Lab 02 pick() sequence as a BT

The `pick()` method maps directly onto a Sequence node:

```
Sequence (Grasp)
├── OpenGripper
├── MoveToPreGrasp
├── MoveToGrasp
├── CloseGripper
└── AttachObject
```

Each step is a leaf node. If `MoveToGrasp` returns FAILURE, the Sequence stops, exactly like `return False`. Instead, we will retry the grasp up to `N` times, and if it's still failing we fall-back to the top-down retry.

```
Retry(N)
└── Sequence (Grasp)
    ├── OpenGripper
    ├── MoveToPreGrasp
    ├── MoveToGrasp
    ├── CloseGripper
    └── AttachObject
```

The Retry re-ticks the entire Sequence on failure, trying a different grasp
candidate each time. This is the recovery that Lab 02 was missing.

**Deliverable 1.2:** Answer the following:

1. In the BT above, if `MoveToGrasp` fails on the first attempt, which nodes
   are re-executed on the next Retry tick?
2. What is the difference between a Sequence with `memory=True` and one with
   `memory=False` in py_trees? When would you use each?
3. If the Retry decorator exhausts all N attempts, what status does it return
   to its parent?

### Exploring the mock tree

A mock behavior tree is provided in `behavior_tree.py`. It implements the
full tree structure from this lab using fake nodes (no robot required) so you
can observe BT semantics before connecting to the real robot.

Run the viewer and the mock tree in separate terminals:

```bash
# Terminal 1
py-trees-tree-viewer --no-sandbox

# Terminal 2
ros2 run panda_moveit_config behavior_tree.py --ros-args -p scenario:=grasp_retry
```

Open `http://localhost:8080` in a browser to see the tree ticking live.

Available scenarios:

| Scenario | Description |
|----------|-------------|
| `happy_path` | Everything succeeds |
| `grasp_retry` | Grasp needs 2 attempts |
| `grasp_exhausted` | Grasp exhausts all N retries, outer Retry resets |
| `place_fails` | The place fails, outer Retry resets |

**Deliverable 1.3:** Run each scenario and take a screenshot of the tree
viewer showing the final status. For `grasp_exhausted`, describe in one
sentence what the outer Retry does that the inner Retry cannot.

---

## Part 2: ComputeGraspCandidates (20 points)

`ComputeGraspCandidates` is itself a **Selector** — it tries GPD first and
falls back to an enumerative strategy if GPD fails to produce candidates:

```
Selector (ComputeGraspCandidates)
├── GPDCandidates
└── EnumerativeCandidates
```

Open `behavior_tree.py` and find the `GPDCandidates` and `EnumerativeCandidates`
nodes. Each reads the target object pose from the blackboard and writes a ranked
list of `GraspCandidate` objects to `/candidates`.

**TODO:** Implement both nodes:

- `GPDCandidates.update()`: use the provided `GPDInterface` to generate learned
  candidates. Return FAILURE if GPD produces no valid candidates.
- `EnumerativeCandidates.update()`: sample candidates at fixed offsets and
  orientations around the object (top-down, angled, side).

Both must write to `/candidates` sorted by descending score.

**Deliverable 2.1:** Run the full tree on the robot and screenshot the tree
viewer showing which branch of the Selector was taken.

**Deliverable 2.2:** In one sentence, explain why the Selector took the branch
it did. If it fell back to enumerative, what did GPD fail to handle?

---

## Part 3: ExecutePick (25 points)

The `ExecutePick` node iterates the candidate list from the blackboard and
attempts each one in order. It mirrors the LAB02 `pick()` sequence using the
provided robot primitives.

**Provided primitives:**

```python
robot.move_arm_to_pose(pose: Pose) -> bool
robot.move_arm_to_joints(joints: dict) -> bool
robot.move_gripper(open: bool) -> bool
robot.attach_object(object_id: str)
```

**TODO:** Implement `ExecutePick.update()`. For each candidate:

1. Compute a pre-grasp pose (offset above the candidate position)
2. Move to pre-grasp
3. Descend to grasp pose
4. Close gripper
5. Attach object to planning scene
6. Retreat upward

Return `SUCCESS` if any candidate succeeds, `FAILURE` if all are exhausted.

**Deliverable 3.1:** Run with `scenario:=grasp_retry` and screenshot the tree
showing the Retry decorator counting failures before a successful pick.

**Deliverable 3.2:** Answer: what happens to the planning scene if
`ExecutePick` returns FAILURE after `attach_object()` has already been called?
How would you guard against this?

---

## Part 4: ExecutePlace (20 points)

The `ExecutePlace` node reads the place target from the blackboard (provided
by `ComputePlaceTarget`) and places the held object there.

**Provided primitives:**

```python
robot.move_arm_to_pose(pose: Pose) -> bool
robot.move_gripper(open: bool) -> bool
robot.detach_object(object_id: str)
```

**TODO:** Implement `ExecutePlace.update()`:

1. Compute a pre-place pose (offset above the target)
2. Move to pre-place
3. Descend to place height
4. Open gripper
5. Detach object from planning scene
6. Retreat upward

**Deliverable 4.1:** Run the full tree with the real robot and screenshot
a successful place.

**Deliverable 4.2:** Answer: `ExecutePlace` has no local Retry in the tree.
If place fails, what does the outer Retry reset to? Is this the right behavior?

---

## Part 5: Tree Composition (20 points)

Open `behavior_tree.py` and find the `build_tree()` function. Currently it
contains a placeholder.

**TODO:** Wire the full tree using `py_trees` composites and decorators:

```
Retry (outer)
└── Sequence (PickAndPlace)
    ├── MoveToHome
    ├── Retry(N)
    │   └── Sequence (Grasp)
    │       ├── ComputeGraspCandidates
    │       ├── FilterAndRank
    │       └── ExecutePick
    └── Sequence (Place)
        ├── ComputePlaceTarget
        └── ExecutePlace
```

**Deliverable 5.1:** Screenshot the tree viewer showing your composed tree
structure with all nodes visible.

**Deliverable 5.2:** Answer the following:

1. Why is `Retry(N)` placed around the Grasp sequence but not around the
   Place sequence?
2. What does the outer `Retry` reset that the inner one does not?
3. `MoveToHome` is the first node in the outer Sequence. What failure mode
   does this positioning handle?
4. The `Sequence` nodes use `memory=True`. What would change if you set
   `memory=False`?