# LAB03 Spec: Behavior Trees for Robot Manipulation

## Learning Objective

In LAB02, pick-and-place was implemented as a monolithic `pick()` function —
a sequential script where failure meant returning `False` with no recovery.
In LAB03, students decompose that same sequence into a Behavior Tree, making
control flow explicit and recovery structural. The goal is to understand how
BT architecture increases **interpretability** (the tree is the documentation),
**testability** (each node can be tested in isolation), and **robustness**
(failure propagation and recovery are defined by the tree, not buried in code).

---

## The Tree

```
Retry (outer)
└── Sequence (PickAndPlace)
    ├── MoveToHome
    ├── Retry(N)
    │   └── Sequence (Grasp)
    │       ├── ProposeGrasps
    │       ├── FilterByIK
    │       └── ExecutePick
    └── Sequence (Place)
        ├── ReadGoal
        └── ExecutePlace
```

---

## Sections

- Part 1: Introduction to Behavior Trees
- Part 2: ProposeGrasps
  - TODO: implement a grasp candidate strategy of your choice + justification
- Part 3: ExecutePick
  - TODO: iterate candidates, execute pick using provided primitives
- Part 4: ExecutePlace
  - TODO: execute place using provided primitives
- Part 5: Tree Composition
  - TODO: wire the full tree + written justification of recovery structure

---

## Provided

- `FilterAndRank`: deterministic IK check
- `ReadGoal`: iterates over fixed row of place targets
- Robot primitives: `move_arm_to_pose`, `move_arm_to_joints`,
  `move_gripper`, `attach_object`, `detach_object`
- Blackboard schema and initialization
- `behavior_tree.py`: mock tree for exploring BT semantics
