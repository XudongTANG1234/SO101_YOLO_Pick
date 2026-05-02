# SO-101 Pick & Place Improvement Plan

## Goal

The final goal is to satisfy all three of the following simultaneously.

1. Estimate the 3D position of an object stably and accurately.
2. Consistently compute the approach direction and gripper rotation angle required for grasping.
3. Based on perception results, have MoveIt and the gripper execute a reproducible pick sequence.

This document fixes the implementation order and serves as the working reference for verifying each step one at a time.

---

## Working Principles

1. Prioritize actual code and runtime behavior over documentation.
2. Fix factors that directly affect accuracy first.
3. Validate perception, transform, planning, and execution separately.
4. Avoid large refactors all at once — leave measurable results at each step.
5. Ensure the pick motion is performed consistently based on the target confirmed at the start.

---

## Agreed Conventions

### Coordinate Frame

The final pick target in the current system is expressed relative to `base_link`, not `world`.

Reason:
- The current goal is not a fixed-arm demo, but a system that also accounts for the arm being mounted on a quadruped robot in the future.
- In that case, expressing the grasp target in the robot body frame rather than a global frame is more natural and extensible.
- Therefore, the perception pipeline targets a structure where values measured in the camera frame are ultimately converted to a target expressed in `base_link`.

Summary:
- Raw sensing frame: camera optical frame
- Execution target frame: `base_link`
- `world` is not treated as a required reference frame at this time

### When the Target Is Committed

The pick target is not committed at the moment the object is first detected. It is committed based on the latest perception value at the moment the actual motion command is received.

Applicable commands:
- `go`
- `pick`
- `plan` and similar commands may follow the same principle if needed

Summary:
- The perception node continuously updates with the latest detection results.
- However, the target the robot actually uses is fixed as a snapshot at the moment the motion command is received.
- During execution, even if new perception results arrive, the target for the current motion does not change.

This convention assumes the following execution flow.

1. Object detection results are continuously updated.
2. The user sends a `go` or `pick` command.
3. The latest `position`, `orientation`, `grip_angle`, and `timestamp` at that moment are committed as a snapshot.
4. From that point on, standoff, approach, and gripper close use only that snapshot.

---

## Phases

### Phase 1. Baseline Cleanup

Objective:
- Consolidate which parameters and values are actually used in the current system.
- Eliminate mismatches between documentation, launch files, yaml files, and code defaults.
- Establish a baseline for subsequent accuracy tuning.

Key tasks:
- Unify the source of camera extrinsic parameters
- Remove mismatches between `launch`, `yaml`, and code default values
- Re-verify topic, frame, and controller names
- Explicitly define which pose the pick sequence uses as its reference

Completion criteria:
- The parameters actually in use are managed in a single place.
- There is no confusion from different camera positions or YOLO defaults across launches.
- The system data flow can be described in one sentence.

---

### Phase 2. Lock Down the Pick Sequence

Objective:
- Ensure that once a pick command is issued, the motion completes against a fixed target even if perception values keep changing.
- Guarantee in code that the robot moves based on the value captured at the moment the command was issued.

Key tasks:
- Latch a snapshot of `pose`, `normal`, `grip_angle`, and `timestamp` when pick starts
- Separate updates to `latest_*` from the execution target during pick
- Explicitly define the order: `80% open → standoff → final approach → close`
- Define retreat and home motions if needed

Completion criteria:
- The target does not shift during a pick command even if perception values change.
- The execution sequence is divided into clearly defined steps in the code.

---

### Phase 3. Camera-to-Base Transform Cleanup

Objective:
- Restructure the camera-to-robot transform, which is the largest source of 3D position error, into a reliable form.

Key tasks:
- Review the current manual `cam_x`, `cam_y`, `cam_z`, `cam_pitch` parameter structure
- Move to a TF-based fixed transform structure where possible
- Organize into a structure that supports calibration
- Verify that perception results consistently come out in `base_link` coordinates

Completion criteria:
- The transform path from camera frame to base frame is explicit.
- The extrinsic values can be updated through a physical measurement or calibration procedure.

---

### Phase 4. Perception Accuracy Improvement

Objective:
- Move beyond simple detection center points and reliably produce a target pose suitable for grasping.

Key tasks:
- Revisit the center ROI depth computation method
- Improve depth foreground / outlier filtering
- Stabilize surface normal computation
- Stabilize bbox-based grip angle computation
- Evaluate introducing segmentation or a depth-based object mask if needed

Completion criteria:
- The perception node stably outputs object position, approach direction, and gripper rotation.
- The debug overlay and logs alone are sufficient to judge whether results are reasonable.

---

### Phase 5. Restructure Around a Grasp Frame

Objective:
- Move beyond the temporary structure of handling `normal` and `grip_angle` separately, and represent the final grasp pose consistently.

Key tasks:
- Define the grasp frame using approach axis, lateral axis, and gripper axis
- Organize the rotation matrix → quaternion generation pipeline
- Revisit the perception output message structure
- Fix the final grasp target format that planning will consume

Completion criteria:
- The grasp target is consistently expressed as `position + orientation`.
- Dependency on ad-hoc logic that overwrites only the wrist roll is reduced.

---

### Phase 6. MoveIt Planning Improvement

Objective:
- Ensure that the computed orientation is meaningfully reflected in actual motion planning and IK.

Key tasks:
- Review whether to keep `position_only_ik`
- Decide between enforcing full orientation vs. an approach-axis-first strategy
- Separate standoff planning from final approach planning
- Define a fallback strategy for planning failures

Completion criteria:
- MoveIt actually reflects the grasp orientation, or at minimum the approach axis.
- The motion to the target becomes more grasp-friendly compared to simple position-only movement.

---

### Phase 7. Add Validation and Tuning Infrastructure

Objective:
- Avoid relying on intuition to judge whether changes are improvements.

Key tasks:
- Enhance perception debug overlay
- Organize logging for detected target, commanded target, and actual motion
- Define criteria for repeatable testing
- Identify error measurement points

Completion criteria:
- After each phase's changes, there are comparable logs or observation criteria.
- Tuning direction can be decided objectively.

---

## Recommended Order of Execution

Work proceeds in the following order.

1. Baseline Cleanup
2. Lock Down the Pick Sequence
3. Camera-to-Base Transform Cleanup
4. Perception Accuracy Improvement
5. Restructure Around a Grasp Frame
6. MoveIt Planning Improvement
7. Add Validation and Tuning Infrastructure

---

## Current Assessment

The current repo already has some of the following in place.

- YOLO + RealSense-based detection
- Depth-based 3D position estimation
- Surface normal computation
- Grip angle computation
- MoveIt integration

However, it is closer to a "working prototype" at this point. To treat it as an accuracy-first system, the following are still needed.

- Consistent extrinsic management
- Target commitment at pick time
- A more stable grasp orientation representation
- A structure where orientation is actually reflected in planning
- A repeatable and verifiable test workflow

---

## Working Method

Work through this document phase by phase, one at a time.

For each phase:

1. Re-confirm the objective.
2. Narrow down the relevant files.
3. Apply only the necessary changes.
4. Record the verification method and remaining risks.
