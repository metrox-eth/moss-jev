# MOSS body integration contract

## Units and coordinate frame

SI units throughout MJCF: metres, kilograms, seconds, radians. +X is rover
forward, +Y left, +Z up. MuJoCo quaternions are WXYZ. The bin sits behind the arm.

The `rover` body is the robot subtree root. `base_link` is the SO-101 arm mount;
`gripper_frame_link` carries the gripper. Site `tcp` is the task-space target.
Use `home` to initialise both qpos and actuator controls before stepping.

## Actuators, in order

| Index | Name / joint | Command |
| --- | --- | --- |
| 0 | `shoulder_pan` | Position, rad |
| 1 | `shoulder_lift` | Position, rad |
| 2 | `elbow_flex` | Position, rad |
| 3 | `wrist_flex` | Position, rad |
| 4 | `wrist_roll` | Position, rad |
| 5 | `finger_left` | Slide position, 0–0.041 m |
| 6 | `finger_right` | Slide position, 0–0.041 m |
| 7 | `base_x` | Base translation, 0–0.8 m |

Joint qpos order differs from actuator order: look up names with
`model.joint(name)` and `model.jnt_qposadr`. Joint limits and servo gains are in
the XML. The opening command used by missions is 0.037 m on each finger.

`moss_robot.xml` has 8 qpos, 8 velocities, 8 actuators. The complete scenes add
the can's free joint (`litter_free`): 15 qpos, 14 velocities, 8 actuators.

## Minimal import versus full missions

For a generic lab body, the robot-only MJCF, mesh directory and home keyframe
are sufficient to load and pose the model. The file does not include a ground
plane, target object, sensors or a locomotion policy. When composing multiple
robots, namespace all body/joint/actuator/mesh/keyframe names consistently, and
preserve the mesh directory resolution. The included example is one robot.

For the existing pickup missions, use `moss.xml` or `moss_visual.xml`, since
`Physics` expects the can and its contacts:

```python
from pathlib import Path
from physics import Physics

p = Physics(Path("model/moss_visual.xml"))
p.reset("can")
p.finish("approach")
print(p.observe())
```

`finish(action)` is a blocking convenience; an interactive host should call
`command(action)` once, then `step()` at the model timestep (0.002 s), until
`p.plan is None`. Actions are `reposition`, `approach`, `align`, `close`, `lift`,
`retry`, `clear`, `carry`, `release`, `home`; `stop` is handled by the session.

Recorded decisions are `rows[*].decision.action`; corresponding observations
are `rows[*].observation`. `replay_missions.py` exercises the same session path as
the browser without loading a credential. No Jev replacement policy is included.

## Visuals and contact model

`model/meshes/` is relative to each MJCF. Visual meshes and tread blocks use
group 2, zero mass, `contype=0`, `conaffinity=0`. Collision proxies use group 3
and are transparent in the visual files. The can/ground remain visible.
Body inertia is frozen to the original compiled model before visuals are added.

The 57 meshes were simplified from the existing GLB for native rendering.
`mesh_manifest.json` maps each mesh to its MuJoCo body and source GLB node.
Jaw visuals follow the finger bodies using the same pad-centre convention as the
browser. Changing visual detail does not introduce new collision surfaces.

## What needs an adapter in a general lab

The base has no yaw or differential drive. To navigate a shared stage, replace
or extend that base model and provide a drive controller. That is a new physics
variant; do not expect existing recorded paths to remain bit-for-bit valid.

There is no calibrated camera/noise model: the Jev demos read MuJoCo state.
For perception experiments, add sensors and an observation adapter explicitly.
The model is suitable as a starting point for import, manipulation and recovery
experiments, not evidence of measured terrain performance or real-world payload.
