# MOSS V0.4 — rover visual update

The first portable MuJoCo export used the earlier Jev presentation model.
This package updates the rover's appearance to the saved V0.4 hull assembly
and the exterior context used for the V0.4 animation.

It is a visual overlay for the existing MOSS MJCF. It does not supply a new
robot controller or a calibrated V0.4 contact/inertia model.

## What is included

- Updated hull and motor brackets from `MOSS_Hull_V04_Cartes_INA90.step`.
- Longer cover, arm platform, bin and track side assemblies from the exterior
  context retained in the V0.4 animation. These are not a claim that every
  planned mechanical revision has been implemented.
- Simplified component envelopes at the saved placements: Jetson, QT Py/BFF,
  MDD3A, Waveshare servo adapter, INA219, camera, motors and candidate battery.
- RealSense body envelope with front-face centre at `(0.156, 0, 0.075)` m.
- Continuous visual belts at Y = ±0.133 m, replacing the earlier tread boxes.
- A manifest with mesh bounds, source categories and SHA-256 fingerprints.

Internal boards, motor envelopes and camera lens markers are illustrative.
The blue battery is the 75 × 82 × 32 mm candidate packaging volume, not the
measured battery installed in the prototype. Supplied/vendor STEP files are
not included. Arm and gripper visuals continue to come from the host model.

## Apply to an existing robot

Extract this directory as `visuals_v04` beside the source MJCF:

```text
model/
  moss_robot.xml
  meshes/                 (existing arm/gripper assets)
  visuals_v04/
    apply_visuals.py
    manifest.json
    meshes/
    ...
```

From `model/`:

```sh
python visuals_v04/apply_visuals.py moss_robot.xml moss_robot_v04_visual.xml
```

For a complete mission scene, the same command accepts `moss_visual.xml`.
Use the generated XML when importing the robot. Keep `visuals_v04` with it:
the output uses relative mesh paths. No external Python package is required
to apply the overlay; MuJoCo is required to load the resulting model.

Apply it to your own corrected, flattened robot XML **before attaching it to
the room**. It preserves your explicit inertial, added base joints, cameras,
actuators, keyframes, solver settings and collision proxies. It replaces only
direct-child `visual_body_*` and `tread_*` geoms marked with zero mass and zero
contact masks. Existing files are not overwritten.

For renamed bodies use `--body m0/rover`; if visual names also have a prefix,
use `--prefix m0/`. Includes, custom asset layouts or a body whose local frame
was changed need a manual merge. The provided `assets_fragment.xml` and
`rover_geoms_fragment.xml` list the elements to add under the host's `<asset>`
and rover `<body>` respectively. Resolve mesh paths relative to the host's
compiler mesh directory when merging. All vertices use the original rover
frame: metres, +X forward, +Y left, +Z up.

## Integration details

**Mass:** this overlay contains no inertial replacement. Keep the host's
measured/estimated values. The old export's 6.138672 kg base was a frozen
solid-geometry calculation, not the measured rover mass. The physical
prototype was weighed at 3.5 kg without the arm; that measurement does not
establish the mass of the revised V0.4 CAD assembly.

**Track width:** the legacy simulation collision proxies are centred at
Y = ±0.122 m (0.244 m spacing). The retained CAD wheels/visual belts are at
Y = ±0.133 m (0.266 m spacing). This update makes that existing geometry
discrepancy explicit; it does not change the drive controller or its calibration.
A consistent V0.4 driving variant should reconcile collision locations and
the track-width parameter together, then validate turning against the real
robot. Skid-steering effective width also depends on the contact surface.

**Track animation:** the new belts and wheels are static visual geoms attached
to `rover`. Translation/yaw follows the body, but this package adds no tread
circulation. A host animating `tread_*` by name must update that animation.
Wheel-centre metadata is in the manifest. Preserve the previous assets until
the host's visual animation has been adapted if that feature is needed.

**Camera:** the envelope was updated, but the host's camera pose/FOV is preserved.
For the nominal front-face frame, the pose is:

```xml
<camera name="front_camera" pos="0.156 0 0.075"
        xyaxes="0 -1 0 0 0 1"/>
```

This is not an RGB/IR optical calibration. The three visual lens markers do
not define optical centres. Camera occlusion by the gripper also depends on
the arm pose and is not solved by replacing the rover mesh.

**Contacts and replay:** new meshes have `mass="0"`, `contype="0"`,
`conaffinity="0"`, `group="2"`. Old collision proxies remain the contact model;
newly visible surfaces are not added as obstacles automatically. Host-room
solver settings remain global: use the original 0.002 s / implicitfast /
elliptic / 80 if reproducing the recorded missions.

## Validation

MuJoCo 3.13.0 loaded the patched robot and full scene. Compiled inertias,
joints, actuators, keyframes, camera parameters, collision proxies and solver
settings match their respective input models. The home-settling qpos delta
was zero. All three offline missions (`can`, `miss`, `far`) completed with the
object settled in the bin and zero API calls.

A separate synthetic host with a 3.5 kg base, extra planar joints and a front
camera also preserved those fields. This check did not use the requesting
lab's files. `validation.json` records the results and `comparison.png` shows both
compiled models from the same viewpoint.

The original replay package and original CAD source files remain the baseline.
