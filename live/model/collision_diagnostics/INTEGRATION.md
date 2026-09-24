# MOSS V0.4 collision diagnostics — 2026-09-25

This package provides runnable checks and a **diagnostic model variant** for
the geometry issues reported during MOSS integration. It does not contain
CAD sources, mesh assets, trained policies, a controller, or hardware commands.
Use the MOSS meshes and V0.4 visual overlay you already have.

## What is resolved

The bin is 260 mm high in the rover frame. The reported 295.7 mm comes from
reading compiled mesh vertices without their geom rotation. `audit.py`
reproduces the exact wrong numbers and verifies the corrected bounds against
the raw STL, including a translated/rotated-body test. See `results.json`.

The legacy 13 mm arm capsules really do underrepresent their visible parts.
Uniformly increasing their radius does not fix their offset. The candidate
reuses each existing visual part as a convex collision hull in exactly its
original pose. The default variant preserves the host's contact masks.

The candidate also restores the four original tapered bin walls (top around
261 mm) and replaces the two legacy track boxes with the convex hulls of the
V0.4 belt meshes (266 mm spacing). It changes no drive scaling/calibration.

## Run on your host robot

Dependencies: Python 3, `numpy`, `mujoco`. Validated on MuJoCo 3.13.0, macOS arm64 and Linux x86_64.
Start with your flattened V0.4 robot XML before attaching it to a room. Keep
the output beside its input so existing mesh/texture paths remain valid.
Use your host with the corrected measured mass model. The patch deliberately
preserves input masses: applying it to the old 6.138672 kg export would retain
that obsolete value. Our local original-export fixture is only a geometry and
preservation reference, not the recommended physical mass model.

```sh
python prepare_candidate.py /path/to/model/moss_v04.xml /path/to/model/moss_collision_candidate.xml --report candidate_report.json
```

For an explicitly separate self-contact diagnostic:

```sh
python prepare_candidate.py /path/to/model/moss_v04.xml /path/to/model/moss_self_contact_diagnostic.xml --self-contacts --report self_contact_report.json
```

For a renamed robot with prefixed names, the patch accepts `--prefix m0/`.
The bundled pose/render checks expect unprefixed standalone names. The patch
requires the original radian convention, named colliders, an explicit rover
and link inertia, the expected V0.4 bin placement, and no includes. It refuses
existing output files, repeat application, and external references to the
capsules that would require reviewing contact/sensor semantics.

`--self-contacts` changes original physical arm/gripper masks from `2/1` to
`2/3`. It adds **no exclusions**, and retains MuJoCo's parent-body filtering.
This is for discovering masked contacts, not blanket certification of all
self-collisions. Hosts with other masks need their own mask review.

## Reproduce our integration and pose checks

```sh
python verify_candidate.py /path/to/model/moss_v04.xml --output validation_local.json
python render_candidate.py /path/to/model/moss_self_contact_diagnostic.xml --pose reported_tuck --contacts --output tuck_local.png
```

On a headless Linux system, rendering may require `MUJOCO_GL=egl`.

Verification creates temporary copies and checks compiled body inertias,
joint layout, actuator mapping/parameters, equality data, keyframes, camera
parameters and selected solver settings. It also tests a deliberately altered
host with a 3.5 kg rover body, two extra planar joints, an extra camera and a
finger equality constraint. Those changes are preserved. This synthetic
fixture is a preservation test, not a physically calibrated mass model.

Measured here:

| Kinematic pose | Original masks/geometry | Updated geometry, original masks | Updated geometry, self-contact diagnostic |
|---|---:|---:|---:|
| Original home | 0 penetrating pairs | 0 | 0 |
| Reported tuck | 0 | 0 | 8 |

The reported tuck uses arm joints `(-0.8193, 0.304, 1.0968, 1.5959, 0.6327)`
in the original joint order; finger positions come from the original home
keyframe. The contacts involve the gripper against shoulder/upper-arm shapes.
`validation.json` records pair names and depths. `tuck_visual.png` shows the
exported geometry in this **kinematic** pose; red markers, when rendered with
`--contacts`, are computed contact points. This is not an animation of a
successful motion, nor a reproduction of the lab's servo-settling test.

## Remaining limits

- Per-part convex hulls fill recesses/holes in those parts. They improve
  coverage over the capsules but can introduce false contacts; review contact
  locations and decompose parts further where necessary.
- Hull, cover, arm base and gripper collision shapes remain legacy. This is
  not a complete V0.4 collision model. The bin remains separate wall/floor
  proxies so its opening is not filled by one convex hull.
- Updating contacts intentionally changes motion outcomes. Passing preservation
  checks does not mean previous pickup replays still work.
- No reach sampling, dynamic tuck trajectory, pickup success, friction,
  torque limits, physical clearance or sim-to-real transfer is validated here.
- No source model, CAD, firmware, controller, camera calibration or hardware
  is modified. No ONNX schema is invented. Please return the actual ordered
  32-observation / 8-action contract and the exact lab model/repro script.

The requesting lab's public repository, at the commit inspected
(`47c62e3fb5193191d6d7e1181cc61089eaa768c7`), had no MOSS files in its tree.
The lab's modified model was therefore not tested.

## Files and traceability

- `audit.py`, `results.json`: independent STL/compiled-frame audit, source hashes.
- `prepare_candidate.py`: creates a new candidate from existing host assets.
- `verify_candidate.py`, `validation.json`: integration checks and pose probe.
- `render_candidate.py`, `tuck_visual.png`: reproducible diagnostic preview.
- `candidate_validation.json`, `candidate_self_validation.json`: change reports.
- `validation_linux.json`: the same verification rerun on Linux x86_64 (MuJoCo 3.13.0), identical outcome.
- `SHA256SUMS`: fingerprints of the published files.

References: [MuJoCo mesh transforms](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)
and [contact filtering / convex meshes](https://mujoco.readthedocs.io/en/stable/computation/index.html#selection).
