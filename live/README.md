# MOSS × Jev — CPU MuJoCo simulator

The executable simulator behind the three MOSS × Jev V0.2 missions. Includes a
portable MOSS MJCF with visual meshes, contact-based grasping, the recorded Jev
choices, a headless replay command and the original local browser interface.

This is the September 2026 demonstration model. It is not the V0.4 manufacturing
CAD or a calibrated digital twin. The public website at the repository root
continues to play pre-recorded physics; this folder runs MuJoCo locally.

## Quick start — no API key

From the `live/` directory, with Python 3.12:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python replay_missions.py
```

All three recorded missions should finish with `"passed": true`, zero API calls
and the can settled in the bin. No GPU is required for this headless command.
The included MJCF is already built; regenerating it is optional.

For the local browser interface:

```sh
.venv/bin/python server.py
```

Open <http://127.0.0.1:8875/> and press **PLAY REPLAY**. The API switch starts OFF.
The browser needs WebGL for the 3D view; MuJoCo physics runs on the CPU.
Set `MOSS_SIM_PORT` to choose another local port. This is a single-user local
development server, not a public hosted service. Keep the existing static site
deployment separate from this Python folder.

## Import MOSS into another MuJoCo lab

Start with **`model/moss_robot.xml` and the entire `model/meshes/` directory**.
All mesh paths are relative; no original CAD files or downloads are required.

```python
import mujoco

model = mujoco.MjModel.from_xml_path("model/moss_robot.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
mujoco.mj_forward(model, data)
```

For a complete scene with ground, can, light and camera, use
**`model/moss_visual.xml`**. To inspect it in the native viewer:

```sh
# Linux, with a display:
.venv/bin/python view_model.py
# macOS (MuJoCo's viewer requires its launcher):
.venv/bin/mjpython view_model.py
```

**V0.4 appearance (optional).** `model/visuals_v04/` updates the rover's look to the
saved V0.4 hull assembly (longer cover, side assemblies, camera envelope) without
touching masses, joints, contacts or the recorded missions:

```sh
cd model && python visuals_v04/apply_visuals.py moss_robot.xml moss_robot_v04_visual.xml
```

Known discrepancy: the CAD belts sit 266 mm apart, the legacy collision boxes
244 mm. The overlay does not change the drive model; see
[`model/visuals_v04/README.md`](model/visuals_v04/README.md).

The robot-only file contains a home keyframe and eight actuators. Its base is
constrained to a single X slide; importing it does not create a differential
drive controller. See [INTEGRATION.md](INTEGRATION.md) for the joint contract,
composition notes and limitations.

Files:

| File | Purpose |
| --- | --- |
| `model/moss_robot.xml` | Robot only, visual meshes, no floor or loose can |
| `model/moss_visual.xml` | Complete visual scene, same contact geometry as the missions |
| `model/moss.xml` | Original compact physics scene; primitives only, no mesh dependency |
| `model/meshes/*.stl` | 57 visual-only meshes derived from the existing presentation GLB |
| `public/replay/assets/moss.glb` | Original presentation geometry for browser viewers |
| `model/so101.urdf` | Arm kinematics/inertias used by the generator; visual/collision elements removed |
| `recordings/{can,miss,far}.json` | Actual recorded Jev decisions and their observations |
| `physics.py` | Contacts, IK and action execution |
| `policy.py` | Optional live Jev API adapter |

The arm URDF is a generator input, **not a complete MOSS URDF**. Use the MJCF
files above for import. Visual STL meshes are simplified presentation assets,
not printable manufacturing parts. The original browser geometry is preserved.

## Missions

| ID | Initial setup | What the recording tests |
| --- | --- | --- |
| `can` | Can at X=0.315 m | Pick, lift, carry and release into the bin |
| `miss` | Same can; first alignment offset by 85 mm | Re-alignment/recovery using state feedback |
| `far` | Can at X=0.640 m | Move base closer, then collect |

Jev selects an action from a fixed menu; inverse kinematics and motion
interpolation execute it. Inputs are simulator state, not camera perception.
The `miss` disturbance is deliberately injected; it is not a spontaneous
perception error. Replay repeats recorded decisions in newly calculated physics.

The can moves through gravity and contacts, with no attachment/weld or scripted
teleport during a pickup. Success requires the released object to remain in the
bin, below 2 cm/s for more than 0.3 seconds. Final resting positions can differ
between platforms; the replay checks the settled outcome after release and
retains position checks during manipulation.

## Optional live Jev decisions

Run `configure_api.py` locally to enter your own OpenRouter key without echo, or
provide `OPENROUTER_API_KEY` in the server environment. The key stays outside the
repository, normally in `~/.config/moss/openrouter.key`; restart the server,
enable **LIVE API**, then choose **RUN LIVE**. You pay for those new API calls.
The default adapter uses `typesafe/jev-1.13` at `/api/alpha/decisions`.
Provider/model availability may change. Live calls were not made for this package.

The local guard allows 60 calls and checks $0.25 of provider-reported cost per
process. This is not a billing guarantee or a public-server spending limit.
Replay requires neither a key nor access to that service.

## Checks and regeneration

```sh
.venv/bin/python -m unittest -v test_sim test_portable
.venv/bin/python replay_missions.py --model model/moss_visual.xml
```

The release verification and exact dependency versions are in
[`validation/release.json`](validation/release.json).

`build_model.py` regenerates `model/moss.xml` from the arm kinematics. To also
regenerate the visual MJCF files and meshes, install the optional export tools:

```sh
.venv/bin/python -m pip install -r requirements-export.txt
.venv/bin/python build_model.py
.venv/bin/python tools/export_visuals.py
```

## Scope

- Base translation is a position-controlled slide, not tracked-ground dynamics.
  The browser animates tread travel; native MJCF tread blocks stay fixed to the base.
- The gripper uses two independent sliding finger servos and assumed forces and
  masses; it does not reproduce the NormaCore transmission or real servo electronics.
- Hull/bin collisions and arm capsules are simplified. Arm-arm self-collision is
  omitted; arm-environment contacts remain enabled.
- The simulator pauses during a live API decision. Recorded/API latency is not
  evidence of a continuous real-time robot control loop.
- No real robot connection, navigation stack or real camera perception is included.

## License and attribution

MOSS simulation code uses Apache-2.0: [LICENSE](LICENSE). MOSS rover visual
geometry and documentation use CC BY 4.0. See [NOTICE](NOTICE) for file scopes.
SO-101 and NormaCore source geometry retain their Apache-2.0 notices. Three.js
is MIT; Space Grotesk is SIL OFL 1.1. Full notices are under
[`public/replay/licenses/`](public/replay/licenses/). No manufacturing CAD is released here.
