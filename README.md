# MOSS × Jev — recorded physics replay

A static 3D demo of MOSS, the litter-picking rover, with Jev (TypeSafe AI's decision model) choosing each manipulation step. V0.2: three missions (pick up a can, retry a missed grasp, move closer and pick up) were run once in MuJoCo with real Jev API decisions; the browser replays the recorded physics at 25 frames per second. No live inference, no API calls, no backend. The seven V0.1 recorded scenarios are kept in the same interface.

Live: https://metrox-eth.github.io/moss-jev/ · Project: https://github.com/metrox-eth/moss

Credits and licenses: `licenses/NOTICE.txt` (SO-101 arm and NormaCore gripper geometry, Apache-2.0; Three.js, MIT). The 3D model is a presentation asset, not the manufacturing files.

## Run the MuJoCo missions locally

The executable CPU simulator, portable MOSS MJCF and meshes, and all three
recorded Jev missions are available in [`live/`](live/README.md).
No API key or GPU is required to replay the missions.

For importing MOSS into another simulator, start with
[`live/model/moss_robot.xml`](live/model/moss_robot.xml) and
[`live/model/meshes/`](live/model/meshes/); see the
[integration contract](live/INTEGRATION.md).
The full scene is [`live/model/moss_visual.xml`](live/model/moss_visual.xml).

This is the V0.2 demonstration model, with simplified base translation and
gripper physics. It is not manufacturing CAD or a calibrated tracked-drive model.
The static browser demo above remains unchanged.
