# MOSS × Jev — recorded physics replay

A static 3D demo of MOSS, the litter-picking rover, with Jev (TypeSafe AI's decision model) choosing each manipulation step. V0.2: three missions (pick up a can, retry a missed grasp, move closer and pick up) were run once in MuJoCo with real Jev API decisions; the browser replays the recorded physics at 25 frames per second. No live inference, no API calls, no backend. The seven V0.1 recorded scenarios are kept in the same interface.

Live: https://metrox-eth.github.io/moss-jev/ · Project: https://github.com/metrox-eth/moss

Credits and licenses: `licenses/NOTICE.txt` (SO-101 arm and NormaCore gripper geometry, Apache-2.0; Three.js, MIT). The 3D model is a presentation asset, not the manufacturing files.
