"""Open the portable visual MJCF at its home pose. No model API or hardware IO."""
from pathlib import Path
import argparse
import time
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path,
        default=Path(__file__).resolve().parent / 'model/moss_visual.xml')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [.05, 0, .15]
        viewer.cam.distance = 1.05
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.opt.geomgroup[3] = False
        while viewer.is_running():
            start = time.monotonic()
            for _ in range(10): mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, 10 * model.opt.timestep - (time.monotonic() - start)))


if __name__ == '__main__': main()
