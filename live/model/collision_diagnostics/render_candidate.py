#!/usr/bin/env python3
"""Render a kinematic diagnostic pose; never connects to hardware.

On a headless Linux host, try MUJOCO_GL=egl. Requires numpy and mujoco.
"""
import argparse
from pathlib import Path
import struct
import zlib

import mujoco
import numpy as np


def png(path, pixels):
    def chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data
                + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))
    h, w = pixels.shape[:2]
    data = b''.join(b'\0' + row.tobytes() for row in pixels)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'
                     + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(data)) + chunk(b'IEND', b''))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('model', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--pose', choices=['home', 'reported_tuck'], default='reported_tuck')
    p.add_argument('--contacts', action='store_true')
    a = p.parse_args()
    m = mujoco.MjModel.from_xml_path(str(a.model.resolve()))
    d = mujoco.MjData(m)
    home = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if home < 0:
        raise ValueError('Expected a home keyframe.')
    mujoco.mj_resetDataKeyframe(m, d, home)
    if a.pose == 'reported_tuck':
        for name, value in zip(['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
                               [-.8193, .304, 1.0968, 1.5959, .6327]):
            d.qpos[m.jnt_qposadr[m.joint(name).id]] = value
    mujoco.mj_forward(m, d)
    m.vis.headlight.ambient[:] = [.65, .65, .65]
    m.vis.headlight.diffuse[:] = [.8, .8, .8]
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[2] = 1
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, .15]
    cam.distance, cam.azimuth, cam.elevation = .68, 120, -22
    with mujoco.Renderer(m, height=480, width=640) as renderer:
        renderer.update_scene(d, camera=cam, scene_option=opt)
        if a.contacts:
            for contact in d.contact:
                if contact.dist >= -1e-6 or renderer.scene.ngeom >= renderer.scene.maxgeom:
                    continue
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.array([.004, .004, .004]), contact.pos,
                                   np.eye(3).flatten(), np.array([1., .12, .03, 1.], dtype=np.float32))
                renderer.scene.ngeom += 1
        png(a.output, renderer.render().copy())
