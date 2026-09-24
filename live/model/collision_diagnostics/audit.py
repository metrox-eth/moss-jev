#!/usr/bin/env python3
"""Read-only geometry audit. No dynamics, CAD edits, or hardware commands.

Requires numpy and mujoco. Pass the original robot XML and V0.4 visual
package directory. Results refer to those files, not a downstream lab fork.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def raw_stl(path):
    data = path.read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * n:
        raise ValueError(f'Not a binary STL: {path.name}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(data, dtype=dtype, count=n, offset=84)['v'].reshape(-1, 3).astype(float)


def bounds(points):
    return {'min': points.min(axis=0).tolist(), 'max': points.max(axis=0).tolist()}


def mesh_points(model, data, gid):
    mid = model.geom_dataid[gid]
    start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    local = model.mesh_vert[start:start + count].astype(float)
    return local @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]


def minimal_mesh(path, rotated=False):
    root = ET.Element('mujoco')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='mesh', file=str(path), inertia='shell')
    world = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(world, 'body', name='rover',
                         pos='0.4 -0.2 0.1' if rotated else '0 0 0',
                         quat='0.9238795325 0 0 0.3826834324' if rotated else '1 0 0 0')
    ET.SubElement(body, 'geom', name='visual', type='mesh', mesh='mesh',
                  mass='0', contype='0', conaffinity='0')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--visuals', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.robot.resolve()
    package = args.visuals.resolve()
    tree = ET.parse(source).getroot()
    meshdir = source.parent / tree.find('compiler').get('meshdir', '')
    used_files = {source}
    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    result = {'mujoco_version': mujoco.__version__, 'units': 'metres unless explicitly labelled',
              'scope': 'Local original robot export and V0.4 visual package; not the modified lab model.'}

    path = package / 'meshes/v04_030_bin.stl'
    used_files.add(path)
    raw = raw_stl(path)
    bm, bd = minimal_mesh(path)
    correct = mesh_points(bm, bd, 0)
    wrong = bm.mesh_vert.astype(float) + bd.geom_xpos[0]
    np.testing.assert_allclose([correct.min(0), correct.max(0)],
                               [raw.min(0), raw.max(0)], atol=1e-6, rtol=0)
    # Independent check after a body translation/yaw: raw STL versus compiled mesh.
    rm, rd = minimal_mesh(path, rotated=True)
    expected = raw @ rd.xmat[1].reshape(3, 3).T + rd.xpos[1]
    transformed = mesh_points(rm, rd, 0)
    np.testing.assert_allclose([transformed.min(0), transformed.max(0)],
                               [expected.min(0), expected.max(0)], atol=1e-6, rtol=0)
    result['bin'] = {'raw_stl': bounds(raw), 'correct_compiled': bounds(correct),
                     'omitted_geom_rotation': bounds(wrong),
                     'compiled_vertex_count': int(bm.mesh_vertnum[0]),
                     'raw_and_compiled_bounds_agree_1um': True,
                     'translated_rotated_body_bounds_agree_1um': True}
    boxes = {}
    signs = np.array(list(itertools.product([-1, 1], repeat=3)))
    for name in ['bin_floor', 'bin_x-1', 'bin_x1', 'bin_y-1', 'bin_y1',
                 'hull', 'track_-1', 'track_1']:
        gid = model.geom(name).id
        points = (signs * model.geom_size[gid]) @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
        boxes[name] = bounds(points)
    result['legacy_collision_boxes_at_zero_pose'] = boxes

    arm = {}
    mesh_assets = {a.get('name'): a for a in tree.findall('./asset/mesh')}
    for name in ['upper_arm_link', 'lower_arm_link', 'wrist_link', 'shoulder_link']:
        body_xml = tree.find(f".//body[@name='{name}']")
        bid = model.body(name).id
        rg = data.xmat[bid].reshape(3, 3)
        all_points = []
        for geom in body_xml.findall('geom'):
            if geom.get('type') != 'mesh':
                continue
            gid = model.geom(geom.get('name')).id
            local = (mesh_points(model, data, gid) - data.xpos[bid]) @ rg
            asset = mesh_assets[geom.get('mesh')]
            mesh_path = meshdir / asset.get('file')
            used_files.add(mesh_path)
            raw_part = raw_stl(mesh_path)
            # This exported arm uses identity local geom poses and no mesh scales.
            assert geom.get('pos', '0 0 0') == '0 0 0'
            assert geom.get('quat', '1 0 0 0') == '1 0 0 0'
            assert asset.get('scale', '1 1 1') == '1 1 1'
            np.testing.assert_allclose([local.min(0), local.max(0)],
                                       [raw_part.min(0), raw_part.max(0)], atol=1e-6, rtol=0)
            all_points.append(local)
        vertices = np.concatenate(all_points)
        proxy = body_xml.find(f"geom[@name='{name}_proxy']")
        a, b = np.fromstring(proxy.get('fromto'), sep=' ').reshape(2, 3)
        delta = b - a
        t = np.clip((vertices - a) @ delta / (delta @ delta), 0, 1)
        distances = np.linalg.norm(vertices - (a + t[:, None] * delta), axis=1)
        radius = float(proxy.get('size'))
        arm[name] = {'visual_bounds_link_frame': bounds(vertices),
                     'proxy_segment_link_frame': [a.tolist(), b.tolist()],
                     'proxy_radius': radius,
                     'max_vertex_distance_from_segment': float(distances.max()),
                     'max_vertex_outside_original_capsule': float(np.maximum(0, distances-radius).max()),
                     'max_vertex_outside_radius_35mm_capsule': float(np.maximum(0, distances-.035).max()),
                     'note': 'Vertex coverage of the exported visuals; not a collision/path validation.'}
    result['arm'] = arm
    mask_ids = [model.geom(f'{n}_proxy').id for n in arm]
    result['arm_collision_masks'] = {
        'explicit_pair_count_in_original': int(model.npair),
        'capsules': {model.geom(g).name: [int(model.geom_contype[g]), int(model.geom_conaffinity[g])]
                     for g in mask_ids},
        'any_capsule_pair_passes_mask_test': any(
            (int(model.geom_contype[a]) & int(model.geom_conaffinity[b])) or
            (int(model.geom_contype[b]) & int(model.geom_conaffinity[a]))
            for a, b in itertools.combinations(mask_ids, 2)),
        'note': 'These original masks exclude automatic capsule/capsule contacts. Downstream masks unknown.'}

    shapes = {}
    for name in ['v04_belt_left', 'v04_belt_right', 'v04_hull']:
        path = package / f'meshes/{name}.stl'
        used_files.add(path)
        vertices = raw_stl(path)
        shapes[name] = bounds(vertices)
    result['v04_visual_bounds_rover_frame'] = shapes
    centres = [(np.array(shapes[name]['min']) + np.array(shapes[name]['max'])) / 2
               for name in ['v04_belt_left', 'v04_belt_right']]
    result['track_centre_spacing'] = {
        'visual_geometry': float(abs(centres[0][1] - centres[1][1])),
        'legacy_proxies': float(abs(model.geom_pos[model.geom('track_1').id, 1] -
                                   model.geom_pos[model.geom('track_-1').id, 1])),
        'note': 'Geometric spacing only; effective skid-steer width needs physical calibration.'}
    result['provenance_sha256'] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(used_files)}
    result['not_verified'] = ['Downstream 40k-pose sample and jam/settling results',
                              'Candidate tuck path and real hardware clearances',
                              'ONNX ordered 32-slot observation and 8-slot action contract',
                              'Camera specifications and measured perception timing']
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['bin', 'arm', 'arm_collision_masks',
                                                  'track_centre_spacing']}, indent=2))


if __name__ == '__main__':
    main()
