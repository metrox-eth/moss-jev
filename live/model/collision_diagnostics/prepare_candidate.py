#!/usr/bin/env python3
"""Create a NEW diagnostic collision variant of a flattened V0.4 MOSS MJCF.

Requires mujoco and numpy; reuses host mesh assets. This is a geometry
candidate, not a validated contact model, controller, or hardware motion.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

POSE_KEYS = ('pos', 'quat', 'euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto')
INVARIANTS = ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos',
              'body_quat', 'jnt_type', 'jnt_bodyid', 'jnt_pos', 'jnt_axis',
              'jnt_range', 'dof_damping', 'dof_armature', 'actuator_trnid',
              'actuator_ctrlrange', 'actuator_forcerange', 'actuator_gainprm',
              'actuator_biasprm', 'actuator_gear', 'eq_data', 'eq_obj1id',
              'eq_obj2id', 'key_qpos', 'key_ctrl', 'cam_pos', 'cam_quat',
              'cam_fovy', 'cam_intrinsic', 'cam_resolution')


def compiled_mesh_world(model, data, gid):
    mid = model.geom_dataid[gid]
    a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    return (model.mesh_vert[a:a+n] @ data.geom_xmat[gid].reshape(3, 3).T
            + data.geom_xpos[gid])


def clone_collision(proxy, visual, name):
    geom = copy.deepcopy(proxy)
    for key in (*POSE_KEYS, 'size', 'mesh', 'hfield', 'density'):
        geom.attrib.pop(key, None)
    geom.set('name', name)
    geom.set('type', 'mesh')
    geom.set('mesh', visual.get('mesh'))
    for key in POSE_KEYS:
        if key in visual.attrib:
            geom.set(key, visual.get(key))
    geom.set('mass', '0')
    geom.set('group', '3')
    geom.set('rgba', '.95 .55 .12 .35')
    return geom


def prepare(source, destination, prefix='', self_contacts=False):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists() or source == destination or source.parent != destination.parent:
        raise ValueError('Choose a NEW output XML beside the input. No files are overwritten.')
    root = ET.parse(source).getroot()
    if root.findall('.//include'):
        raise ValueError('Flatten includes before using this diagnostic patch.')
    compiler = root.find('compiler')
    if compiler is None or compiler.get('angle', 'degree') != 'radian':
        raise ValueError('This patch expects the original radian angle convention.')
    if compiler.get('strippath', 'false').lower() == 'true':
        raise ValueError('strippath=true is not supported by this patch.')
    if root.find("./custom/text[@name='moss_collision_candidate']") is not None:
        raise ValueError('Candidate already applied; start from the host model.')
    before = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(before)
    mujoco.mj_forward(before, data)
    rover = root.find(f".//body[@name='{prefix}rover']")
    if rover is None or rover.find('inertial') is None:
        raise ValueError('Expected named rover body with explicit inertia.')
    # Check the actual referenced V0.4 bin in the rover frame, including all rotations.
    bid = before.body(prefix + 'rover').id
    gid = before.geom(prefix + 'v04_030_bin').id
    points = ((compiled_mesh_world(before, data, gid) - data.xpos[bid])
              @ data.xmat[bid].reshape(3, 3))
    np.testing.assert_allclose([points.min(0), points.max(0)],
                              [[-.179, -.116, .1089982], [.005, .116, .26000008]],
                              atol=2e-6, rtol=0,
                              err_msg='Different bin/placement: review instead of guessing.')
    changes = []
    for suffix, pos, angle in [
        ('x-1', '-.168 0 .186', '0 -.096 0'),
        ('x1', '-.006 0 .186', '0 .096 0'),
        ('y-1', '-.087 -.1 .186', '.092 0 0'),
        ('y1', '-.087 .1 .186', '-.092 0 0'),
    ]:
        geom = rover.find(f"geom[@name='{prefix}bin_{suffix}']")
        if geom is None:
            raise ValueError(f'Missing original bin_{suffix} collider')
        old = dict(geom.attrib)
        for key in (*POSE_KEYS, 'mesh', 'hfield'):
            geom.attrib.pop(key, None)
        geom.set('type', 'box')
        geom.set('pos', pos)
        geom.set('euler', angle)
        geom.set('size', '.003 .11 .075' if suffix.startswith('x') else '.09 .003 .075')
        changes.append({'geom': geom.get('name'), 'before': old, 'after': dict(geom.attrib)})

    mapped = []
    for link in ['shoulder_link', 'upper_arm_link', 'lower_arm_link', 'wrist_link']:
        body = rover.find(f".//body[@name='{prefix}{link}']")
        if body is None or body.find('inertial') is None:
            raise ValueError(f'{link} needs explicit inertia to preserve dynamics.')
        proxy = body.find(f"geom[@name='{prefix}{link}_proxy']")
        if proxy is None or proxy.get('type') != 'capsule':
            raise ValueError(f'Expected capsule for {link}; review custom geometry manually.')
        for elem in root.iter():
            if elem is not proxy and proxy.get('name') in [elem.get(k) for k in ('geom', 'geom1', 'geom2', 'objname')]:
                raise ValueError(f'{proxy.get("name")} is referenced externally; review pair/sensor semantics first.')
        visuals = [g for g in body.findall('geom')
                   if g.get('name', '').startswith(prefix + 'visual_' + link + '_')
                   and g.get('type') == 'mesh' and g.get('contype') == '0'
                   and g.get('conaffinity') == '0']
        if not visuals:
            raise ValueError(f'No original visual parts on {link}.')
        body.remove(proxy)
        for i, visual in enumerate(visuals):
            name = proxy.get('name') if i == 0 else f'{prefix}{link}_part_{i}_collision'
            if any(g.get('name') == name for g in root.iter('geom')):
                raise ValueError(f'Name conflict: {name}')
            geom = clone_collision(proxy, visual, name)
            body.append(geom)
            mapped.append({'collision': name, 'visual': visual.get('name')})
        changes.append({'replaced_capsule': proxy.get('name'), 'convex_parts': len(visuals)})

    for sign, side in [('-1', 'right'), ('1', 'left')]:
        old = rover.find(f"geom[@name='{prefix}track_{sign}']")
        visual = rover.find(f"geom[@name='{prefix}v04_belt_{side}']")
        if old is None or visual is None or visual.get('type') != 'mesh':
            raise ValueError('Expected original track collider and V0.4 belt visual.')
        new = clone_collision(old, visual, old.get('name'))
        rover.remove(old)
        rover.append(new)
        mapped.append({'collision': new.get('name'), 'visual': visual.get('name')})
        changes.append({'replaced_track_box': old.get('name'), 'with': 'V0.4 belt convex hull'})

    if self_contacts:
        arm = rover.find(f".//body[@name='{prefix}base_link']")
        for geom in arm.iter('geom'):
            # Existing physical gripper geoms and the replacement arm shapes.
            if geom.get('contype') == '2' and geom.get('conaffinity') == '1':
                geom.set('conaffinity', '3')
        changes.append({'diagnostic_self_contacts': '2/3 masks; no new exclusions; parent filtering retained'})
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'text', name='moss_collision_candidate',
                  data='diagnostic-v1; convex-per-part; NOT hardware validated')
    ET.indent(root)
    xml = ET.tostring(root, encoding='unicode')
    # Resolve asset paths in a throwaway tree for validation before writing.
    validation = copy.deepcopy(root)
    compiler = validation.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(validation, 'compiler')
    for kind in ('meshdir', 'texturedir'):
        directory = compiler.get(kind, compiler.get('assetdir', '.'))
        compiler.set(kind, str((source.parent / directory).resolve()))
    after = mujoco.MjModel.from_xml_string(ET.tostring(validation, encoding='unicode'))
    verified = []
    for name in INVARIANTS:
        np.testing.assert_array_equal(getattr(before, name), getattr(after, name), err_msg=name)
        verified.append(name)
    for name in ('timestep', 'integrator', 'cone', 'iterations', 'gravity'):
        np.testing.assert_array_equal(getattr(before.opt, name), getattr(after.opt, name), err_msg=name)
    ad = mujoco.MjData(after)
    mujoco.mj_forward(after, ad)
    for pair in mapped:
        cg, vg = after.geom(pair['collision']).id, after.geom(pair['visual']).id
        np.testing.assert_allclose(compiled_mesh_world(after, ad, cg),
                                   compiled_mesh_world(after, ad, vg), atol=1e-9, rtol=0)
    with destination.open('x') as f:
        f.write(xml + '\n')
    return {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'candidate_sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
            'mujoco_version': mujoco.__version__, 'self_contacts': self_contacts,
            'changes': changes, 'collision_visual_pairs': mapped,
            'unchanged_compiled_fields': verified,
            'collision_vertices_match_visual_parts': True,
            'limitations': ['Per-part convex hulls can overfill recesses and openings.',
                            'Hull, cover, arm base and gripper collision geometry remain legacy.',
                            'No new exclusions; self-contact variant is diagnostic only.',
                            'No actuator, controller, calibration, mass or camera changes.',
                            'No successful pickup, tuck or hardware safety claim.']}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--prefix', default='')
    p.add_argument('--self-contacts', action='store_true', help='Diagnostic arm self-contact mask variant')
    p.add_argument('--report', type=Path, required=True)
    args = p.parse_args()
    if args.report.exists():
        p.error('Report exists; choose a new filename.')
    result = prepare(args.input, args.output, args.prefix, args.self_contacts)
    with args.report.open('x') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    print(json.dumps({'created': args.output.name, 'report': args.report.name,
                      'status': 'diagnostic candidate, not validated for hardware'}))
