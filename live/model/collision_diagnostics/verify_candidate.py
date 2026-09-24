#!/usr/bin/env python3
"""Reproduce the diagnostic geometry patch and integration checks in a temp directory.

No source writes, training or hardware I/O. Input: a flattened V0.4 MOSS XML.
"""
import argparse
import copy
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from prepare_candidate import prepare


def pose_probe(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    results = {}
    for pose in ('home', 'reported_tuck'):
        home = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'home')
        if home < 0:
            raise ValueError('Pose probe needs the original home keyframe.')
        mujoco.mj_resetDataKeyframe(m, d, home)
        if pose == 'reported_tuck':
            for joint, value in zip(
                ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
                [-.8193, .304, 1.0968, 1.5959, .6327],
            ):
                d.qpos[m.jnt_qposadr[m.joint(joint).id]] = value
        mujoco.mj_forward(m, d)
        pairs = {}
        for contact in d.contact:
            if contact.dist < -1e-6:
                pair = ' / '.join(sorted([m.geom(contact.geom1).name, m.geom(contact.geom2).name]))
                pairs[pair] = max(pairs.get(pair, 0), float(-contact.dist * 1000))
        results[pose] = {'penetrating_pairs_max_depth_mm': pairs,
                         'finger_positions_m': {name: float(d.qpos[m.jnt_qposadr[m.joint(name).id]])
                                                for name in ('finger_left', 'finger_right')},
                         'scope': 'Kinematic pose only; no trajectory, settling or hardware validation.'}
    return results


def verify(source):
    source = Path(source).resolve()
    root = ET.parse(source).getroot()
    c = root.find('compiler')
    for name in ('meshdir', 'texturedir'):
        c.set(name, str((source.parent / c.get(name, c.get('assetdir', '.'))).resolve()))
    out = {'mujoco_version': mujoco.__version__, 'tests': {}, 'poses': {}}
    with tempfile.TemporaryDirectory(prefix='moss_collision_check_') as tmp:
        tmp = Path(tmp)
        src = tmp / 'host.xml'
        ET.ElementTree(root).write(src)
        for label, self_contacts in [('geometry', False), ('self_contacts', True)]:
            dst = tmp / f'{label}.xml'
            report = prepare(src, dst, self_contacts=self_contacts)
            out['tests'][label] = {'unchanged_fields': report['unchanged_compiled_fields'],
                                    'visual_alignment': report['collision_vertices_match_visual_parts']}
            out['poses'][label] = pose_probe(dst)
        out['poses']['original'] = pose_probe(src)
        try:
            prepare(src, tmp / 'geometry.xml')
        except ValueError:
            out['tests']['refuses_overwrite'] = True
        else:
            raise AssertionError('Overwrite was accepted.')

        displaced = copy.deepcopy(root)
        displaced.find(".//geom[@name='v04_030_bin']").set('pos', '0 0 .01')
        bad_source, bad_output = tmp / 'displaced_bin.xml', tmp / 'bad_candidate.xml'
        ET.ElementTree(displaced).write(bad_source)
        try:
            prepare(bad_source, bad_output)
        except AssertionError:
            assert not bad_output.exists()
            out['tests']['refuses_different_bin_pose_before_writing'] = True
        else:
            raise AssertionError('Unexpected bin placement was silently accepted.')

        # Deliberately perturb the host to ensure its settings survive, instead
        # of merely comparing a second copy of our original fixture.
        synthetic = copy.deepcopy(root)
        rover = synthetic.find(".//body[@name='rover']")
        inertial = rover.find('inertial')
        inertial.set('mass', '3.5')
        ET.SubElement(rover, 'camera', name='audit_host_camera', pos='.156 0 .075',
                      xyaxes='0 -1 0 0 0 1', fovy='62')
        rover.insert(0, ET.Element('joint', name='audit_base_y', type='slide', axis='0 1 0'))
        rover.insert(1, ET.Element('joint', name='audit_base_yaw', type='hinge', axis='0 0 1'))
        for key in synthetic.findall('./keyframe/key'):
            if 'qpos' in key.attrib:
                key.set('qpos', '0 0 ' + key.get('qpos'))
            if 'qvel' in key.attrib:
                key.set('qvel', '0 0 ' + key.get('qvel'))
        eq = synthetic.find('equality')
        if eq is None:
            eq = ET.SubElement(synthetic, 'equality')
        ET.SubElement(eq, 'joint', name='audit_finger_coupling',
                      joint1='finger_left', joint2='finger_right', polycoef='0 1 0 0 0')
        path = tmp / 'modified_host.xml'
        ET.ElementTree(synthetic).write(path)
        result = prepare(path, tmp / 'modified_host_candidate.xml')
        patched = mujoco.MjModel.from_xml_path(str(tmp / 'modified_host_candidate.xml'))
        assert patched.body_mass[patched.body('rover').id] == 3.5
        assert patched.camera('audit_host_camera').id >= 0
        assert patched.joint('audit_base_y').id >= 0
        assert patched.joint('audit_base_yaw').id >= 0
        assert patched.equality('audit_finger_coupling').id >= 0
        out['tests']['modified_host_preserved'] = {
            'mass_kg': 3.5, 'extra_base_joints': 2, 'camera_preserved': True,
            'finger_equality_preserved': True,
            'unchanged_fields': result['unchanged_compiled_fields']}
    return out


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    results = verify(args.input)
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps({'integration_checks': 'passed',
                      'original_home_contact_pairs': len(results['poses']['original']['home']['penetrating_pairs_max_depth_mm']),
                      'candidate_home_contact_pairs': len(results['poses']['self_contacts']['home']['penetrating_pairs_max_depth_mm']),
                      'candidate_tuck_contact_pairs': len(results['poses']['self_contacts']['reported_tuck']['penetrating_pairs_max_depth_mm'])}))
