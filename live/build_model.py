"""SO-101 URDF joint geometry, inertias and limits; simplified MOSS contact shapes.

The parallel gripper is an idealized NC90 with two independent position servos.
It is a simulation approximation, not a calibrated model of the physical gripper.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

def nums(a):
    return ' '.join(f'{float(x):.9g}' for x in a)

def pose(el):
    if el is None:
        return {}
    q = Rotation.from_euler('xyz', np.fromstring(el.get('rpy', '0 0 0'), sep=' ')).as_quat()
    return {'pos': el.get('xyz', '0 0 0'), 'quat': nums([q[3], *q[:3]])}

def build():
    src = ET.parse(ROOT / 'model/so101.urdf').getroot()
    links = {e.get('name'): e for e in src.findall('link')}
    joints = [e for e in src.findall('joint') if e.find('parent') is not None and e.get('name') != 'gripper']
    root = ET.Element('mujoco', model='MOSS mobile manipulation')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast', cone='elliptic', iterations='80')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'joint', damping='0.15', armature='0.015')
    ET.SubElement(default, 'geom', friction='0.9 0.005 0.0001', solref='0.008 1', solimp='0.95 0.99 0.001', condim='4')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='ground', type='plane', size='2 2 .05', rgba='.05 .09 .13 1')
    rover = ET.SubElement(world, 'body', name='rover')
    ET.SubElement(rover, 'joint', name='base_x', type='slide', axis='1 0 0', range='0 .8', damping='40')
    def box(name, pos, size, rgba='.16 .23 .28 1', euler=None):
        ET.SubElement(rover, 'geom', name=name, type='box', pos=nums(pos), size=nums(size), rgba=rgba, **({'euler':nums(euler)} if euler else {}))
    # Contact proxies are deliberately simple; visual CAD stays intact.
    box('hull', [-.015, 0, .065], [.13, .093, .035])
    for side in [-1, 1]:
        box('track_' + str(side), [0, side*.122, .049], [.14, .019, .045])
    # Bin interior: open at top; no invisible collision across its opening.
    box('bin_floor', [-.087, 0, .1102], [.077, .096, .002], '.25 .47 .59 1')
    for s in [-1, 1]:
        box('bin_x'+str(s), [-.087+s*.081, 0, .186], [.003, .110, .075], '.25 .47 .59 1', [0,s*.096,0])
        box('bin_y'+str(s), [-.087, s*.100, .186], [.09, .003, .075], '.25 .47 .59 1', [-s*.092,0,0])
    base = ET.SubElement(rover, 'body', name='base_link', pos='.085 -.03 .1004', euler='0 0 1.308996939')
    bodies = {'base_link': base}
    pending = joints.copy()
    while pending:
        for j in pending.copy():
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in bodies:
                continue
            b = ET.SubElement(bodies[parent], 'body', name=child, **pose(j.find('origin')))
            bodies[child] = b
            if j.get('type') != 'fixed':
                lim = j.find('limit')
                ET.SubElement(b, 'joint', name=j.get('name'), axis=j.find('axis').get('xyz'), range=lim.get('lower')+' '+lim.get('upper'))
            pending.remove(j)
    for name, body in bodies.items():
        inertia = links[name].find('inertial')
        if inertia is not None and name != 'gripper_frame_link':
            v = inertia.find('inertia')
            ET.SubElement(body, 'inertial', pos=inertia.find('origin').get('xyz'), mass=inertia.find('mass').get('value'),
                          fullinertia=' '.join(v.get(k) for k in ['ixx','iyy','izz','ixy','ixz','iyz']))
        # Coarse link collision capsules; arm-arm self contact omitted in this first model.
        if name not in ['base_link', 'gripper_frame_link', 'gripper_link']:
            children = [j for j in joints if j.find('parent').get('link') == name]
            for k, j in enumerate(children):
                p = np.fromstring(j.find('origin').get('xyz'), sep=' ')
                if np.linalg.norm(p) > .025:
                    ET.SubElement(body, 'geom', name=name+'_proxy', type='capsule', fromto='0 0 0 '+nums(p), size='.013',
                                  contype='2', conaffinity='1', group='3', rgba='.95 .5 .4 .25')
    tool = bodies['gripper_frame_link']
    # NC90 frame and rail axis from the existing presentation adaptation matrix.
    center=np.array([-.0079,.000218,0]);axis=np.array([.75041,.66097,0]);axis/=np.linalg.norm(axis)
    rz=nums([0,0,np.arctan2(axis[1],axis[0])])
    ET.SubElement(tool, 'geom', name='palm', type='box', pos=nums(center+[0,0,-.059]), euler=rz, size='.060 .017 .008', mass='.1', rgba='.21 .55 .49 1', contype='2', conaffinity='1')
    ET.SubElement(tool, 'geom', name='gripper_mount', type='box', pos=nums(center+[0,0,-.08]), size='.018 .017 .018', mass='.05', rgba='.12 .2 .25 1', contype='2', conaffinity='1')
    ET.SubElement(tool, 'site', name='tcp', pos=nums(center+[0,0,-.014]), size='.002')
    for s, name in [(1, 'left'), (-1, 'right')]:
        f = ET.SubElement(tool, 'body', name='finger_'+name, pos=nums(center+s*.008*axis))
        ET.SubElement(f, 'joint', name='finger_'+name, type='slide', axis=nums(s*axis), range='0 .041', damping='2', armature='.002')
        ET.SubElement(f, 'geom', name='pad_'+name, type='box', pos='0 0 -.014', euler=rz, size='.004 .015 .018', mass='.025',
                      friction='1.4 .01 .001', rgba='.32 .85 .72 1', contype='2', conaffinity='1')
    obj = ET.SubElement(world, 'body', name='litter', pos='.315 0 .026')
    ET.SubElement(obj, 'freejoint', name='litter_free')
    ET.SubElement(obj, 'geom', name='can', type='cylinder', size='.033 .0575', mass='.018', rgba='1 .58 .49 1', contype='1', conaffinity='3')
    act = ET.SubElement(root, 'actuator')
    for n in JOINTS:
        ET.SubElement(act, 'position', name=n, joint=n, kp='70', kv='3', forcerange='-2.2 2.2', inheritrange='1')
    for n in ['left','right']:
        ET.SubElement(act, 'position', name='finger_'+n, joint='finger_'+n, kp='700', kv='8', forcerange='-8 8', ctrlrange='0 .041')
    ET.SubElement(act, 'position', name='base_x', joint='base_x', kp='3000', kv='300', ctrlrange='0 .8', forcerange='-200 200')
    ET.indent(root)
    ET.ElementTree(root).write(ROOT/'model/moss.xml', encoding='unicode')

if __name__ == '__main__':
    build()
