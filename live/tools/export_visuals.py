"""Export the existing presentation GLB into portable, non-colliding MJCF visuals."""
from pathlib import Path
import copy
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from physics import Physics


def decimate(mesh, limit=5000):
    if len(mesh.faces) <= limit:
        return mesh.copy()
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.asarray(mesh.vertices), deep=True))
    cells = vtk.vtkCellArray()
    cells.SetCells(len(mesh.faces), numpy_to_vtkIdTypeArray(
        np.c_[np.full(len(mesh.faces), 3), mesh.faces].astype(np.int64).ravel(), deep=True))
    poly = vtk.vtkPolyData(); poly.SetPoints(points); poly.SetPolys(cells)
    filt = vtk.vtkQuadricDecimation(); filt.SetInputData(poly)
    filt.SetTargetReduction(1 - limit / len(mesh.faces)); filt.Update()
    result = filt.GetOutput()
    return trimesh.Trimesh(vtk_to_numpy(result.GetPoints().GetData()),
        vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:], process=False)


def main():
    p = Physics()
    tree = ET.parse(ROOT / 'model/moss.xml'); root = tree.getroot()
    root.set('model', 'MOSS Jev V0.2 - visual scene')
    root.find('compiler').set('meshdir', 'meshes')
    # Preserve the original inertial properties before adding visual-only geoms.
    for body in root.iter('body'):
        if body.find('inertial') is None:
            i = p.m.body(body.get('name')).id
            ET.SubElement(body, 'inertial', mass=str(p.m.body_mass[i]),
                pos=' '.join(map(str, p.m.body_ipos[i])),
                quat=' '.join(map(str, p.m.body_iquat[i])),
                diaginertia=' '.join(map(str, p.m.body_inertia[i])))
    for g in root.find('worldbody').iter('geom'):
        if g.get('name') not in ('ground', 'can'):
            g.set('group', '3'); g.set('rgba', '0.2 0.3 0.4 0')
    asset = ET.SubElement(root, 'asset')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', diffuse='.8 .8 .8')
    ET.SubElement(world, 'camera', name='overview', pos='.85 -.95 .72',
        xyaxes='.79 .61 0 -.29 .38 .88')
    bodies = {b.get('name'): b for b in root.iter('body')}
    scene = trimesh.load(ROOT / 'public/replay/assets/moss.glb', force='scene', process=False)
    meshdir = ROOT / 'model/meshes'; meshdir.mkdir(exist_ok=True)
    # Child meshes in this GLB are expressed in their owning arm link frame.
    owners = {child: parent for parent, child, _ in scene.graph.to_edgelist()}
    centers = {side: scene.geometry[scene.graph[name][1]].bounds.mean(axis=0)
        for side, name in [('left', 'grip_159'), ('right', 'grip_160')]}
    manifest = []
    for node in sorted(scene.graph.nodes_geometry):
        matrix, name = scene.graph[node]
        source = scene.geometry[name]
        parent = owners.get(node, 'world')
        body = parent[4:] if parent.startswith('arm_') else 'rover'
        mesh = decimate(source)
        rgba = np.asarray(source.visual.material.baseColorFactor, dtype=float) / 255
        if node in ('grip_133', 'grip_159', 'grip_134', 'grip_160'):
            side = 'left' if node in ('grip_133', 'grip_159') else 'right'
            # Same mapping as live.js: centre the visual pad on its contact proxy.
            mesh.vertices += np.array([0, 0, -.014]) - centers[side]
            body = 'finger_' + side
        elif not parent.startswith('arm_'):
            mesh.apply_transform(matrix)
        file = node + '.stl'
        mesh.export(meshdir / file)
        ET.SubElement(asset, 'mesh', name='visual_' + node, file=file, inertia='shell')
        ET.SubElement(bodies[body], 'geom', name='visual_' + node, type='mesh',
            mesh='visual_' + node, rgba=' '.join(map(str, rgba)),
            contype='0', conaffinity='0', group='2', mass='0')
        manifest.append({'mesh': file, 'body': body, 'source_node': node,
            'source_triangles': len(source.faces), 'triangles': len(mesh.faces)})
    # Static visual tread blocks: locomotion is still the original slide joint.
    for side in (-1, 1):
        length = .5 + 2 * np.pi * .036
        for i in range(68):
            s = i / 68 * length
            if s < .25: x, z, a = -.125 + s, .036, 0
            elif s < .25 + np.pi * .036:
                a = np.pi/2 - (s-.25)/.036
                x, z = .125+.036*np.cos(a), .036*np.sin(a); a -= np.pi/2
            elif s < .5 + np.pi * .036:
                x, z, a = .125-(s-.25-np.pi*.036), -.036, 0
            else:
                a = -np.pi/2-(s-.5-np.pi*.036)/.036
                x, z = -.125+.036*np.cos(a), .036*np.sin(a); a -= np.pi/2
            ET.SubElement(bodies['rover'], 'geom', name=f'tread_{side}_{i}', type='box',
                size='.00425 .018 .004', pos=f'{x} {side*.121} {z+.041}',
                euler=f'0 {-a} 0', rgba='.18 .23 .27 1',
                contype='0', conaffinity='0', group='2', mass='0')
    keys = ET.SubElement(root, 'keyframe')
    ET.SubElement(keys, 'key', name='home', qpos=' '.join(map(str, p.d.qpos)),
        ctrl=' '.join(map(str, p.d.ctrl)))
    ET.indent(root); tree.write(ROOT / 'model/moss_visual.xml', encoding='unicode')
    # Robot-only file for composition into somebody else's stage.
    robot = copy.deepcopy(root); robot.set('model', 'MOSS Jev V0.2 - robot only')
    w = robot.find('worldbody')
    for e in list(w):
        if e.tag != 'body' or e.get('name') != 'rover': w.remove(e)
    q = np.delete(p.d.qpos, np.s_[p.oq:p.oq+7])
    robot.find('keyframe/key').set('qpos', ' '.join(map(str, q)))
    ET.indent(robot); ET.ElementTree(robot).write(ROOT / 'model/moss_robot.xml', encoding='unicode')
    (ROOT / 'model/mesh_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'meshes': len(manifest), 'triangles': sum(m['triangles'] for m in manifest)}))


if __name__ == '__main__': main()
