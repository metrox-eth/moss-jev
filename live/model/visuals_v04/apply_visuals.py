#!/usr/bin/env python3
"""Replace MOSS rover visuals while preserving the host's physical model.

Run on a standalone, flattened MJCF before prefixing/attaching it to a room.
Standard library only. Writes a new XML beside the input; never overwrites it.
The adjacent manifest and meshes must remain available at their relative path.
"""
from pathlib import Path
import argparse
import json
import os
import re
import xml.etree.ElementTree as ET


def apply(source, destination, package=None, body_name="rover", prefix=""):
    source = Path(source).resolve()
    destination = Path(destination).resolve()
    package = Path(package or Path(__file__).parent).resolve()
    if source == destination or destination.exists():
        raise ValueError("Choose a new output filename; existing files are not overwritten.")
    if source.parent != destination.parent:
        raise ValueError("Output must be beside the input XML to preserve its asset paths.")
    root = ET.parse(source).getroot()
    if root.findall(".//include"):
        raise ValueError("Flatten includes first, or merge the provided visual fragments manually.")
    bodies = [b for b in root.iter("body") if b.get("name") == body_name]
    if len(bodies) != 1:
        raise ValueError(f"Expected one body named {body_name!r}; use --body for a renamed body.")
    body = bodies[0]
    if body.find("inertial") is None:
        raise ValueError("The rover needs its explicit inertial first; this update does not set mass/COM.")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    if compiler.get("strippath", "false").lower() == "true":
        raise ValueError("strippath=true is unsupported; merge the asset fragments explicitly.")
    meshdir = compiler.get("meshdir", compiler.get("assetdir", "."))
    meshbase = (source.parent / meshdir).resolve()
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    removed, old_meshes = [], set()
    for geom in list(body.findall("geom")):
        name = geom.get("name", "")
        local = name[len(prefix):] if name.startswith(prefix) else name
        old = re.fullmatch(r"visual_body_\d+|tread_-?1_\d+", local)
        if not old:
            continue
        if any(geom.get(k) != "0" for k in ("mass", "contype", "conaffinity")):
            raise ValueError(f"Refusing to remove {name}: not explicitly visual-only.")
        removed.append(name)
        if geom.get("mesh"):
            old_meshes.add(geom.get("mesh"))
        body.remove(geom)
    if not removed:
        raise ValueError("No original MOSS rover visuals found. Merge the fragments manually for a different asset layout.")
    used = {e.get("mesh") for e in root.iter() if e.get("mesh")}
    for asset in list(assets):
        if asset.tag == "mesh" and asset.get("name") in old_meshes - used:
            assets.remove(asset)
    manifest = json.loads((package / "manifest.json").read_text())
    taken = {e.get("name") for e in root.iter() if e.get("name")}
    for row in manifest["meshes"]:
        name = prefix + row["name"]
        if name in taken:
            raise ValueError(f"Name collision: {name}")
        mesh_path = package / row["file"]
        if not mesh_path.is_file():
            raise FileNotFoundError(mesh_path)
        ET.SubElement(assets, "mesh", name=name,
                      file=os.path.relpath(mesh_path, meshbase), inertia="shell")
        ET.SubElement(body, "geom", name=name, type="mesh", mesh=name,
                      pos="0 0 0", quat="1 0 0 0", rgba=" ".join(map(str, row["rgba"])),
                      group="2", mass="0", contype="0", conaffinity="0")
    # Deliberately do not touch cameras, collisions, solver, joint/actuator order,
    # keyframes, drive logic, or any inertial. The host owns those decisions.
    ET.indent(root)
    ET.ElementTree(root).write(destination, encoding="utf-8", xml_declaration=True)
    return {"removed_visual_geoms": len(removed), "added_visual_geoms": len(manifest["meshes"]),
            "output": str(destination)}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--body", default="rover")
    p.add_argument("--prefix", default="", help="Prefix already present on original visual names")
    args = p.parse_args()
    try:
        print(json.dumps(apply(args.input, args.output, body_name=args.body, prefix=args.prefix), indent=2))
    except (ValueError, FileNotFoundError) as error:
        p.exit(2, f"{error}\n")
