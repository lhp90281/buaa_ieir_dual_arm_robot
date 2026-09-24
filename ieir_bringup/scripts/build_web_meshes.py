#!/usr/bin/env python3
"""Developer-only visual LOD generator. Runtime ROS/Web UI needs no mesh tools.

pip install trimesh==4.7.4 fast-simplification==0.1.13
python build_web_meshes.py --description ../../description --output ../web/meshes
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.description.resolve()
    urdf = ET.parse(root / 'dual_arm_support/urdf/dual_arm_robot_plug.urdf')
    sources = sorted({v.get('filename').removeprefix('package://')
                      for v in urdf.findall('.//visual/geometry/mesh')})
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for relative in sources:
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise ValueError('Mesh outside description')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        original = trimesh.load_mesh(path, process=True)
        count = len(original.faces)
        target = min(count, max(800, min(6000, int(count * 0.12))))
        mesh = original
        while target < count:
            candidate = original.simplify_quadric_decimation(face_count=target)
            if (len(candidate.faces) and np.isfinite(candidate.vertices).all()
                    and np.max(np.abs(candidate.bounds - original.bounds)) <= 0.001):
                mesh = candidate
                break
            target *= 2  # Preserve small features rather than accepting a distorted silhouette.
        output = digest + '.stl'
        mesh.export(args.output / output, file_type='stl')
        manifest[digest] = dict(file=output, source=relative, source_faces=count, faces=len(mesh.faces))
        print(f'{relative}: {count} -> {len(mesh.faces)} faces', flush=True)
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
