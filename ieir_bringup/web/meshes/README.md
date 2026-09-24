# Web display meshes

Historical experiment, NOT selected by the console. Full original STL is now
used because decimation damaged thin-wall/hole appearance. These assets and the
generator remain only for offline investigation, not the default display path.

Generated from this repository's URDF-referenced STL files. These are visual-only
LODs, never collision, inertia or control assets. Original description files are
unchanged. The manifest records the source SHA-256 for offline comparisons.

The developer-only `scripts/build_web_meshes.py` uses
[fast-simplification](https://github.com/pyvista/fast-simplification) via trimesh.
Generation validates finite vertices and bounding-box changes of at most 0.001
source units, reducing less aggressively where necessary. This is not a full
surface-distance or collision-geometry guarantee.

To regenerate after changing CAD assets, use a separate Python environment with
trimesh 4.7.4 and fast-simplification 0.1.13, then from the repository root:

```bash
python ieir_bringup/scripts/build_web_meshes.py \
  --description description --output ieir_bringup/web/meshes
```

The normal ROS/Web UI installation requires none of these developer dependencies.
The generated meshes retain the provenance and license of their source meshes.
