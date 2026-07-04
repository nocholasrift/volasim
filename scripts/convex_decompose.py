#!/usr/bin/env python3
"""Convex-decompose an OBJ mesh into a collision mesh for volasim.

Produces a ``<name>_cd.obj`` whose separate ``o geometry_N`` objects are each a
convex hull. This is the exact format volasim's ``MeshRenderable::readConvexDecomp``
expects for a ``<geometry ... decomp_file="...">`` file: Assimp loads every object
as its own mesh, and each becomes one ``JPH::ConvexHullShapeSettings`` in a
``StaticCompoundShapeSettings`` (see src/simulation/physics_interface.cpp).

Requires coacd and trimesh, which live in the venv at ~/Programs/venv. Run with
that interpreter, e.g.:

    ~/Programs/venv/bin/python scripts/convex_decompose.py \
        definitions/meshes/realsense_d435i.obj

Then point the sensor/vehicle XML at the result:

    <geometry type="mesh" model_file="./definitions/meshes/realsense_d435i.obj"
              decomp_file="./definitions/meshes/realsense_d435i_cd.obj"/>
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import trimesh
    import coacd
except ImportError as exc:  # pragma: no cover - environment guard
    sys.exit(
        f"Missing dependency: {exc.name}. Run this with the coacd venv, e.g.\n"
        "  ~/Programs/venv/bin/python scripts/convex_decompose.py <input.obj>"
    )


def load_as_single_mesh(path: Path) -> trimesh.Trimesh:
    """Load an OBJ as one Trimesh, concatenating multi-object scenes."""
    loaded = trimesh.load(path, force="mesh")
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(loaded.dump())
    if not isinstance(loaded, trimesh.Trimesh) or loaded.faces.shape[0] == 0:
        sys.exit(f"'{path}' did not load as a triangle mesh.")
    return loaded


def decompose(mesh: trimesh.Trimesh, args) -> list[trimesh.Trimesh]:
    """Run coacd and return one Trimesh per convex part (with a debug color)."""
    coacd.set_log_level("warn" if not args.verbose else "info")
    parts = coacd.run_coacd(
        coacd.Mesh(mesh.vertices, mesh.faces),
        threshold=args.threshold,
        max_convex_hull=args.max_convex_hull,
        preprocess_mode=args.preprocess,
        resolution=args.resolution,
        seed=args.seed,
    )

    # Deterministic per-part colors so the hulls are distinguishable if rendered.
    rng = np.random.default_rng(args.seed)
    hulls = []
    for verts, faces in parts:
        hull = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        color = np.concatenate([rng.integers(80, 230, size=3), [255]])
        hull.visual.vertex_colors = np.tile(color, (len(hull.vertices), 1))
        hulls.append(hull)
    return hulls


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Convex-decompose an OBJ into a volasim _cd.obj collision mesh.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("input", type=Path, help="Input mesh (e.g. .obj)")
    ap.add_argument(
        "-o", "--output", type=Path, default=None,
        help="Output path (default: <input_stem>_cd.obj next to the input).",
    )
    ap.add_argument(
        "-t", "--threshold", type=float, default=0.05,
        help="coacd concavity threshold; lower = tighter fit, more hulls (0.01-1).",
    )
    ap.add_argument(
        "--max-convex-hull", type=int, default=-1,
        help="Cap on number of hulls (-1 = no cap).",
    )
    ap.add_argument(
        "--preprocess", choices=["auto", "on", "off"], default="auto",
        help="coacd manifold preprocessing. Use 'off' only for clean manifolds.",
    )
    ap.add_argument("--resolution", type=int, default=2000,
                    help="Voxel sampling resolution for coacd.")
    ap.add_argument("--seed", type=int, default=0, help="RNG seed (reproducible).")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Verbose coacd logging.")
    args = ap.parse_args()

    if not args.input.exists():
        sys.exit(f"Input not found: {args.input}")

    out = args.output or args.input.with_name(f"{args.input.stem}_cd.obj")

    mesh = load_as_single_mesh(args.input)
    print(f"Loaded {args.input}: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")

    hulls = decompose(mesh, args)
    n_verts = sum(len(h.vertices) for h in hulls)
    n_faces = sum(len(h.faces) for h in hulls)
    print(f"coacd -> {len(hulls)} convex hulls, {n_verts} verts, {n_faces} faces")

    # One geometry per hull; trimesh writes each as a separate `o geometry_N`
    # object, which is what Assimp/Jolt consume as individual convex shapes.
    scene = trimesh.Scene()
    for i, hull in enumerate(hulls):
        scene.add_geometry(hull, geom_name=f"geometry_{i}")

    out.parent.mkdir(parents=True, exist_ok=True)
    scene.export(out)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
