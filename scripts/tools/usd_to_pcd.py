#!/usr/bin/env python3
"""
usd_to_pcd.py
-------------
通过直接采样三角形面，将 USD 场景转换为点云。

使用随机重心坐标，并按面积比例对每个三角形采样。因此，密集几何体会获得
更多采样点，同时不会遗漏任何表面。不考虑传感器位置，不进行光线投射，
也不处理遮挡——仅对表面进行采样。

依赖项
------------
  pip install usd-core numpy

用法
-----
  python usd_to_pcd.py <input.usd[a|z]> [options]

示例
--------
  python usd_to_pcd.py scene.usda                         # 约 50 万个点，PLY
  python usd_to_pcd.py scene.usda -n 1000000 -o map.pcd   # 100 万个点，PCD
  python usd_to_pcd.py scene.usda --normals               # 包含法向量
  python usd_to_pcd.py scene.usda --density 100           # 每平方单位 100 个点
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom


# ---------------------------------------------------------------------------
# USD 数据提取
# ---------------------------------------------------------------------------

def _world_matrix(prim) -> np.ndarray:
    """
    USD 使用行主序变换：world_pt = [x,y,z,1] @ M。
    平移量位于最后一行（M[3, :3]），而不是最后一列。
    因此正确的应用方式为：pts_world = pts_h @ M（不是 @ M.T）。
    """
    xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(xform).reshape(4, 4)


def extract_triangles(usd_path: str):
    """返回世界坐标系下的三角形和面法向量。"""
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        raise RuntimeError(f"Cannot open: {usd_path}")

    all_v0, all_v1, all_v2 = [], [], []

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh    = UsdGeom.Mesh(prim)
        pts_a   = mesh.GetPointsAttr()
        cnt_a   = mesh.GetFaceVertexCountsAttr()
        idx_a   = mesh.GetFaceVertexIndicesAttr()
        if not (pts_a and cnt_a and idx_a):
            continue

        pts     = np.array(pts_a.Get(),  dtype=np.float32)
        counts  = np.array(cnt_a.Get(),  dtype=np.int32)
        indices = np.array(idx_a.Get(),  dtype=np.int32)
        if pts is None or len(pts) == 0:
            continue

        # 应用世界变换——USD 为行主序：pts_world = [x,y,z,1] @ M
        M    = _world_matrix(prim)
        ones = np.ones((len(pts), 1), dtype=np.float32)
        pts  = (np.hstack([pts, ones]) @ M)[:, :3]

        # 对面进行三角剖分（扇形三角剖分）
        i = 0
        for count in counts:
            face = indices[i:i + count]
            for k in range(1, count - 1):
                all_v0.append(pts[face[0]])
                all_v1.append(pts[face[k]])
                all_v2.append(pts[face[k + 1]])
            i += count

    if not all_v0:
        raise RuntimeError("No mesh geometry found in USD stage.")

    v0 = np.array(all_v0, dtype=np.float32)
    v1 = np.array(all_v1, dtype=np.float32)
    v2 = np.array(all_v2, dtype=np.float32)

    # 面法向量和面积
    e1      = v1 - v0
    e2      = v2 - v0
    crosses = np.cross(e1, e2)
    areas   = np.linalg.norm(crosses, axis=1) / 2.0
    normals = crosses / (np.linalg.norm(crosses, axis=1, keepdims=True) + 1e-12)

    print(f"[USD]  {len(v0):,} triangles | surface area ≈ {areas.sum():.2f} units²")
    return v0, v1, v2, normals.astype(np.float32), areas.astype(np.float32)


# ---------------------------------------------------------------------------
# 面采样
# ---------------------------------------------------------------------------

def sample_points(v0, v1, v2, normals, areas,
                  n_points: int = None, density: float = None,
                  seed: int = 42):
    total_area = areas.sum()
    if density is not None:
        n_points = max(1, int(total_area * density))
        print(f"[Sample] density={density}/unit² → {n_points:,} points")
    else:
        print(f"[Sample] {n_points:,} points requested")

    rng     = np.random.default_rng(seed)
    probs   = areas / total_area
    tri_idx = rng.choice(len(v0), size=n_points, p=probs)

    # 均匀重心坐标采样（Osada 等人的方法）
    r1      = rng.random(n_points).astype(np.float32)
    r2      = rng.random(n_points).astype(np.float32)
    sqrt_r1 = np.sqrt(r1)
    u = 1.0 - sqrt_r1
    v = sqrt_r1 * (1.0 - r2)
    w = sqrt_r1 * r2

    pts = (u[:, None] * v0[tri_idx] +
           v[:, None] * v1[tri_idx] +
           w[:, None] * v2[tri_idx])
    nrm = normals[tri_idx]
    return pts.astype(np.float32), nrm.astype(np.float32)


# ---------------------------------------------------------------------------
# 文件写入函数
# ---------------------------------------------------------------------------

def write_ply(path, points, normals=None):
    has_n  = normals is not None
    props  = "property float x\nproperty float y\nproperty float z\n"
    if has_n:
        props += "property float nx\nproperty float ny\nproperty float nz\n"
    header = (f"ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(points)}\n{props}end_header\n")
    data   = (np.hstack([points, normals]) if has_n else points).astype(np.float32)
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(data.tobytes())
    print(f"[PLY]  {path}  ({len(points):,} pts)")


def write_pcd(path, points, normals=None):
    has_n = normals is not None
    n     = len(points)
    if has_n:
        fields = "x y z normal_x normal_y normal_z"
        meta   = "SIZE 4 4 4 4 4 4\nTYPE F F F F F F\nCOUNT 1 1 1 1 1 1"
        data   = np.hstack([points, normals]).astype(np.float32)
    else:
        fields = "x y z"
        meta   = "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1"
        data   = points.astype(np.float32)
    header = (f"# .PCD v0.7\nVERSION 0.7\nFIELDS {fields}\n{meta}\n"
              f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {n}\nDATA binary\n")
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(data.tobytes())
    print(f"[PCD]  {path}  ({n:,} pts)")


# ---------------------------------------------------------------------------
# 命令行界面
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Sample USD mesh faces → point cloud (.ply / .pcd)")
    p.add_argument("input")
    p.add_argument("-o", "--output", default=None,
                   help="Output path (.ply or .pcd). Default: <input>_sampled.ply")
    p.add_argument("-n", "--n-points", type=int, default=500_000,
                   help="Number of points to sample (default: 500000). "
                        "Ignored if --density is set.")
    p.add_argument("--density", type=float, default=None,
                   help="Points per unit² of surface area (overrides -n)")
    p.add_argument("--normals", action="store_true",
                   help="Include surface normals in output")
    p.add_argument("--seed", type=int, default=42,
                   help="Random seed (default: 42)")
    args = p.parse_args()

    out = args.output or str(
        Path(args.input).parent / (Path(args.input).stem + "_sampled.ply"))
    fmt = Path(out).suffix.lower()
    if fmt not in ('.ply', '.pcd'):
        print("Output must be .ply or .pcd"); sys.exit(1)

    t0 = time.time()
    v0, v1, v2, normals, areas = extract_triangles(args.input)
    pts, nrm = sample_points(v0, v1, v2, normals, areas,
                             n_points=args.n_points,
                             density=args.density,
                             seed=args.seed)
    nrm_out = nrm if args.normals else None
    (write_ply if fmt == '.ply' else write_pcd)(out, pts, nrm_out)
    print(f"[Done] {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()
