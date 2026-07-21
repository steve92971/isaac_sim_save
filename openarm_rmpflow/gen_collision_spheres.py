#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
從 URDF 的 collision STL 產生 Lula collision_spheres（繞開坑⑧的 GUI 貼球死路）。

做法：對每根 link 的 collision mesh
  1. trimesh 載入 → 套 scale（含負值翻轉）→ 套 collision origin（本機全 rpy=0，只平移）
     → 頂點落在「該 link 的 frame」，正是 Lula collision_spheres.center 要的 frame。
  2. PCA 找主軸 → 沿軸按固定間距切段 → 每段：球心=段內頂點質心，半徑=段內頂點到球心最大距離。
     → 保證球完整包住該段 mesh；細長 link 自然攤成一串小球，不會被一顆巨球包死。
輸出：把原 lula description 的 collision_spheres 以前的段原封保留，接上新產生的 collision_spheres。
"""
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import sys

PKG = "openarm_description"
PKG_ROOT = "/home/Intern_1/openarm_ws/src/openarm_description"
URDF = f"{PKG_ROOT}/urdf/robot/openarm_bimanual.urdf"

SIDE = sys.argv[1] if len(sys.argv) > 1 else "left"
SPACING = 0.05     # 沿主軸每 ~5cm 一顆球
MAX_SEG = 8        # 單根 link 球數上限
RADIUS_MARGIN = 1.0  # 半徑膨脹係數（1.0=剛好包住；要更安全可調 1.05）


def resolve(fn):
    return fn.replace(f"package://{PKG}", PKG_ROOT)


def wanted(name):
    return name == "openarm_body_link0" or name.startswith(f"openarm_{SIDE}_")


def link_spheres(verts):
    """verts: (N,3) 已在 link frame。回傳 [(center(3,), radius), ...]"""
    c = verts.mean(axis=0)
    centered = verts - c
    # PCA 主軸
    cov = np.cov(centered.T)
    w, V = np.linalg.eigh(cov)
    axis = V[:, np.argmax(w)]
    t = centered @ axis
    span = t.max() - t.min()
    k = int(np.clip(round(span / SPACING), 1, MAX_SEG))
    edges = np.linspace(t.min(), t.max(), k + 1)
    spheres = []
    for i in range(k):
        lo, hi = edges[i], edges[i + 1]
        m = (t >= lo) & (t <= hi) if i == k - 1 else (t >= lo) & (t < hi)
        seg = verts[m]
        if len(seg) == 0:
            continue
        sc = seg.mean(axis=0)
        r = np.linalg.norm(seg - sc, axis=1).max() * RADIUS_MARGIN
        spheres.append((sc, r))
    return spheres


def main():
    tree = ET.parse(URDF)
    out = {}   # link_name -> spheres
    total = 0
    for link in tree.getroot().findall("link"):
        name = link.get("name")
        if not wanted(name):
            continue
        col = link.find("collision")
        if col is None:
            continue
        mesh_el = col.find("geometry/mesh")
        if mesh_el is None:
            continue
        path = resolve(mesh_el.get("filename"))
        scale = np.array([float(v) for v in mesh_el.get("scale", "1 1 1").split()])
        o = col.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])])
        rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])])

        mesh = trimesh.load(path, force="mesh")
        verts = np.asarray(mesh.vertices) * scale          # 套 scale（負值翻轉座標，包圍球不受影響）
        # rpy 旋轉（本機全 0，仍一般化處理）
        cr, cp, cy = np.cos(rpy); sr, sp, sy = np.sin(rpy)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        verts = verts @ R.T + xyz                          # → link frame
        sph = link_spheres(verts)
        out[name] = sph
        total += len(sph)
        rs = [r for _, r in sph]
        print(f"  {name:26s} 球數={len(sph):2d}  半徑 {min(rs):.3f}~{max(rs):.3f} m", file=sys.stderr)

    print(f"\n總球數 = {total}\n", file=sys.stderr)

    # 產出 YAML 片段
    lines = ["collision_spheres:"]
    for name, sph in out.items():
        lines.append(f"  - {name}:")
        for c, r in sph:
            lines.append(f"    - center: [{c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f}]")
            lines.append(f"      radius: {r:.5f}")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
