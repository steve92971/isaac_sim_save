#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""驗證產出的 collision_spheres 是否真的包住各 link 的 collision mesh（守門 frame 正確性）。"""
import xml.etree.ElementTree as ET
import numpy as np, trimesh, yaml, sys

PKG_ROOT = "/home/Intern_1/openarm_ws/src/openarm_description"
URDF = f"{PKG_ROOT}/urdf/robot/openarm_bimanual.urdf"
YAML = sys.argv[1]

spheres = {}
doc = yaml.safe_load(open(YAML))
for entry in doc["collision_spheres"]:
    for link, lst in entry.items():
        spheres[link] = [(np.array(s["center"]), float(s["radius"])) for s in lst]

tree = ET.parse(URDF)
worst = 0.0
for link in tree.getroot().findall("link"):
    name = link.get("name")
    if name not in spheres:
        continue
    col = link.find("collision"); mesh_el = col.find("geometry/mesh")
    path = mesh_el.get("filename").replace("package://openarm_description", PKG_ROOT)
    scale = np.array([float(v) for v in mesh_el.get("scale", "1 1 1").split()])
    o = col.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz").split() if o is not None else [0, 0, 0])])
    verts = np.asarray(trimesh.load(path, force="mesh").vertices) * scale + xyz  # rpy=0
    # 每頂點對所有球取「距離-半徑」最小值；<=0 表示被包住
    C = np.array([c for c, _ in spheres[name]]); R = np.array([r for _, r in spheres[name]])
    d = np.linalg.norm(verts[:, None, :] - C[None, :, :], axis=2) - R[None, :]
    out = d.min(axis=1)                      # 每頂點外露距離（<=0 覆蓋）
    covered = (out <= 1e-6).mean() * 100
    maxout = max(0.0, out.max())
    worst = max(worst, maxout)
    flag = "OK" if maxout < 1e-3 else "⚠"
    print(f"  {name:26s} 覆蓋 {covered:6.2f}%  最大外露 {maxout*1000:6.2f} mm  {flag}")
print(f"\n全機最大外露 = {worst*1000:.2f} mm  → {'✅ 球完整包住 mesh，frame 正確' if worst < 1e-3 else '⚠ 有外露，需檢查'}")
