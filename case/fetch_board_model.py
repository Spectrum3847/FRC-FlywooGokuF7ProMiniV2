"""Download Flywoo's 3D model of the Goku F722 Pro Mini V2 and convert it to goku_board.stl in the case frame.

The model is Flywoo's (embedded on https://flywoo.net/products/goku-f7-mini-v2-20x20-flight-controller), so it
isn't committed here. Run this once before fitcheck.py / cadcheck:

    pip install trimesh DracoPy shapely
    python fetch_board_model.py

Frame (see goku_board.scad): origin = centre of the 4 mount holes, z = 0 on the PCB bottom face,
+X toward the USB-C edge, +Y toward the RX1 tab, +Z the top side. Scale from the 20 mm hole pitch.
"""
import urllib.request
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
URL = "https://files.myshopline.com/file/dimensional/999999999/7ce362e985874f23b734af1d3986f049.glb"
GLB = HERE / "flywoo_model" / "goku_f722_pro_mini_v2_flywoo.glb"

if not GLB.exists():
    GLB.parent.mkdir(exist_ok=True)
    urllib.request.urlretrieve(URL, GLB)

s = trimesh.load(GLB)
meshes = []
for n in s.graph.nodes_geometry:
    T, g = s.graph[n]
    m = s.geometry[g].copy()
    m.apply_transform(T)
    meshes.append(m)
# the PCB is the mesh with this footprint (model units); its four hole circles give centre and scale
pcb = next(m for m in meshes if abs(m.extents[0] - 0.5783) < 0.002 and abs(m.extents[1] - 0.6506) < 0.002)
v, c = pcb.vertices[:, :2], (pcb.bounds[0][:2] + pcb.bounds[1][:2]) / 2
centres = []
for sx in (-1, 1):
    for sy in (-1, 1):
        near = v[np.linalg.norm(v - (c + np.array([sx, sy]) * 0.2), axis=1) < 0.05]
        A = np.c_[2 * near, np.ones(len(near))]
        centres.append(np.linalg.lstsq(A, (near ** 2).sum(1), rcond=None)[0][:2])
centres = np.array(centres)
pitch = (np.ptp(centres[:, 0]) + np.ptp(centres[:, 1])) / 2
S, C = 20.0 / pitch, np.r_[centres.mean(0), 0.0]
R = np.diag([1, -1, -1])          # 180 deg about X: the model's USB-C side (-Z) becomes the top (+Z)
for m in meshes:
    m.vertices = ((m.vertices - C) * S) @ R.T
zb = pcb.vertices[:, 2].min()     # pcb is one of meshes, already transformed
for m in meshes:
    m.vertices[:, 2] -= zb        # z = 0 on the PCB bottom face
trimesh.util.concatenate(meshes).export(HERE / "goku_board.stl")
print(f"scale {S:.2f} mm/unit, wrote goku_board.stl")
