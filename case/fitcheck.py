"""Fit check: case parts vs the KiCad board model, and tray vs lid. Needs trimesh + rtree + manifold3d.
Run in case/:  python fitcheck.py   (exports the assembled parts with OpenSCAD first)."""
import subprocess, re, numpy as np, trimesh
O = "C:/github/SpectrumShine/tools/openscad/openscad-2021.01/openscad.com"
out = {}
for p in ("tray", "lid_asm"):
    r = subprocess.run([O, "-D", f'part="{p}"', "-o", f"_{p}.stl", "gokuimu_case.scad"], capture_output=True, text=True)
    out[p] = r.stderr + r.stdout
zbb = float(re.search(r"zbb = ([\d.]+)", out["tray"]).group(1))
t, l = trimesh.load("_tray.stl"), trimesh.load("_lid_asm.stl")
b = trimesh.load("goku_board.stl"); b.apply_translation([0, 0, zbb])
for n, m in (("tray", t), ("lid", l)): print(f"{n}: watertight {m.is_watertight}, volume {m.volume:.0f} mm3")
ov = t.intersection(l); print(f"tray/lid overlap volume: {0 if ov.is_empty else ov.volume:.3f} mm3")
s, _ = trimesh.sample.sample_surface(b, 60000, seed=1); pts = np.vstack([s, b.vertices])
# the model's copper rings round the mount holes have a cosmetic 2.87 mm inner hole (the real open bore is 3.3-3.7);
# ignore board points inside r 1.65 of a hole centre so the pegs are judged against the real bore
hc = np.array([[x, y] for x in (-10, 10) for y in (-10, 10)])
keep = np.min(np.linalg.norm(pts[:, None, :2] - hc[None], axis=2), axis=1) > 1.65
pts = pts[keep]
for n, m in (("tray", t), ("lid", l)):
    ins = np.concatenate([m.contains(pts[i:i + 20000]) for i in range(0, len(pts), 20000)])
    print(f"{n}: board points inside solid {int(ins.sum())}"
          + (f"  z {pts[ins][:, 2].min():.2f}..{pts[ins][:, 2].max():.2f}" if ins.sum() else ""))
    # clearance from each board point to the part, grouped into 3 mm cells; contact (< 0.06) is where the board is clamped
    q = pts[::3]; _, d, _ = trimesh.proximity.closest_point(m, q)
    cell = np.c_[np.round(q[:, :2] / 3), np.round(q[:, 2])]
    u, inv = np.unique(cell, axis=0, return_inverse=True); inv = inv.ravel()
    rows = sorted((d[inv == i].min(), *q[inv == i][np.argmin(d[inv == i])].round(1)) for i in range(len(u)))
    print(f"  {n}: closest approaches (mm, at x y z):")
    for r in rows[:12]: print("    %.2f  at %6.1f %6.1f %5.1f" % r)
