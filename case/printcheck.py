"""Print-orientation check (as exported by part = "tray" | "lid"): every face that looks down more steeply
than 45 degrees and is not on the bed, grouped by height. Small entries are hole roofs; large ones need a fix."""
import numpy as np, trimesh
for p in ("tray", "lid"):
    m = trimesh.load(f"GokuIMU_case_{p}.stl"); m.apply_translation([0, 0, -m.bounds[0][2]])
    n, c, a = m.face_normals, m.triangles_center, m.area_faces
    bad = (n[:, 2] < -0.72) & (c[:, 2] > 0.06)
    print(f"{p}: print size {np.round(m.extents, 1)} mm, bed contact {a[(n[:, 2] < -0.99) & (c[:, 2] < 0.06)].sum():.0f} mm2, overhang {a[bad].sum():.1f} mm2")
    zs = np.round(c[bad][:, 2], 1)
    for z in np.unique(zs):
        sel = bad.copy(); sel[bad] = zs == z; bb = m.triangles[sel].reshape(-1, 3)
        print(f"    z {z:5.1f}: {a[sel].sum():5.1f} mm2   x {bb[:, 0].min():.1f}..{bb[:, 0].max():.1f}  y {bb[:, 1].min():.1f}..{bb[:, 1].max():.1f}")
