# Rebuilds goku.kicad_pcb: Goku F7 Pro Mini V2 recreated from library footprints placed at
# positions measured from photos (board frame: origin = hole centre, +X = USB edge, +Y = "2540" tab).
# Run with KiCad's python:  "<KiCad>/bin/python.exe" build_board.py
import pcbnew, math
import os
L = os.environ.get("KICAD10_FOOTPRINT_DIR", os.path.expandvars(r"%LOCALAPPDATA%/Programs/KiCad/10.0/share/kicad/footprints")) + "/"
O = (100.0, 100.0)                       # board origin on the KiCad sheet
mm = lambda x, y: pcbnew.VECTOR2I(pcbnew.FromMM(O[0] + x), pcbnew.FromMM(O[1] + y))
b = pcbnew.BOARD()
b.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(1.6))

# ---- outline: body X +/-14.55, Y +/-13.0, R3.5 corners; pad tabs X +/-9.3 to Y +/-16.1
BX, BY, R, TX, TY = 14.55, 13.0, 3.5, 9.3, 16.1
pts = []
def arc(cx, cy, a0, a1, r=R, n=8):
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n); pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
pts += [(-TX, -TY), (TX, -TY), (TX, -BY)]
arc(BX - R, -BY + R, -90, 0)
pts += [(BX, BY - R)]
arc(BX - R, BY - R, 0, 90)
pts += [(TX, BY), (TX, TY), (-TX, TY), (-TX, BY)]
arc(-BX + R, BY - R, 90, 180)
arc(-BX + R, -BY + R, 180, 270)
pts += [(-TX, -BY)]
for i in range(len(pts)):
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
    s.SetStart(mm(*pts[i])); s.SetEnd(mm(*pts[(i + 1) % len(pts)])); s.SetWidth(pcbnew.FromMM(0.05)); b.Add(s)
for x in (-10, 10):
    for y in (-10, 10):
        c = pcbnew.PCB_SHAPE(b); c.SetShape(pcbnew.SHAPE_T_CIRCLE); c.SetLayer(pcbnew.Edge_Cuts)
        c.SetCenter(mm(x, y)); c.SetEnd(mm(x + 1.75, y)); c.SetWidth(pcbnew.FromMM(0.05)); b.Add(c)

def place(ref, lib, name, x, y, rot, bottom=False):
    f = pcbnew.FootprintLoad(L + lib + ".pretty", name); f.SetReference(ref)
    f.SetPosition(mm(x, y)); f.SetOrientationDegrees(rot); b.Add(f)
    if bottom: f.Flip(f.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    return f

GH = "JST_GH_SM0%dB-GHS-TB_1x0%d-1MP_P1.25mm_Horizontal"
# top side. GH housings: footprint opening faces +y; housing centre is 0.425 mm from the origin
place("J1", "Connector_JST", GH % (4, 4), -3.35, -12.75, 180)
place("J2", "Connector_JST", GH % (4, 4),  4.00, -12.75, 180)
place("J3", "Connector_JST", GH % (4, 4), -3.60,  12.20, 0)
place("J4", "Connector_JST", GH % (4, 4),  3.40,  12.20, 0)
place("J5", "Connector_USB", "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal", 11.15, -1.50, 90)
place("U1", "Package_QFP", "LQFP-64_10x10mm_P0.5mm", -1.70, -2.00, 0)
for i, y in enumerate((4.1, 5.1, 6.0, 7.0)):
    place("D%d" % (i + 1), "LED_SMD", "LED_0402_1005Metric", 13.2, y, 0)
# bottom side
place("J6", "Connector_JST", GH % (6, 6),  0.25, -12.80, 180, bottom=True)
place("J7", "Connector_JST", GH % (8, 8), -0.30,  12.20, 0, bottom=True)
place("U2", "Package_SO", "TSSOP-28_4.4x9.7mm_P0.65mm", 3.60, 3.15, 90, bottom=True)
place("U3", "Package_SON", "WSON-8-1EP_6x5mm_P1.27mm_EP3.4x4mm", 4.75, -4.55, 90, bottom=True)
place("L1", "Inductor_SMD", "L_Sunlord_SWPA4030S", -4.60, -7.00, 0, bottom=True)
place("L2", "Inductor_SMD", "L_Sunlord_SWPA4030S", -10.90, 0.65, 0, bottom=True)
b.Save("goku.kicad_pcb")
print("saved")
