"""GokuIMU case v3 (build123d): sealed tray + clear lid for the Flywoo Goku F7 Pro Mini V2.

Port of gokuimu_case.scad (v2, 0.4 mm nozzle) to build123d, re-proportioned for the team's
0.6 mm nozzles / 0.3 mm layers. Same design rules as v2 (see README.md):
  - flat floor = VHB face, no fasteners, every part prints without supports
  - shiplap seam: lid outer wall lands on the tray's outer lip; RTV goes in the lip overlap
  - lid held shut by a press fit: 4 tray pegs go up through the board holes into sockets in 4 lid pins;
    the pins stop just above the board, so it can't lift
  - the only opening is the USB-C overmold pocket; a lid collar closes it off from the inside
What changed for 0.6 mm: every lip, dam, ledge, collar and nub is >= 1.24 mm (2 lines of 0.62), so the wall
is 2.68 (1.24 lip / 0.2 gap / 1.24 lip) instead of 1.8; nubs are 1.24 dia instead of 0.7; the
0.3 mm seam reveal chamfers are gone (smaller than one line).

Check / render / export:
    python <Spectrum-AI-Tools>/plugins/spectrum-cad/skills/build123d-cad/scripts/cadcheck.py gokuimu_case.py
"""
import re
from pathlib import Path

from build123d import *

HERE = Path(__file__).parent

# ---------------- board (frame of goku_board.scad: origin = mount-hole centre, z=0 PCB bottom,
# +X = USB-C edge, +Y = RX1 tab). Constants from Flywoo's model; see board_model.md.
GB_T, GB_PITCH = 1.5, 20.0
GB_BX = 14.52                     # +/-X board edge
GB_TOP_H, GB_BOT_H = 3.51, 3.18   # tallest part above / below the PCB
USB_FACE, USB_Y = 14.47, 2.01
USB_ZC = GB_T + 1.75
USB_W, USB_H = 8.96, 3.53
OUTLINE = [tuple(map(float, p)) for p in re.findall(
    r"\[(-?[\d.]+), (-?[\d.]+)\]", (HERE / "goku_outline_flywoo.scad").read_text())]

# ---------------- printer
NOZZLE, LAYER = 0.6, 0.3
LINE = 0.62                       # slicer line width for the 0.6 nozzle
LINE2 = 2 * LINE                  # 1.24: thinnest wall/lip/rib that prints as 2 solid lines

# ---------------- case parameters (mm)
gap, gap_y = 0.4, 0.2             # board outline to wall; extra in Y (photo spread)
lip_t = LINE2                     # tray outer lip and lid inner lip, each 2 lines
lip_gap = 0.2                     # between the two lips (RTV goes here)
wall = 2 * lip_t + lip_gap        # 2.68
flr = 4 * LAYER                   # 1.2 floor = VHB face
bclr, tclr = 0.5, 0.6             # under bottom parts / over top parts
lidt = 4 * LAYER                  # 1.2 lid top
lap = 4 * LAYER                   # 1.2 shiplap height
ledge = LINE2 + 0.15              # board-edge ledge behind the USB pocket: 2 lines wide in front of the pocket
pcb_t = 1.6                       # real PCB thickness for the stack-up (Flywoo's model says 1.5; FC boards are usually 1.6)
peg_d = 3.0                       # tray pegs: through the board holes (open bore 3.3-3.7) and up into the lid pins
peg_in = 2.8                      # how far each peg reaches into its lid pin
sock_d = 3.1                      # socket in each lid pin, as drawn. Vertical holes print 0.2-0.3 undersize at 0.6 mm,
                                  # so it prints ~2.85: 0.075 a side of interference on the peg = the press fit.
                                  # Too tight: raise toward 3.2. Too loose: lower toward 3.0.
pin_d = sock_d + 2 * (LINE2 + 0.05)   # 5.68: socket wall 2 lines + a little
pin_clr = 0.05                    # pin bottom to board top
lead = 0.4                        # lead-in chamfer on the peg tip and the socket mouth
ch = 0.6                          # outside chamfers (one line)
OM_W, OM_H, OM_R = 12.8, 6.8, 2.4 # USB-C overmold pocket (spec max 12.35 x 6.5)
shell_clr = 0.15                  # collar around the receptacle shell
collar_t = 1.4                    # collar thickness in X (>= 2 lines)
led_roof = 2 * LAYER              # 0.6 roof over the status LEDs so they glow through clear PETG

# ---------------- derived heights
zbb = flr + GB_BOT_H + bclr       # board bottom
zbt = zbb + pcb_t                 # board top = seam step
zs = zbt + lap                    # tray lip top
zusb = zbb + pcb_t + (USB_ZC - GB_T)   # the receptacle sits on the board top
zin = max(zbt + GB_TOP_H + tclr, zusb + OM_H / 2 + 0.3)   # lid ceiling
ztop = zin + lidt
xin = GB_BX + gap                 # inner face of the +X wall
xp = GB_BX + 0.15                 # pocket back plane
xout = xin + wall
pk0, pk1 = USB_Y - OM_W / 2, USB_Y + OM_W / 2

# ---------------- 2D
outline = Polygon(*OUTLINE, align=None)
board2d = (Pos(0, -gap_y) * outline) + (Pos(0, gap_y) * outline)
cav2d = offset(board2d, gap, kind=Kind.ARC)
OUT = offset(offset(board2d, gap + wall + 2, kind=Kind.ARC), -2, kind=Kind.ARC)  # concave corners R2


def slab(face, z0, z1):
    return extrude(Pos(0, 0, z0) * face, z1 - z0)


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=Align.MIN)


def yz_prism(sketch, x0, length):
    """Sketch drawn in the Y-Z plane centred at (USB_Y, zusb), extruded +X from x0."""
    pl = Plane(origin=(x0, USB_Y, zusb), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return extrude(pl * sketch, length)


usb_pocket = yz_prism(RectangleRounded(OM_W, OM_H, OM_R), xp, xout - xp + 1)
usb_shell = yz_prism(RectangleRounded(USB_W + 2 * shell_clr, USB_H + 2 * shell_clr, 1.2 + shell_clr),
                     USB_FACE - 8.2, 8.2 + wall + 1)
holes = [(x * GB_PITCH / 2, y * GB_PITCH / 2) for x in (-1, 1) for y in (-1, 1)]

# ---------------- tray (prints floor down)
tray = slab(OUT, 0, zbt)
tray = chamfer(tray.edges().group_by(Axis.Z)[0], ch)
tray += slab(OUT, zbt, zs) - slab(offset(OUT, -lip_t, kind=Kind.ARC), zbt - 1, zs + 1)
tray -= slab(cav2d, flr, zs + 1)
tray += box(GB_BX - ledge, xin + 0.1, pk0 - 1, pk1 + 1, flr - 0.1, zbb)        # ledge under the board edge
for y in (pk0 - LINE2, pk1):                                                     # dams closing the edge slot
    tray += box(xp, xin + 0.1, y, y + LINE2, flr - 0.1, zbt)
tray -= usb_pocket
for x, y in holes:
    tray += Pos(x, y, flr - 0.01) * Cylinder(pin_d / 2, zbb - flr + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    h_peg = zbt + peg_in - flr + 0.01
    peg = Pos(x, y, flr - 0.01) * Cylinder(peg_d / 2, h_peg, align=(Align.CENTER, Align.CENTER, Align.MIN))
    tray += chamfer(peg.edges().group_by(Axis.Z)[-1], lead)

# ---------------- lid (assembly position; prints top face down)
lid = slab(OUT, zs, ztop)
lid = chamfer(lid.edges().group_by(Axis.Z)[-1], ch)
lid += slab(offset(OUT, -(lip_t + lip_gap), kind=Kind.ARC), zbt + 0.05, zs + 0.01) - slab(cav2d, zbt - 1, zs + 1)
lid -= slab(cav2d, zbt - 1, zin)
lid -= usb_pocket
lid -= box(11.8, 14.3, -7.7, -3.4, zin - 0.01, zin + lidt - led_roof)           # thin roof over LEDs
collar = box(xp - collar_t, xin + 0.01, pk0 - 1, pk1 + 1, zbt + 0.1, zin + 0.01) & slab(cav2d, zbt, zin + 0.01)   # 0.1 above the board; RTV closes it
shell_y0 = USB_Y - USB_W / 2 - shell_clr
collar -= box(xp - collar_t - 1, xin + 1, pk0 - 2, shell_y0, zbt - 1, zbt + 0.9)  # notch over the LEDs, out to the shell
collar -= usb_shell
collar -= box(xp - collar_t - 1, xin + 1, shell_y0, USB_Y + USB_W / 2 + shell_clr, zbt - 1, zusb)   # square below the axis: the receptacle sits on the board
collar -= usb_pocket
lid += collar
for x, y in holes:
    # lid pin over each mount-hole pad: stops pin_clr above the board and takes a tray peg in its socket
    pin = Pos(x, y, zbt + pin_clr) * Cylinder(pin_d / 2, zin - zbt - pin_clr + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pin -= Pos(x, y, zbt - 0.1) * Cylinder(sock_d / 2, peg_in + 0.3, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pin -= Pos(x, y, zbt + pin_clr - 0.01) * Cone(sock_d / 2 + lead, sock_d / 2, lead + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    pin -= box(x - 4, x + 4, -7.45, 7.45, zbt - 1, zbt + 1.0)   # relief over small top-side parts inboard of the pads (|y| < 7.45)
    if x > 0:   # +X pins: trim where parts come within 7.2 of the hole centre
        pin &= box(7.2, 40, -40, 40, 0, 50)
    lid += pin

# ---------------- reference: Flywoo's own 3D model of the board (goku_board.stl, the real one; the
# kicad/ rebuild is approximate). Its copper rings round the mount holes have a cosmetic 2.87 mm
# hole (real open bore 3.3-3.7), so ref points within r 1.65 of each hole centre are ignored.

# ---------------- cadcheck contract
parts = {"tray": tray, "lid": lid}
refs = {"board": {"mesh": "goku_board.stl", "move": (0, 0, zbb),
                  "ignore_xy_circles": [(x, y, 1.65) for x, y in holes]}}
printer = {"nozzle": NOZZLE, "line": LINE, "layer": LAYER}
print_orientation = {"lid": (180, 0, 0)}
checks = {
    # intended contact: board bottom on the posts/ledge. The lid stays clear of the board (pins pin_clr above a
    # 1.6 board, 0.15 above the 1.5 mm model).
    "no_overlap": [("tray", "lid")],   # drawn 0.05 a side clear; the press fit comes from the socket printing undersize
    "contact": [("tray", "board")],
    "clearance": [("lid", "board", 0.05)],
    "max_size": (250, 250, 250),
    "max_overhang_area": 5.0,
}
sections = [("XZ", USB_Y), ("XZ", 10.0), ("YZ", 13.0), ("XY", zusb)]

if __name__ == "__main__":
    print(f"outside {2 * xout:.1f} x {OUT.bounding_box().size.Y:.1f} x {ztop:.2f}  zbb={zbb:.2f} zin={zin:.2f}")
