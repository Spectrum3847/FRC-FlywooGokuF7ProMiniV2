// GokuIMU case v2: sealed tray + clear lid for the Flywoo Goku F7 Pro Mini V2.
// Built around goku_board.scad / goku_board.stl (board rebuilt in KiCad from the real parts).
// Same rules as the SpectrumShine-S case:
//   - flat floor = VHB face; no fasteners
//   - shiplap seam: the lid's outer wall lands on the tray's outer lip, so the closed seam has no gap;
//     the lip overlap is where the RTV goes
//   - the board is clamped flat between a perimeter ledge in the tray and four pins in the lid
//   - the only opening is the USB-C pocket; an inner collar on the lid hugs the receptacle shell
//   - every part prints without supports in the orientation part = "..." exports
// Render: openscad -D 'part="tray"' -o GokuIMU_case_tray.stl gokuimu_case.scad   (part = tray | lid | assembly | section)
//   tray: floor down.   lid: top face down (print the lid in clear PETG so the status LEDs show).

include <goku_board.scad>

part = "assembly";
$fn = 48;

// ---------------- case parameters
gap    = 0.4;     // board outline to wall (outline is measured to about +/-0.3)
gap_y  = 0.2;     // extra in Y: the photos give 32.3-32.8 mm overall against the 32.2 model
wall   = 1.8;     // split 0.8 (tray lip) / 0.1 gap / 0.9 (lid lip): each lip at least 2 lines of a 0.4 nozzle
flr    = 1.2;     // floor = VHB face
bclr   = 0.5;     // clearance under the bottom-side connectors
tclr   = 0.6;     // clearance over the top-side connectors
lidt   = 1.2;     // lid top
lap    = 1.2;     // shiplap height
ledge  = 0.8;     // width of the ledge under the board edge behind the USB pocket
relief = 0.3;     // the board rests on the four posts; everything else under it is relieved (bottom-edge pads)
pcb_t  = 1.6;     // real PCB thickness used for the stack-up (Flywoo's model says 1.5; FC boards are usually 1.6)
pin_d  = 5.0;     // lid pins over the mount-hole pads; they hold the board down and take the press-fit pegs
pin_clr = 0.05;   // pin bottom to board top
peg_d  = 3.0;     // pegs: through the board holes (open bore 3.3-3.7) and up into the lid pins
peg_in = 2.8;     // how far a peg reaches up into its lid pin
sock_d = 3.2;     // socket in each lid pin ...
rib_w  = 0.5;  rib_t = 0.075;   // ... with 4 crush ribs that grip the peg 0.075 a side: the press fit that holds the lid shut
ch     = 0.5;     // outside chamfers (top and bottom edges)
ch_seam = 0.3;    // reveal at the seam, also a groove to tool the RTV
// USB-C cable overmold pocket (USB-C spec max overmold 12.35 x 6.5)
OM_W = 12.8;  OM_H = 6.8;  OM_R = 2.4;
shell_clr = 0.15; // collar around the receptacle shell

zbb  = flr + GB_BOT_H + bclr;               // board bottom
zbt  = zbb + pcb_t;                         // board top = seam step
zs   = zbt + lap;                           // top of the tray lip = bottom of the lid's outer wall
zusb = zbb + pcb_t + (USB_ZC - GB_T);       // receptacle axis (the receptacle sits on the board top)
zin  = max(zbt + GB_TOP_H + tclr, zusb + OM_H / 2 + 0.3);   // lid ceiling
ztop = zin + lidt;
om_z0 = zusb - OM_H / 2;                    // full 6.8 mm pocket (USB-C spec overmold 6.5)
om_z1 = zusb + OM_H / 2;
xin  = GB_BX + gap;                         // inner face of the +X wall (USB side)
xp   = GB_BX + 0.15;                        // pocket back plane: inside the receptacle face (USB_FACE), just outside the board edge
pk0  = USB_Y - OM_W / 2;  pk1 = USB_Y + OM_W / 2;   // pocket y span
xout = xin + wall;

// ---------------- 2D shapes
module board2d()        { for (dy = [-gap_y, gap_y]) translate([0, dy]) goku_outline(); }   // outline grown by gap_y in Y
module cav2d(o = gap)   { offset(r = o) board2d(); }
module out2d(o = 0)     { offset(r = o) offset(r = -2) offset(r = gap + wall + 2) board2d(); }   // outside: concave corners filleted R2
module under2d() { cav2d(); }  // the board rests on the four posts; everything under it is open
module slab(z0, z1) { translate([0, 0, z0]) linear_extrude(z1 - z0) children(); }
// outer shell between z0 and z1 with optional chamfers (bottom cb, top ct)
module shell(z0, z1, cb = 0, ct = 0) {
    hull() {
        if (cb > 0) slab(z0, z0 + 0.01) out2d(-cb);
        slab(z0 + cb, z1 - ct) out2d();
        if (ct > 0) slab(z1 - 0.01, z1) out2d(-ct);
    }
}

// USB-C overmold pocket through the +X wall, from the outside to the receptacle face plane
module usb_pocket() {
    translate([xp, USB_Y, 0]) rotate([90, 0, 90]) linear_extrude(xout - xp + 1)
        translate([-OM_W / 2, om_z0]) offset(r = OM_R) offset(delta = -OM_R) square([OM_W, om_z1 - om_z0]);
}
// receptacle shell envelope (plus clearance), running inward from the face
module usb_shell(clr = shell_clr) {
    translate([USB_FACE - 8.2, USB_Y, zusb]) rotate([90, 0, 90]) linear_extrude(8.2 + wall + 1)
        offset(r = 1.2 + clr) square([USB_W - 2.4, USB_H - 2.4], center = true);
}

// ---------------- tray (prints floor down)
module tray() {
    difference() {
        union() {
            shell(0, zbt, ch, 0);
            difference() {                                      // outer shiplap lip, 0.8 thick
                slab(zbt - 0.01, zs) out2d();
                slab(zbt - 1, zs + 1) out2d(-0.8);
                // reveal chamfer on the outer top edge of the lip
                difference() { slab(zs - ch_seam, zs + 1) out2d(1); hull() { slab(zs - ch_seam, zs - ch_seam + 0.01) out2d(); slab(zs, zs + 0.01) out2d(-ch_seam); } }
            }
        }
        difference() {
            union() {
                slab(zbb - relief, zs + 1) cav2d();             // board pocket, relieved 0.3 under the board edge
                slab(flr, zbb + 0.01) under2d();                // room for the bottom-side parts
            }
            // under the pocket: a ledge the board edge sits on, and dams closing the edge slot either side
            translate([GB_BX - ledge, pk0 - 1, flr - 0.1]) cube([xin - GB_BX + ledge + 0.1, OM_W + 2, zbb - flr + 0.1]);
            for (y = [pk0 - 1, pk1]) translate([xp, y, flr - 0.1]) cube([xin - xp + 0.1, 1, zbt - flr + 0.1]);
        }
        usb_pocket();
    }
    for (x = [-1, 1], y = [-1, 1]) translate([x * GB_PITCH / 2, y * GB_PITCH / 2, flr - 0.01]) {
        cylinder(d = pin_d, h = zbb - flr + 0.01);              // post under each mount-hole pad
        hull() {                                                // peg: through the board and up into the lid pin, 0.4 chamfered tip
            cylinder(d = peg_d, h = zbt + peg_in - flr - 0.4);
            cylinder(d = peg_d - 0.8, h = zbt + peg_in - flr);
        }
    }
}

// ---------------- lid (prints top face down)
module lid() {
    difference() {
        union() {
            shell(zs, ztop, 0, ch);
            difference() {                                      // inner shiplap lip, 0.9 thick, 0.1 clear of the tray lip
                slab(zbt + 0.05, zs + 0.01) out2d(-0.8 - 0.1);
                slab(zbt - 1, zs + 1) cav2d();
            }
            // bottom reveal chamfer of the outer wall
        }
        difference() { slab(zs - 0.01, zs + ch_seam) difference() { out2d(1); out2d(-0.9 + 0.01); } hull() { slab(zs + ch_seam, zs + ch_seam + 0.01) out2d(); slab(zs - 0.01, zs) out2d(-ch_seam); } }
        slab(zbt - 1, zin) cav2d();                             // cavity
        usb_pocket();
        // thinner roof over the status LEDs so they show through the clear lid
        translate([11.8, -7.7, zin - 0.01]) cube([2.5, 4.3, 0.6]);
    }
    // collar: closes the pocket off from the inside. It sits 0.1 above the board top behind the pocket and fills
    // the board-edge slot either side of it; the receptacle shell passes through with shell_clr all round.
    difference() {
        intersection() {
            translate([xp - 0.8, pk0 - 1, zbt + 0.1]) cube([xin - xp + 0.8 + 0.01, OM_W + 2, zin - zbt]);   // 0.1 above the board: an RTV dab closes it
            slab(zbt, zin + 0.01) cav2d();
        }
        usb_shell();
        usb_pocket();
    }
    // pins over the four mount-hole pads: they stop pin_clr above the board, so the board can't lift, and each
    // takes a tray peg in a ribbed socket. Pushing the lid home presses the four pegs in: that is what holds it shut.
    for (x = [-1, 1], y = [-1, 1]) translate([x * GB_PITCH / 2, y * GB_PITCH / 2, 0]) {
        difference() {
            translate([0, 0, zbt + pin_clr]) cylinder(d = pin_d, h = zin - zbt - pin_clr + 0.01);
            difference() {                                      // ribbed socket with a 0.4 lead-in chamfer
                union() {
                    translate([0, 0, zbt - 0.1]) cylinder(d = sock_d, h = peg_in + 0.3);
                    translate([0, 0, zbt + pin_clr - 0.01]) cylinder(d1 = sock_d + 0.8, d2 = sock_d, h = 0.4);
                }
                for (a = [45, 135, 225, 315]) rotate(a) translate([peg_d / 2 - rib_t, -rib_w / 2, zbt + pin_clr + 0.4]) cube([1, rib_w, peg_in]);
            }
        }
    }
}

// ---------------- views
if (part == "tray") tray();
else if (part == "lid") translate([0, 0, ztop]) rotate([180, 0, 0]) lid();
else if (part == "lid_asm") lid();
else if (part == "section") difference() {
    union() { color("#555") tray(); color("#cfe8ff", 0.7) lid(); color("green") translate([0, 0, zbb]) goku_board(); }
    translate([-40, USB_Y, -1]) cube([80, 40, 40]);
}
else if (part == "closed") { color("#555") tray(); color("green") translate([0, 0, zbb]) goku_board(); color("#cfe8ff", 0.6) lid(); }
else { color("#555") tray(); color("green") translate([0, 0, zbb]) goku_board(); color("#cfe8ff", 0.45) translate([0, 0, 14]) lid(); }

echo(size = [2 * (xout), 2 * (16.39 + gap + gap_y + wall), ztop], zbb = zbb, zbt = zbt, zin = zin, usb_axis = zusb);
