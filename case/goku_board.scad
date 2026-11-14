// Flywoo Goku F7 Pro Mini V2: board model for case design.
//
// goku_board.stl is Flywoo's own 3D model of the board (the one embedded on the product page,
// flywoo_model/goku_f722_pro_mini_v2_flywoo.glb), scaled and placed in the frame below. The scale comes
// from its four mount holes (20 mm pitch); drawn over the rectified photos it matches every pad, part and
// edge (board_overlay_top.png / board_overlay_bottom.png). kicad/ holds an independent rebuild from KiCad
// library footprints, kept as a cross-check.
//
// Frame (right-handed, looking down on the component side):
//   origin = centre of the 4 mount holes, z = 0 on the PCB bottom face
//   +X = toward the USB-C edge, +Y = toward the "RX1" tab, +Z = top (MCU / USB-C) side.
// Use: include <goku_board.scad>   then goku_board();  or goku_outline(); (2D)

include <goku_outline_flywoo.scad>   // GOKU_OUTLINE: PCB outline from Flywoo's model

GB_T      = 1.5;                 // PCB thickness (Flywoo model 1.496)
GB_PITCH  = 20.0;                // mount hole pitch
GB_HOLE_D = 3.5;                 // PCB hole 3.55 in Flywoo's model; open bore 3.3-3.7 in the photo with the grommets removed
                                 // (the model's copper ring meshes have a 2.87 inner hole; that is cosmetic and ignored)
GB_BX = 14.52;                   // +X / -X board edge (USB side), from the outline
GB_TOP_H = 3.51;                 // tallest above the top face: USB-C shell (other parts <= 3.21, JST SH 1.0 connectors)
GB_BOT_H = 3.18;                 // tallest below the bottom face: JST SH 6P / 8P

// USB-C receptacle: opening on the +X face
USB_FACE = 14.47;                // x of the receptacle face (flush with the board edge at 14.52)
USB_Y    = 2.01;                 // y of the receptacle centre
USB_ZC   = GB_T + 1.75;          // z of the receptacle centre (shell from the board top to 5.01 above the PCB bottom)
USB_W    = 8.96;  USB_H = 3.53;  // shell envelope in Flywoo's model (y -2.46 .. 6.47)

// Status LEDs (top side, beside the USB-C): STATE, 10V, 5V, 3V3
GB_LEDS  = [[13.7, -6.4], [13.7, -5.5], [13.7, -4.6], [13.7, -3.7]];
// BOOT switch: side-actuated, top side, overhangs the -X edge by 0.25
GB_BOOT  = [[-14.77, -2.99], [-13.97, -1.30], 2.95];

module goku_outline() { polygon(GOKU_OUTLINE); }

module goku_board() { import("goku_board.stl"); }

if ($preview && is_undef(part)) goku_board();
