"""V4 R5-AL (user decisions 2026-10-27): aluminium instead of steel, no lathe parts.

Rule (memory material-rule-aluminium): aluminium by default; steel only where a number needs it; stainless only where a
per-part check shows it is better; lathe parts become 3D prints or bought parts. A home stick welder cannot weld
aluminium, so every R4-DIY weld becomes a bolted joint (M3 8.8 + rivnut / tapped 6061). Z-up, nose +Y, mm (AGENTS.md).
CONCEPT CAD for fit, mass and balance. r3/, r4_steel/ and r4_diy/ are not touched.

Changes from build_r4_diy.py (D1):
  * frame 12.7 x 12.7 x 2.0 aluminium square tube, 6063-T6 minimum (6061-T6 better; T5 fails), bolted, same layout;
    tabs 2.0 aluminium plate (hinge/motor/strap/wall tabs) bolted to the tubes
  * weapon module back to R3's bolted 6061 parts: uprights 6 mm, top/bottom braces 3 mm, M06 motor mount 3 mm.
    Each upright is bolted to the frame through one 12.7 aluminium tube rung (upright -> rail) and one 2 mm aluminium
    angle foot (upright -> bulkhead cross); the M06 mount sits on two 2 mm aluminium angles
  * rotor without lathe work: the 6061 hub is replaced by a 3/4" black steel pipe (OD 26.7, wall 2.3, ID ~22.1,
    measure it) 40 mm long, the two SS400 discs WELDED to its ends (steel kept: weapon), 608s inside the pipe with
    retaining compound; the belt runs on the pipe (root D26.7 vs R3 groove root D27) between two printed PETG flanges
  * printed PETG instead of turned 6061: shaft spacers (snug nut, low preload), weapon motor pulley, drive wheel hubs
    (same O-ring tyres, web 3 mm)
  * open issue fixed: a mid-wall tube stub + tab on each side halves the PC side-wall span (158 -> ~79 mm)
  * steel kept with a reason: discs (weapon), M8 12.9 dead shaft, 1.6 wedgelets (thin hard edge), hinge pins
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r5_al/build_r5_al.py
Writes robot_v4/print_revision/r5_al/output/.
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PR = HERE.parent                                        # robot_v4/print_revision
sys.path.insert(0, str(PR))

import cadquery as cq
import numpy as np
from cad_backend import Body, part, group, block, prism, find_body
from revision import build, wedgelet_plate
from legacy_geometry import d_bore
from checks import mass_properties

OUT = HERE / 'output'
OUT.mkdir(exist_ok=True)
config = json.loads((PR / 'r3' / 'inputs_R3.json').read_text())
FRONT_Y = config['design']['front_axle_y_mm']           # 18: the only driven axle in 2WD

# ---------------------------------------------------------------- parameters (mm, g/mm^3)
TUBE, WALL = 12.7, 2.0      # 1/2" x 2.0 aluminium square tube, 6063-T6 or 6061-T6 (6063-T5 fails the 5.2 kN check: calc/r5_al_calc.py)
TAB = 2.0                   # aluminium plate 2 mm (5052/6061), hand-cut tabs, bolted
WEDGE_T = 1.6               # SS400 wedgelets stay steel (thin hard edge)
BRACKET_T = 1.5             # aluminium skirt brackets (unchanged from W1)
SKIN = 2.0                  # PC skin thickness
FLOOR_Z0 = 4.0              # floor z 4..6, same underside as R3
RAIL_Z0 = FLOOR_Z0 + SKIN   # tubes stand on the floor: z 6..18
RAIL_Z1 = RAIL_Z0 + TUBE    # 18.7: 0.8 mm under the drive motors (z 19.5) -> 0.8 mm EPDM
LID_Z0 = 61.0
WALL_X = 83.0
REAR_Y = -110.0
CROSS_Y = (50.0, 62.0)      # bulkhead cross: front tyre reaches y 46.8 at z <= 18 (r 32 about y 18, z 32)
RAIL_X = (46.0, 46.0 + TUBE)  # |x|: inner face 46 as W1; outer 58.7 (wheel hub from 59.5)
BEAM_Y = (98.0 - TUBE, 98.0)  # hinge beam: front face 98 as W1 (pin y 102)
MOTOR_CROSS_Y = (10.0, 22.0)
HINGE_TAB_X = ((37.0, 37.0 + TAB), (62.0, 62.0 + TAB), (85.0, 85.0 + TAB))
HINGE_BEAM_X = (33.5, 91.0)   # longer than R4-DIY (37..88) so every tab has a bolting leg on the beam (91.0: clear of the skirt)
UPR_IN, UPR_OUT = 27.0, 33.0  # R3 6061 6 mm uprights, unchanged
PIN_Y, PIN_Z = 102.0, 14.5                                # wedgelet hinge axis (R3)
WHEEL_R, WHEEL_X = 32.0, (59.5, 75.5)
MOTOR_HOLES_YZ = [(-8.5, 0.0), (8.5, 0.0)]                # JB361 face: 2 x M3 at +-8.5 about the shaft
GEARBOX_SCREWS_YZ = [(0.0, -8.5), (0.0, 8.5)]
BATTERY = (57.0, 31.0, 24.0)                              # 3S 650 mAh envelope, ASSUMED
RHO = {'al_tube': 0.00270, 'al_tab': 0.00270, 'al_bar': 0.00270, 'steel_pipe': 0.00785, 'steel_wedgelet': 0.00785, 'steel_bolt': 0.00785,
       'PC': 0.00120, 'UHMW': 0.00094, 'EPDM': 0.00130, 'al5052_bracket': 0.00268, 'NBR': 0.00130,
       'petg_print': 0.00127, 'PETG_80': 0.00127 * 0.8}   # structural prints (spacers, pulley, hubs) 100 % infill
EXPLICIT_G = {'battery_650': 58.0, 'nylon_standoff': 0.6}
ORING_ID, ORING_CS = 51.6, 5.7
HUB = {'groove_root_d': 52.8, 'land_d': 57.0, 'rim_id': 49.0, 'web_t': 3.0, 'boss_d': 12.0, 'groove_w': 6.6,
       'land_w': (1.0, 0.8, 1.0), 'holes': (5, 4.5, 15.5)}
ORING_CS_FITTED = ORING_CS * (1 - 0.5 * (HUB['groove_root_d'] - ORING_ID) / ORING_ID)
RIVNUTS = 24                # M3 steel rivnut 0.9 g + screw 0.5 g (floor + rear wall into the tubes)
SWITCH_MOUNT_G = 3.0
LID_ANGLES = 4              # bought aluminium angles: lid to walls (NoCage)
ROTOR_WELD_G = 3.0          # pipe-to-disc fillets, ASSUMED
JOINT_G = {'tube_tube': 5.3, 'tab_tube': 2.8}  # 20x20x2 Al cleat 2.5 g + 2 x (rivnut 0.9 + M3 0.5); tab: 2 x (rivnut + M3)
THK = {}                    # part -> stock thickness, for the cut list

# ---------------------------------------------------------------- R3 in memory, minus what DIY replaces
root, _ = build(config)
REMOVE = ('Tub_TPU', 'Lid_TPU_2mm', 'JGA25_370_400rpm_Rear_', 'Wheel_', 'Battery_Tray_PETG', 'Electronics_Platform_PETG',
          'Electronics_Standoff_PETG', 'Wedgelet_', 'Weapon_Mount_Foot_PETG', 'Disconnect_Collar_PETG',
          '3S_850mAh_LiPo_ENVELOPE', 'Nose_Skid', 'Hall_Post_PETG', 'Hinge_Pin_D3', 'Eclip_',
          'Beater_Hub_6061_D30', 'Motor_Pulley_D30_1to1', 'Spacer_', 'Weapon_Motor_Mount_3mm')
r3_m06 = find_body(root, 'Weapon_Motor_Mount_3mm').shape
r3_pulley = find_body(root, 'Motor_Pulley_D30_1to1').shape
r3_spacers = {n: find_body(root, n).shape for n in ('Spacer_Outer_L_ID8.2_OD11', 'Spacer_Inner_ID8.2_OD11', 'Spacer_Outer_R_ID8.2_OD11')}
removed = []


def prune(p):
    for c in list(p.children):
        if c.body is not None and c.body.name.startswith(REMOVE):
            removed.append(c.body.name)
            c.body = None
        prune(c)
        if c.body is None and not c.children:
            p.children.remove(c)


prune(root)
modified = []
for b in root.bodies():
    if b.name.startswith('Beater_Disc'):
        b.material = 'steel'                              # SS400 6 mm instead of Hardox: same density, same outline

# ---------------------------------------------------------------- new geometry
grp = group(root, '01_R5_AL')
new = []


def add(name, shape, material, thk=None):
    comp = part(grp, name)
    comp.body = Body(shape.clean(), name, material, comp)
    new.append(comp.body)
    if thk is not None:
        THK[name] = thk
    return comp.body


def tube_x(name, x0, x1, y0, z0):
    s = block(x0, x1, y0, y0 + TUBE, z0, z0 + TUBE).cut(block(x0 - 1, x1 + 1, y0 + WALL, y0 + TUBE - WALL, z0 + WALL, z0 + TUBE - WALL))
    return add(name, s, 'al_tube', WALL)


def tube_y(name, x0, y0, y1, z0):
    s = block(x0, x0 + TUBE, y0, y1, z0, z0 + TUBE).cut(block(x0 + WALL, x0 + TUBE - WALL, y0 - 1, y1 + 1, z0 + WALL, z0 + TUBE - WALL))
    return add(name, s, 'al_tube', WALL)


def hole_x(shape, y, z, r):
    return shape.cut(cq.Solid.makeCylinder(r, 400, cq.Vector(-200, y, z), cq.Vector(1, 0, 0)))


def mirror_x(shape, s):
    return shape if s > 0 else shape.mirror('YZ')


# -- frame tubes (NoCage: no posts, no top rear cross)
REAR_CROSS_Y = REAR_Y + SKIN                           # -108..-96
tube_x('Frame_Rear_Cross', -(WALL_X - SKIN), WALL_X - SKIN, REAR_CROSS_Y, RAIL_Z0)
tube_x('Frame_Bulkhead_Cross', -RAIL_X[0], RAIL_X[0], CROSS_Y[0], RAIL_Z0)
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (RAIL_X[1], WALL_X - SKIN) if s > 0 else (-(WALL_X - SKIN), -RAIL_X[1])
    tube_x('Frame_Bulkhead_Stub_' + side, x0, x1, CROSS_Y[0], RAIL_Z0)
    tube_y('Frame_Rail_' + side, RAIL_X[0] if s > 0 else -RAIL_X[1], REAR_CROSS_Y + TUBE, BEAM_Y[0], RAIL_Z0)
    x_in, x_out = HINGE_BEAM_X if s > 0 else (-HINGE_BEAM_X[1], -HINGE_BEAM_X[0])
    tube_x('Frame_Hinge_Beam_' + side, x_in, x_out, BEAM_Y[0], RAIL_Z0)
tube_x('Frame_Motor_Cross', -RAIL_X[0], RAIL_X[0], MOTOR_CROSS_Y[0], RAIL_Z0)
tube_x('Frame_Clamp_Bar', -14.0, 14.0, MOTOR_CROSS_Y[0], 45.0)

# -- tabs (1.6, rectangles; holes drilled by hand)
DRILL_AFTER_WELD = []
STACK_DRILL = []          # coaxial pairs drilled together BEFORE welding, then tacked on a rod
AS_WELDED = {}
for side, s in (('L', -1), ('R', 1)):
    x0 = 56.0 if s > 0 else -56.0 - TAB
    t = block(x0, x0 + TAB, FRONT_Y - 16, FRONT_Y + 16, RAIL_Z1, RAIL_Z1 + 32)
    t = hole_x(t, FRONT_Y, 32.0, 4.25)                    # D8.5 over the D7 boss: stack-drilled L+R, tacked on a D8 rod
    AS_WELDED['Tab_Drive_Motor_' + side] = t
    for dy, dz in MOTOR_HOLES_YZ:
        t = hole_x(t, FRONT_Y + dy, 32.0 + dz, 1.8)
        DRILL_AFTER_WELD.append({'part': 'Tab_Drive_Motor_' + side, 'd_mm': 3.6, 'what': 'JB361 M3 mount', 'yz_mm': [FRONT_Y + dy, 32.0 + dz]})
    for dy, dz in GEARBOX_SCREWS_YZ:
        t = hole_x(t, FRONT_Y + dy, 32.0 + dz, 2.75)
        DRILL_AFTER_WELD.append({'part': 'Tab_Drive_Motor_' + side, 'd_mm': 5.5, 'what': 'JB361 gearbox screw head', 'yz_mm': [FRONT_Y + dy, 32.0 + dz]})
    for y0_, y1_ in ((FRONT_Y - 16, FRONT_Y - 13), (FRONT_Y + 13, FRONT_Y + 16)):
        t = t.fuse(block(*((RAIL_X[0], 56.0 + TAB) if s > 0 else (-56.0 - TAB, -RAIL_X[0])), y0_, y1_, RAIL_Z1, RAIL_Z1 + TAB))
    add('Tab_Drive_Motor_' + side, t.clean(), 'al_tab', TAB)
    # wedgelet hinge tabs on the beam front, D3.2 for the M3 bolt pin; top-front corner one straight chamfer
    for i, (a, b) in enumerate(HINGE_TAB_X):
        x0, x1 = (a, b) if s > 0 else (-b, -a)
        t = block(x0, x1, BEAM_Y[1], 105.0, 8.0, 17.7)
        cham = cq.Workplane(cq.Plane(origin=(x0 - 1, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))).polyline(
            [(100.6, 17.8), (105.1, 17.8), (105.1, 14.6)]).close().extrude(x1 - x0 + 2).val()
        # bolting leg on the beam front face, below the pin bolt, on the side away from the wedgelet ears (bent 5052 2 mm, one M3 rivnut)
        lx = {0: (HINGE_BEAM_X[0], a), 1: (b, b + 6.0), 2: (b, HINGE_BEAM_X[1])}[i]
        leg = block(*(lx if s > 0 else (-lx[1], -lx[0])), BEAM_Y[1], BEAM_Y[1] + TAB, 6.5, 11.0)   # under the pin bolt head/nut (z >= 11.3)
        add('Tab_Hinge_%s%d' % (side, i + 1), hole_x(t.cut(cham), PIN_Y, PIN_Z, 1.6).fuse(leg).clean(), 'al_tab', TAB)
    for y in (-50.0, -31.0):
        x0 = RAIL_X[0] - TAB if s > 0 else -RAIL_X[0]
        add('Tab_Battery_Strap_%s_%d' % (side, -y), block(x0, x0 + TAB, y - 4, y + 4, RAIL_Z0, RAIL_Z1), 'al_tab', TAB)
    # wall tabs (NoCage): stand on the rear cross / bulkhead stub ends, flush with the wall's inner face, one M3 each
    for nm, y0 in (('Rear', REAR_CROSS_Y), ('Front', CROSS_Y[0])):
        x0 = WALL_X - SKIN - TAB if s > 0 else -(WALL_X - SKIN)
        t = hole_x(block(x0, x0 + TAB, y0, y0 + TUBE, RAIL_Z1, RAIL_Z1 + 8), y0 + TUBE / 2, RAIL_Z1 + 4, 1.7)
        add('Tab_Wall_%s_%s' % (nm, side), t, 'al_tab', TAB)
# mid-wall support (R4-DIY open issue: 7.5 mm wall deflection > 3 mm): a 12.7 Al stub from the rail's outer face to
# the wall at y MIDWALL_Y with a 2 mm tab, so the PC side wall spans ~79 mm instead of 158
MIDWALL_Y = -55.0          # clear of the power switch (x 73, y -30); spans 40 and 92 mm instead of 145
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (RAIL_X[1], WALL_X - SKIN - TAB) if s > 0 else (-(WALL_X - SKIN - TAB), -RAIL_X[1])
    tube_x('Frame_Midwall_Stub_' + side, x0, x1, MIDWALL_Y, RAIL_Z0)
    xt = WALL_X - SKIN - TAB if s > 0 else -(WALL_X - SKIN)
    add('Tab_Wall_Mid_' + side, hole_x(block(xt, xt + TAB, MIDWALL_Y, MIDWALL_Y + TUBE, RAIL_Z0, RAIL_Z1 + 8), MIDWALL_Y + TUBE / 2, RAIL_Z1 + 4, 1.7), 'al_tab', TAB)
# drive-motor clamp (W1 pattern on the 12 tube): EPDM pads, a centre post and the clamp bar
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (6.2, 14.0) if s > 0 else (-14.0, -6.2)
    add('Pad_Drive_Motor_EPDM_' + side, block(x0, x1, 11.0, 21.0, RAIL_Z1, 19.5), 'EPDM')
    add('Pad_Drive_Motor_Top_EPDM_' + side, block(x0, x1, 11.0, 21.0, 44.5, 45.0), 'EPDM')
add('Clamp_Drive_Motor_Post', block(-TAB / 2, TAB / 2, MOTOR_CROSS_Y[0], MOTOR_CROSS_Y[1], RAIL_Z1, 45.0), 'al_tab', TAB)

# -- weapon module: R3's 6061 uprights/braces stay (bolted). Per side: one 12.7 Al tube rung from the upright's outer
# face to the rail's inner face (one M3 8.8 through rail, rung and into the tapped 6 mm upright), and one 2 mm Al angle
# foot: leg A on the bulkhead cross front face (rivnut), leg B on the upright's outer face (tapped).
JOINTS = []               # bolted joints that replace R4-DIY welds: (name, parts, bolts, where)
RUNG_Y = (BEAM_Y[0] - 0.6 - TUBE, BEAM_Y[0] - 0.6)          # ends 0.6 short of the hinge beam
RY = round(sum(RUNG_Y) / 2, 2)
ZM = round((RAIL_Z0 + RAIL_Z1) / 2, 2)
ANG_T = 3.0                                                 # 6061 angles on the weapon path (bearing / tear-out)
WEAPON_BOLTS = []                                           # (side, name, y, z): x-direction M4 8.8 into the tapped upright
for side, s_ in (('L', -1), ('R', 1)):
    # solid 12.7 6061 square bar spacer: the M4 is supported over its length and sees shear at the upright face
    add('Rung_Al_Bar_Solid_' + side, mirror_x(block(UPR_OUT, RAIL_X[0], RUNG_Y[0], RUNG_Y[1], RAIL_Z0, RAIL_Z1), s_), 'al_bar', TUBE)
    WEAPON_BOLTS.append((side, 'rung M4x50 through rail + bar', RY, ZM))
    # riser angle: vertical leg up the upright's outer face, horizontal leg on the rung + rail tops (M4 into a rail rivnut)
    rv = block(UPR_OUT, UPR_OUT + ANG_T, RUNG_Y[0], RUNG_Y[1], RAIL_Z1, 36.0)
    rh = block(UPR_OUT, RAIL_X[1] - 1.0, RUNG_Y[0], RUNG_Y[1], RAIL_Z1, RAIL_Z1 + ANG_T)
    add('Upright_Riser_Angle_6061_' + side, mirror_x(rv.fuse(rh).clean(), s_), 'al_tab', ANG_T)
    WEAPON_BOLTS.append((side, 'riser angle vertical leg', RY, 30.0))
    # foot angle: leg A on the bulkhead cross front face (2 x M3 rivnuts), leg B on the upright (z <= 24: clear of the L motor can)
    fa = block(UPR_OUT, RAIL_X[0], CROSS_Y[1] + TUBE - 12.0, CROSS_Y[1] + TUBE - 12.0 + ANG_T, RAIL_Z0, RAIL_Z1)
    fb = block(UPR_OUT, UPR_OUT + ANG_T, CROSS_Y[1] + TUBE - 12.0, RUNG_Y[0] - 0.3, RAIL_Z0, 24.0)
    add('Upright_Foot_Angle_6061_' + side, mirror_x(fa.fuse(fb).clean(), s_), 'al_tab', ANG_T)
    WEAPON_BOLTS.append((side, 'foot angle leg B', round((CROSS_Y[1] + TUBE - 12.0 + ANG_T + RUNG_Y[0] - 0.3) / 2, 2), 20.0))
    JOINTS.append(('upright -> frame, ' + side, ['Upright_6061_6mm_' + side, 'Rung_Al_Bar_Solid_' + side, 'Upright_Riser_Angle_6061_' + side,
                   'Upright_Foot_Angle_6061_' + side], 3, '3 x M4 8.8 into the tapped (M4) upright + riser M4 into a rail rivnut + foot leg A 2 x M3 rivnuts'))
# weapon motor mount: R3 M06 (6061 3 mm) clear of the crosses, on two 2 mm Al angles bolted to the motor cross and the
# bulkhead cross top faces
m06 = r3_m06.cut(block(-46, -40, CROSS_Y[0], CROSS_Y[1] + TUBE - 12.0 + 0.7, 0, RAIL_Z1 + 0.3))
m06 = m06.cut(block(-46, -40, MOTOR_CROSS_Y[0], MOTOR_CROSS_Y[0] + TUBE, 0, RAIL_Z1 + 0.3)).clean()
add('Weapon_Motor_Mount_6061_3mm', m06, 'al_tab', 3.0)
for nm, (y0, y1) in (('Front', (CROSS_Y[0] + 1.0, CROSS_Y[0] + TUBE - 1.0)), ('Mid', (35.0, 49.0))):
    # outboard of the mount (the weapon can is inboard): leg on the rail/cross tops, leg up the mount's outer face (x -45)
    leg_h = block(-56.0, -45.0, y0, y1, RAIL_Z1, RAIL_Z1 + TAB)
    leg_v = block(-45.0 - TAB, -45.0, y0, y1, RAIL_Z1, RAIL_Z1 + 12.0)
    add('Motor_Mount_Angle_Al_' + nm, leg_h.fuse(leg_v).clean(), 'al_tab', TAB)
    JOINTS.append(('M06 -> frame via %s angle' % nm.lower(), ['Weapon_Motor_Mount_6061_3mm', 'Motor_Mount_Angle_Al_' + nm], 2,
                   'one M3 down into a rivnut in the rail top, one M3 through the mount'))
# Hall post: R4-DIY outline, printed PETG 3 mm, screwed to the bulkhead cross top
hall = prism([('poly', [(55.5, RAIL_Z1), (61.5, RAIL_Z1), (61.5, 53.1), (45.5, 53.1), (45.5, 51.5), (55.5, 51.5)])], 'x', 8.0, 11.0)
add('Hall_Post_PETG_3mm', hall, 'petg_print')

# -- rotor without lathe work: 3/4" black pipe (OD 26.7, wall 2.3) 40 long between the discs, discs welded to it;
# the R3 608s sit inside the pipe ends (ID 22.1 vs 608 OD 22.0: retaining compound); belt on the pipe between printed flanges
AX = cq.Vector(1, 0, 0)
pipe = cq.Solid.makeCylinder(26.7 / 2, 40.0, cq.Vector(-20.0, 95.0, 32.0), AX).cut(cq.Solid.makeCylinder(22.1 / 2, 42.0, cq.Vector(-21.0, 95.0, 32.0), AX))
add('Rotor_Pipe_Steel_3-4in_L40', pipe, 'steel_pipe')
for nm, x0 in (('L', -4.5), ('R', 3.0)):
    fl = cq.Solid.makeCylinder(16.0, 1.5, cq.Vector(x0, 95.0, 32.0), AX).cut(cq.Solid.makeCylinder(26.8 / 2, 3.0, cq.Vector(x0 - 0.5, 95.0, 32.0), AX))
    add('Belt_Flange_PETG_' + nm, fl, 'petg_print')
for n_, sh in r3_spacers.items():
    add(n_.replace('Spacer_', 'Spacer_PETG_').replace('_ID8.2_OD11', ''), sh, 'petg_print')
add('Motor_Pulley_PETG_D30', r3_pulley, 'petg_print')

# -- wedgelets: SS400 1.6, no countersunk holes; two ears bent 90 deg down from the blank carry the M3 bolt pin.
# Ear = convex hull of a R3.4 boss about the pin (stays put while swinging: 0.6 mm from the beam face at y 98) and the
# underside from the rear edge to 10 mm down-slope; the tongue's slot is cut beside the fold.
spec = dict(config['design']['wedgelet'], hole_x_mm=[])
a = math.atan2(spec['underside_rear_yz_mm'][1] - spec['underside_tip_yz_mm'][1], spec['underside_tip_yz_mm'][0] - spec['underside_rear_yz_mm'][0])
U = np.array([math.cos(a), -math.sin(a)]); N = np.array([math.sin(a), math.cos(a)])
REAR = np.array(spec['underside_rear_yz_mm'], float)
EAR_S, EAR_R, EAR_SLOT = 10.0, 3.4, 8.0
EARS_X = ((39.1, 39.1 + WEDGE_T, +1), (84.5 - WEDGE_T, 84.5, -1))   # (x0, x1, slot direction): next to tabs 1 and 3


def hull(pts):
    pts = sorted(set(pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, hi = half(pts), half(reversed(pts))
    return lo[:-1] + hi[:-1]


ear_pts = [tuple(np.round(REAR + 0.8 * N, 4)), tuple(np.round(REAR + EAR_S * U + 0.8 * N, 4))]   # 0.8 into the plate: one solid
ear_pts += [(round(PIN_Y + EAR_R * math.cos(t), 4), round(PIN_Z + EAR_R * math.sin(t), 4)) for t in np.linspace(0, 2 * math.pi, 49)[:-1]]
EAR_YZ = hull(ear_pts)
for side, s in (('L', -1), ('R', 1)):
    w, _ = wedgelet_plate(spec, s)
    for x0, x1, d in EARS_X:
        sx0, sx1 = (x1, x1 + EAR_SLOT) if d > 0 else (x0 - EAR_SLOT, x0)
        corner = [tuple(REAR - N * 0.5), tuple(REAR + U * EAR_S - N * 0.5), tuple(REAR + U * EAR_S + N * 3), tuple(REAR + N * 3)]
        w = w.cut(mirror_x(prism([('poly', corner)], 'x', sx0, sx1), s))
        ear = prism([('poly', EAR_YZ)], 'x', x0, x1)
        ear = hole_x(ear, PIN_Y, PIN_Z, 1.6)
        w = w.fuse(mirror_x(ear, s))
    w = w.clean()
    add('Wedgelet_DIY_' + side, w, 'steel_wedgelet', WEDGE_T)
    # pin: M3x55 socket head from the inner tab, nylock beyond the outer tab
    x_head0 = HINGE_TAB_X[0][0] - 3.0
    bolt = cq.Solid.makeCylinder(2.75, 3.0, cq.Vector(x_head0, PIN_Y, PIN_Z), cq.Vector(1, 0, 0))
    bolt = bolt.fuse(cq.Solid.makeCylinder(1.5, 55.0, cq.Vector(HINGE_TAB_X[0][0], PIN_Y, PIN_Z), cq.Vector(1, 0, 0)))
    bolt = bolt.fuse(cq.Solid.makeCylinder(3.2, 4.0, cq.Vector(HINGE_TAB_X[2][1], PIN_Y, PIN_Z), cq.Vector(1, 0, 0)))
    add('Hinge_Bolt_M3x55_' + side, mirror_x(bolt, s), 'steel_bolt')

# -- skirt brackets (aluminium, unchanged from W1 except the rear rivnut height on the 12 tube)
SKIRT_N = (-0.819, -0.574)
SKIRT_FACE = {'side': (96.83, 8.26), 'rear': (118.83, 8.27)}
SKIRT_HOLE = {'side': (94.43, 15.17), 'rear': (116.43, 15.17)}
WALL_OUT = {'side': WALL_X, 'rear': -REAR_Y}
BRACKET_W = 15.0
skirt_brackets = []


def skirt_bracket(kind, sgn, station, bolt_dt=0.0, bolt_z=16.0):
    n = np.array(SKIRT_N); n /= np.linalg.norm(n)
    face = np.array(SKIRT_FACE[kind]); hole = np.array(SKIRT_HOLE[kind])
    u_leg = WALL_OUT[kind] + BRACKET_T
    z_top = face[1] + n[0] * (u_leg - face[0]) / -n[1]
    p0 = hole - np.dot(hole - face, n) * n
    s_dir = np.array([n[1], -n[0]]) * (-1 if n[1] * 1 < 0 else 1)
    if s_dir[0] < 0: s_dir = -s_dir
    a_ = np.array([u_leg, z_top]) + n * 0.1; b_ = p0 + s_dir * 4.0 + n * 0.1
    z_low = min(bolt_z - 4.0, z_top - 8.0)
    prof_leg = [(WALL_OUT[kind], z_low), (u_leg, z_low), (u_leg, z_top), (WALL_OUT[kind], z_top)]
    prof_seat = [tuple(a_), tuple(b_), tuple(b_ + n * BRACKET_T), tuple(a_ + n * BRACKET_T)]

    def solid(prof, t0, t1):
        wp = cq.Workplane('XZ').polyline([(u, z) for u, z in prof]).close().extrude(-(t1 - t0))
        return wp.val().translate(cq.Vector(0, t0, 0))
    t0, t1 = station - BRACKET_W / 2, station + BRACKET_W / 2
    sh = solid(prof_leg, t0, t1).fuse(solid(prof_seat, t0, t1)).clean()
    sh = sh.cut(cq.Solid.makeCylinder(1.7, 10, cq.Vector(WALL_OUT[kind] - 2, station + bolt_dt, bolt_z), cq.Vector(1, 0, 0)))
    sh = sh.cut(cq.Solid.makeCylinder(1.2, 10, cq.Vector(p0[0] - 5 * n[0], station, p0[1] - 5 * n[1]), cq.Vector(n[0], 0, n[1])))
    if kind == 'side':
        world = sh if sgn > 0 else sh.mirror('YZ')
    else:
        world = sh.rotate(cq.Vector(), cq.Vector(0, 0, 1), -90)
    flat = {'width': BRACKET_W, 'wall_leg': round(z_top - z_low, 2), 'seat_leg': round(float(np.linalg.norm(b_ - a_)), 2),
            'bend_deg_from_flat': round(float(np.degrees(np.arctan2(s_dir[0], -s_dir[1]))), 1),
            'm3_hole_from_leg_end': round(bolt_z - z_low, 2), 'm3_hole_offset_from_centre': bolt_dt,
            'm2_hole_from_bend': round(float(np.linalg.norm(p0 - a_)), 2)}
    return world, flat


for side, s in (('L', -1), ('R', 1)):
    for y in (-82.5, -27.5, 32.5, 62.5):
        sh, flat = skirt_bracket('side', s, y)
        b = add('Skirt_Bracket_%s_y%+.0f' % (side, y), sh, 'al5052_bracket', BRACKET_T)
        skirt_brackets.append((b.name, flat))
for x in (-57.5, 0.0, 57.5):
    sh, flat = skirt_bracket('rear', 1, x, bolt_z=RAIL_Z0 + TUBE / 2)
    b = add('Skirt_Bracket_Rear_x%+.0f' % x, sh, 'al5052_bracket', BRACKET_T)
    skirt_brackets.append((b.name, flat))

# -- PC 2 mm skins (plastic/acrylic shop, or cut at home); side walls run to the hinge beam
floor = block(-WALL_X, WALL_X, REAR_Y + SKIN, CROSS_Y[1], FLOOR_Z0, RAIL_Z0)
for s in (-1, 1):
    x0, x1 = (34.0, WALL_X) if s > 0 else (-WALL_X, -34.0)
    floor = floor.fuse(block(x0, x1, CROSS_Y[1] - 1, BEAM_Y[1], FLOOR_Z0, RAIL_Z0))
    floor = floor.cut(block(*((WHEEL_X[0] - 1, WHEEL_X[1] + 1) if s > 0 else (-WHEEL_X[1] - 1, -WHEEL_X[0] + 1)), FRONT_Y - 21, FRONT_Y + 21, 0, 10))
add('Skin_PC2_Floor', floor, 'PC', SKIN)
add('Skin_PC2_Rear_Wall', block(-WALL_X, WALL_X, REAR_Y, REAR_Y + SKIN, FLOOR_Z0, LID_Z0), 'PC', SKIN)
for side, s in (('L', -1), ('R', 1)):
    x0 = WALL_X - SKIN if s > 0 else -WALL_X
    add('Skin_PC2_Side_Wall_' + side, block(x0, x0 + SKIN, REAR_Y + SKIN, BEAM_Y[0], RAIL_Z0, LID_Z0), 'PC', SKIN)
lid = block(-WALL_X, WALL_X, REAR_Y, CROSS_Y[1], LID_Z0, LID_Z0 + SKIN)
for s in (-1, 1):
    lid = lid.cut(block(*((WHEEL_X[0] - 1, WHEEL_X[1] + 1) if s > 0 else (-WHEEL_X[1] - 1, -WHEEL_X[0] + 1)), FRONT_Y - 23, FRONT_Y + 23, 0, 80))
add('Skin_PC2_Lid', lid, 'PC', SKIN)
add('Electronics_Plate_PC2', block(-58, 58, -56, -1.5, 37.6, 39.6), 'PC', SKIN)


def wheel_hub(sgn):
    """Hub (printed PETG, 100 %, web 3 mm) + 2 O-rings. Built for the R side, mirrored for L."""
    x0, x1 = WHEEL_X
    g, lw = HUB['groove_w'], HUB['land_w']
    xc = lambda x: cq.Vector(x, FRONT_Y, 32.0)
    ax = cq.Vector(1, 0, 0)
    cyl = lambda r, a_, b_: cq.Solid.makeCylinder(r, b_ - a_, xc(a_), ax)
    rim = cyl(HUB['land_d'] / 2, x0, x1).cut(cyl(HUB['rim_id'] / 2, x0 - 1, x1 + 1))
    grooves = [(x0 + lw[0], x0 + lw[0] + g), (x0 + lw[0] + g + lw[1], x0 + lw[0] + 2 * g + lw[1])]
    for a_, b_ in grooves:
        rim = rim.cut(cyl(HUB['land_d'] / 2 + 1, a_, b_).cut(cyl(HUB['groove_root_d'] / 2, a_ - 1, b_ + 1)))
    web = cyl(HUB['rim_id'] / 2 + 0.01, x1 - HUB['web_t'], x1)
    n, hr, pr = HUB['holes']
    for k in range(n):
        an = math.radians(60 * (k + 1))
        web = web.cut(cq.Solid.makeCylinder(hr, 10, cq.Vector(x1 - 5, FRONT_Y + pr * math.cos(an), 32 + pr * math.sin(an)), ax))
    hub = rim.fuse(web).fuse(cyl(HUB['boss_d'] / 2, x0, x1)).clean()
    db = cq.Workplane('YZ').polyline(d_bore(FRONT_Y, 32.0, 4.10 / 2, 1.55, 144)).close().extrude(x1 - x0 + 2).val()
    hub = hub.cut(db.translate(cq.Vector(x0 - 1, 0, 0)))
    xs = 56.0 + 11.5 - 2.5
    hub = hub.cut(cq.Solid.makeCylinder(1.25, 40, cq.Vector(xs, FRONT_Y, 32.0), cq.Vector(0, 1, 0)))
    hub = hub.cut(cq.Solid.makeCylinder(1.75, 10, cq.Vector(xs, FRONT_Y + HUB['rim_id'] / 2 - 2, 32.0), cq.Vector(0, 1, 0)))
    R = HUB['groove_root_d'] / 2 + ORING_CS_FITTED / 2
    rings = [cq.Solid.makeTorus(R, ORING_CS_FITTED / 2, xc((a_ + b_) / 2), ax) for a_, b_ in grooves]
    if sgn < 0:
        hub = hub.mirror('YZ'); rings = [r.mirror('YZ') for r in rings]
    return hub.clean(), rings


for side, s in (('L', -1), ('R', 1)):
    hub, rings = wheel_hub(s)
    add('Wheel_Hub_PETG_Front_' + side, hub, 'petg_print')
    for k, r in enumerate(rings):
        add('Wheel_ORing_P52_Front_%s%d' % (side, k + 1), r, 'NBR')
    for y in (-52.0, -8.0):
        add('Standoff_Nylon_M3_%s_%d' % (side, -y), cq.Solid.makeCylinder(3.0, 37.6 - RAIL_Z1, cq.Vector(s * 55.0, y, RAIL_Z1), cq.Vector(0, 0, 1)), 'nylon_standoff')
bl, bw, bh = BATTERY
add('3S_650mAh_LiPo_ENVELOPE', block(-bl / 2, bl / 2, -40.5 - bw / 2, -40.5 + bw / 2, RAIL_Z0 + 1, RAIL_Z0 + 1 + bh), 'battery_650')
add('Rear_Skid_UHMW_3mm', block(-20, 20, REAR_Y + 2, REAR_Y + 14, 1.0, FLOOR_Z0), 'UHMW', 3.0)
add('Nose_Skid_UHMW_3mm', block(-20, 20, 64.5, 74.0, 1.0, 4.0), 'UHMW', 3.0)   # under R3's bottom brace (z 4)

# ---------------------------------------------------------------- checks
def bbox_overlap(a_, b_, tol=0.01):
    A, B = a_.BoundingBox(), b_.BoundingBox()
    return A.xmin < B.xmax - tol and B.xmin < A.xmax - tol and A.ymin < B.ymax - tol and B.ymin < A.ymax - tol and A.zmin < B.zmax - tol and B.zmin < A.zmax - tol


bodies = root.bodies()
checked = new + modified
interference = []
for i, a_ in enumerate(checked):
    for o in bodies:
        if o is a_ or (o in checked and checked.index(o) <= i): continue
        if not bbox_overlap(a_.shape, o.shape): continue
        if {a_.name, o.name} & {'Dead_Shaft_M8_12.9'} and {a_.name, o.name} & {'Weapon_Axle_Nut_M8'}: continue   # thread engagement
        v = a_.shape.intersect(o.shape).Volume()
        if v > 0.01:
            interference.append({'a': a_.name, 'b': o.name, 'volume_mm3': round(v, 3)})
invalid = [b.name for b in new if not b.shape.isValid() or len(b.shape.Solids()) != 1]


def rotated(shape, deg):
    return shape.rotate(cq.Vector(0, PIN_Y, PIN_Z), cq.Vector(1, PIN_Y, PIN_Z), deg)


swing = {}
for side in ('L', 'R'):
    moving = [find_body(root, 'Wedgelet_DIY_' + side)]
    others = [o for o in bodies if o not in moving and not o.name.startswith('Hinge_Bolt') and o.material != 'keepout']
    res = {}
    for direction, steps in (('up', range(1, 61)), ('down', range(-1, -21, -1))):
        free = 0
        for deg in steps:
            hit = None
            r = rotated(moving[0].shape, deg)
            for o in others:
                if bbox_overlap(r, o.shape) and r.intersect(o.shape).Volume() > 0.05:
                    hit = o.name; break
            if hit:
                res[direction] = {'free_deg': free, 'first_contact': hit, 'at_deg': deg}
                break
            free = deg
        else:
            res[direction] = {'free_deg': free, 'first_contact': None}
    res['sim_range_deg'] = [-2.0, 14.0]
    res['ok'] = res['up']['free_deg'] >= 14 and abs(res['down']['free_deg']) >= 2
    swing[side] = res

metal = [b for b in new if b.material in ('al_tube', 'al_tab')]
SHIFT = 0.05
welds, edge_only = [], []        # 'welds' = face contacts that are now BOLTED joints
for i, a_ in enumerate(metal):
    for o in metal[i + 1:]:
        if a_.shape.distance(o.shape) > 0.01: continue
        best = (0.0, None)
        for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            c = a_.shape.intersect(o.shape.translate(cq.Vector(*d) * SHIFT))
            v = c.Volume()
            if v > best[0]: best = (v, c)
        area = best[0] / SHIFT
        if area < 5.0:
            edge_only.append({'a': a_.name, 'b': o.name, 'area_mm2': round(area, 2)})
            continue
        bb = best[1].BoundingBox()
        welds.append({'a': a_.name, 'b': o.name, 'contact_area_mm2': round(area, 1),
                      'contact_centre_mm': [round(v, 1) for v in best[1].Center().toTuple()],
                      'contact_size_mm': sorted((round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)), reverse=True)[:2]})
joined = {w['a'] for w in welds} | {w['b'] for w in welds}
floating = [b.name for b in metal if b.name not in joined and not b.name.startswith('Weapon_Motor_Mount')]

# rotor gap: discs/hub/spacers must clear the uprights' inner faces and everything welded
rotor = [find_body(root, n) for n in ('Beater_Disc_L', 'Beater_Disc_R')] + [find_body(root, 'Rotor_Pipe_Steel_3-4in_L40')]
sweep = cq.Solid.makeCylinder(29.7 + 1.0, 53.0, cq.Vector(-26.5, 95.0, 32.0), cq.Vector(1, 0, 0))     # tooth tip r 29.7 + 1 mm
ROTOR_SET = ('Rotor_Pipe', 'Belt_Flange', 'Spacer_PETG')
sweep_hits = [o.name for o in bodies if not o.name.startswith(('Beater_', 'Bearing_608', 'Dead_Shaft', 'Weapon_Axle', 'Belt_') + ROTOR_SET)
              and o.material != 'keepout' and bbox_overlap(sweep, o.shape) and sweep.intersect(o.shape).Volume() > 0.01]

# ---------------------------------------------------------------- mass and balance
root.children.remove(grp)
mp = mass_properties(root, config, {})
root.children.append(grp)
rows = []
for r in mp['parts']:
    r = dict(r)
    rows.append(r)
steel_g = 0.0
for b in new:
    c = list(b.shape.Center().toTuple())
    if b.material in EXPLICIT_G:
        g, method = EXPLICIT_G[b.material], 'ASSUMED bought-part mass'
    else:
        g, method = b.shape.Volume() * RHO[b.material], 'CAD volume x density'
        if b.material.startswith('steel'): steel_g += g
    rows.append({'part': b.name, 'material': b.material, 'mass_g': round(g, 3), 'centroid_mm': c, 'method': method})
rows.append({'part': 'rotor pipe-to-disc welds (ARC)', 'mass_g': ROTOR_WELD_G, 'centroid_mm': [0, 95, 32], 'method': 'ASSUMED'})
n_tt = sum(1 for w in welds if w['a'].startswith('Frame_') and w['b'].startswith('Frame_'))
n_tab = len(welds) - n_tt
rows.append({'part': 'bolted-joint hardware: %d tube-tube cleats, %d tab joints' % (n_tt, n_tab), 'mass_g': round(n_tt * JOINT_G['tube_tube'] + n_tab * JOINT_G['tab_tube'], 1),
             'centroid_mm': [0, -10, 12], 'method': 'ASSUMED %s g per joint' % JOINT_G})
rows.append({'part': 'rivnuts M3 + screws x%d' % RIVNUTS, 'mass_g': RIVNUTS * 1.4, 'centroid_mm': [0, -20, 20], 'method': 'ASSUMED 0.9 + 0.5 g'})
rows.append({'part': 'switch panel mount (bought)', 'mass_g': SWITCH_MOUNT_G, 'centroid_mm': [80, -30, 25], 'method': 'ASSUMED'})
rows.append({'part': 'lid corner angles, bought Al x%d + M3' % LID_ANGLES, 'mass_g': 1.5 * LID_ANGLES + 4 * 0.5, 'centroid_mm': [0, -24, 58], 'method': 'ASSUMED 1.5 g each'})
rows.append({'part': 'wall-tab M3 screws + nylocks x4, skid screws x2', 'mass_g': 6 * 0.9, 'centroid_mm': [0, 0, 15], 'method': 'ASSUMED'})
total = sum(r['mass_g'] for r in rows)
cg = sum(np.array(r['centroid_mm']) * r['mass_g'] for r in rows) / total
NOSE_SKID_Y, REAR_SKID_Y = 69.0, REAR_Y + 8
other = NOSE_SKID_Y if cg[1] > FRONT_Y else REAR_SKID_Y
wheel_frac = (cg[1] - other) / (FRONT_Y - other)
F_MOTOR, MU = 3.99144375, 0.6
w = total / 1000 * 9.81
push = 2 * min(F_MOTOR, MU * w * wheel_frac / 2) - 0.2 * w * (1 - wheel_frac)

groups = {}
for r in rows:
    m = r.get('material')
    key = ('frame tubes (Al)' if m == 'al_tube' else 'Al plates / angles' if m == 'al_tab' else 'PC skins' if m == 'PC'
           else 'PETG prints (ex-lathe)' if m == 'petg_print' else 'rotor pipe (steel)' if m == 'steel_pipe'
           else 'joint hardware' if r['part'].startswith('bolted-joint') else 'R3 parts kept + bought')
    groups[key] = round(groups.get(key, 0) + r['mass_g'], 1)
cut_list = [{'part': b.name, 'wall_mm': WALL, 'length_mm': round(max(b.shape.BoundingBox().xlen, b.shape.BoundingBox().ylen, b.shape.BoundingBox().zlen), 1)}
            for b in new if b.material == 'al_tube']
sheet_list = []
for b in new:
    if b.material not in ('al_tab', 'steel_wedgelet', 'PC', 'al5052_bracket', 'UHMW'): continue
    bb = b.shape.BoundingBox()
    dims = sorted((bb.xlen, bb.ylen, bb.zlen))
    sheet_list.append({'part': b.name, 'material': b.material, 'thickness_mm': THK.get(b.name), 'blank_mm': [round(dims[2], 1), round(dims[1], 1)],
                       'mass_g': round(b.shape.Volume() * RHO.get(b.material, 0), 2)})
report = {
    'status': 'CONCEPT_CAD', 'revision': 'V4 R5-AL (aluminium bolted frame, R3 6061 weapon module, no lathe parts)', 'orientation': 'Z-up, nose +Y, mm',
    'removed_from_r3': sorted(removed), 'invalid_bodies': invalid, 'frame_members_not_touching': floating, 'edge_only_contacts': edge_only,
    'bolted_contacts': welds, 'weapon_joints': JOINTS, 'interference': interference, 'rotor_sweep_hits': sweep_hits, 'wedgelet_swing': swing,
    'ear_yz': EAR_YZ, 'drill_list': DRILL_AFTER_WELD, 'midwall_y': MIDWALL_Y,
    'weapon_bolts_yz': WEAPON_BOLTS, 'weapon_angle_t_mm': ANG_T, 'foot_leg_b_y': [CROSS_Y[1] + TUBE - 12.0, RUNG_Y[0] - 0.3],
    'skirt_brackets_flat': dict(skirt_brackets),
    'mass': {'total_g': round(total, 1), 'margin_to_2000_g': round(2000 - total, 1), 'margin_to_1950_target_g': round(1950 - total, 1),
             'cg_mm': cg.round(2).tolist(), 'groups_g': groups, 'new_steel_g': round(steel_g, 1)},
    'support_2wd': {'drive_axle_y': FRONT_Y, 'other_support': 'nose skid y 69' if other == NOSE_SKID_Y else 'rear skid y %.0f' % REAR_SKID_Y,
                    'drive_wheel_load_pct': round(100 * wheel_frac, 1), 'push_N_mu0.6': round(push, 2)},
    'tube_cut_list_mm': cut_list, 'tube_total_mm': round(sum(c['length_mm'] for c in cut_list), 1), 'sheet_list': sheet_list,
    'bounds_mm': {k: round(v, 2) for k, v in zip(('xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'),
                  (lambda bb: (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax))(cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).BoundingBox()))},
    'rows': rows,
}
(OUT / 'build_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')

# ---------------------------------------------------------------- exports + preview
cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).exportStep(str(OUT / 'ROBOT_V4_R5_AL.step'))
asm = cq.Assembly(name='ROBOT_V4_R5_AL')
COLS = {'al_tube': (0.75, 0.78, 0.82), 'al_tab': (0.62, 0.68, 0.75), 'petg_print': (0.95, 0.55, 0.1), 'steel_pipe': (0.4, 0.42, 0.45),
        'steel': (0.4, 0.45, 0.5), 'steel_wedgelet': (0.55, 0.42, 0.25), 'PC': (0.34, 0.70, 0.69)}
for b in bodies:
    if b.material != 'keepout':
        asm.add(cq.Workplane().add(b.shape), name=b.name, color=cq.Color(*COLS.get(b.material, (0.34, 0.42, 0.64))))
asm.save(str(OUT / 'ROBOT_V4_R5_AL_ASSEMBLY.step'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
colors = {'al_tube': '#b8c2cc', 'al_tab': '#8fa3b8', 'petg_print': '#f39c12', 'steel_pipe': '#555', 'steel_wedgelet': '#8e6b3f', 'steel_bolt': '#333', 'PC': '#56b2b0', '6061': '#a6b4c5',
          'steel': '#667381', 'purchased': '#566ca3', 'al6061_hub': '#a6b4c5', 'NBR': '#2c2c2c', 'al5052_bracket': '#7f8c8d', 'battery_650': '#566ca3',
          'belt': '#343b45', 'UHMW': '#eeeeee', 'EPDM': '#222', 'nylon_standoff': '#ddd'}
fig = plt.figure(figsize=(14, 7), facecolor='#fafafa')
for index, title, skins in ((1, 'Assembly (PC skins translucent)', True), (2, 'Aluminium frame + internals (no skins)', False)):
    ax = fig.add_subplot(1, 2, index, projection='3d')
    for b in bodies:
        if b.material == 'keepout' or (not skins and b.material == 'PC' and b.name.startswith('Skin')): continue
        v, t = b.shape.tessellate(0.5, 0.3)
        v = np.array([p.toTuple() for p in v])
        alpha = 0.18 if b.name.startswith('Skin') else 0.95
        ax.add_collection3d(Poly3DCollection(v[np.array(t)], facecolors=colors.get(b.material, '#aaa'), edgecolors='none', alpha=alpha))
    ax.set_xlim(-110, 110); ax.set_ylim(-125, 135); ax.set_zlim(0, 80); ax.set_box_aspect((220, 260, 80)); ax.view_init(elev=32, azim=-55)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Front +Y (mm)'); ax.set_zlabel('Z (mm)'); ax.set_title(title, fontsize=12)
fig.suptitle('V4 R5-AL — CONCEPT CAD · %.1f g · CG y %.1f' % (total, cg[1]), fontsize=15, fontweight='bold')
fig.text(0.5, 0.03, 'Al tube 12.7x2.0: light grey | Al plate: blue-grey | 6061 (R3): grey | PETG prints: orange | steel: dark | PC 2 mm: teal', ha='center')
fig.tight_layout(rect=(0, 0.05, 1, 0.93)); fig.savefig(OUT / 'assembly_preview.png', dpi=150); plt.close(fig)

print(json.dumps({k: report[k] for k in ('invalid_bodies', 'frame_members_not_touching', 'edge_only_contacts', 'interference', 'rotor_sweep_hits',
                                          'wedgelet_swing', 'mass', 'support_2wd', 'tube_total_mm', 'bounds_mm')}, ensure_ascii=False, indent=1))
