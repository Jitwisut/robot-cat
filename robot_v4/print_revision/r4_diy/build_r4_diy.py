"""V4 R4-DIY (user decisions 2026-10-27): R4-steel set A rebuilt from common-shop stock, welded at home with a stick
(ARC) welder, every custom part replaced where hand tools can make it. Z-up, nose +Y, mm (AGENTS.md).
CONCEPT CAD for fit, mass, balance and the DIY drawings. r3/ and r4_steel/ (W1, released) are not touched.

Differences from build_r4_steel.py (W1):
  * frame tube 12x12x1.2 black square tube (ordinary steel shop) instead of 10x10x1.0/1.5; tabs 1.6 sheet, rectangles
  * NoCage (user choice 2026-10-27): no rear posts, no top rear cross; the lid sits on 4 bought aluminium angles and the
    side walls bolt to 4 small wall tabs on the rear cross and bulkhead stubs
  * weapon module welded steel: uprights 3.2 (2" x 1/8" flat bar, same outline, inner faces |x| 27 kept), one 1.6 base
    plate (old bottom brace + floor flanges) welded to the bulkhead cross and both rails, 2 rungs per side, 1.6 top brace
  * weapon motor mount 2.3 sheet (calc: 1.6 fails the 250 g drop), standing on a 1.6 foot strip between the two crosses
  * wedgelets SS400 1.6 with two hinge ears bent down from the blank (no carriers), M3x55 bolt + nylock as the pin
  * Hall post = 1.6 L strip welded on the bulkhead cross; nose skid 5 mm UHMW/HDPE
  * M8x75 axle (uprights 3.2 instead of 6), Hardox discs -> SS400 6 mm, same outline (same inertia)
  * still from a lathe shop: weapon hub, motor pulley, 3 spacers, 2 wheel hubs (unchanged geometry)
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r4_diy/build_r4_diy.py
Writes robot_v4/print_revision/r4_diy/output/.
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
TUBE, WALL = 12.0, 1.2      # 1/2" black square tube (steellead table: 2.4 kg / 6 m); not galvanised
TAB = 1.6                   # black sheet offcut, hand-cut rectangles
FLAT = 3.2                  # 2" x 1/8" flat bar: uprights
MOUNT_T = 2.3               # weapon motor mount sheet (calc/frame_strength_diy.py: 1.6 fails the 250 g drop)
BRACKET_T = 1.5             # aluminium skirt brackets (unchanged from W1)
SKIN = 2.0                  # PC skin thickness
FLOOR_Z0 = 4.0              # floor z 4..6, same underside as R3
RAIL_Z0 = FLOOR_Z0 + SKIN   # tubes stand on the floor: z 6..18
RAIL_Z1 = RAIL_Z0 + TUBE    # 18: 1.5 mm under the drive motors (z 19.5) -> 1.5 mm EPDM
LID_Z0 = 61.0
WALL_X = 83.0
REAR_Y = -110.0
CROSS_Y = (50.0, 62.0)      # bulkhead cross: front tyre reaches y 46.8 at z <= 18 (r 32 about y 18, z 32)
RAIL_X = (46.0, 58.0)       # |x|: outer face 58 as W1 (wheel hub from 59.5)
BEAM_Y = (86.0, 98.0)       # hinge beam: front face 98 as W1 (pin y 102)
MOTOR_CROSS_Y = (10.0, 22.0)
HINGE_TAB_X = ((37.0, 37.0 + TAB), (62.0, 62.0 + TAB), (85.0, 85.0 + TAB))
UPR_IN = 27.0               # upright inner face |x| (R3 stack: 54 between the faces, unchanged)
UPR_OUT = UPR_IN + FLAT
BASE_Z = (RAIL_Z0, RAIL_Z0 + TAB)                         # base plate z 6..7.6; uprights stand on it
PIN_Y, PIN_Z = 102.0, 14.5                                # wedgelet hinge axis (R3)
WHEEL_R, WHEEL_X = 32.0, (59.5, 75.5)
MOTOR_HOLES_YZ = [(-8.5, 0.0), (8.5, 0.0)]                # JB361 face: 2 x M3 at +-8.5 about the shaft
GEARBOX_SCREWS_YZ = [(0.0, -8.5), (0.0, 8.5)]
BATTERY = (57.0, 31.0, 24.0)                              # 3S 650 mAh envelope, ASSUMED
RHO = {'steel_tube': 0.00785, 'steel_tab': 0.00785, 'steel_wedgelet': 0.00785, 'steel_bolt': 0.00785, 'PC': 0.00120,
       'UHMW': 0.00094, 'EPDM': 0.00130, 'al5052_bracket': 0.00268, 'al6061_hub': 0.00270, 'NBR': 0.00130}
EXPLICIT_G = {'battery_650': 58.0, 'nylon_standoff': 0.6}
ORING_ID, ORING_CS = 51.6, 5.7
HUB = {'groove_root_d': 52.8, 'land_d': 57.0, 'rim_id': 49.0, 'web_t': 2.0, 'boss_d': 12.0, 'groove_w': 6.6,
       'land_w': (1.0, 0.8, 1.0), 'holes': (5, 4.5, 15.5)}
ORING_CS_FITTED = ORING_CS * (1 - 0.5 * (HUB['groove_root_d'] - ORING_ID) / ORING_ID)
RIVNUTS = 24                # M3 steel rivnut 0.9 g + screw 0.5 g (floor + rear wall into the tubes)
SWITCH_MOUNT_G = 3.0
LID_ANGLES = 4              # bought aluminium angles: lid to walls (NoCage)
WELD = 0.08                 # ARC beads + spatter + paint, fraction of steel: ASSUMED (calc/diy_substitution_calc.py)
THK = {}                    # part -> stock thickness, for the cut list

# ---------------------------------------------------------------- R3 in memory, minus what DIY replaces
root, _ = build(config)
REMOVE = ('Tub_TPU', 'Lid_TPU_2mm', 'JGA25_370_400rpm_Rear_', 'Wheel_', 'Battery_Tray_PETG', 'Electronics_Platform_PETG',
          'Electronics_Standoff_PETG', 'Wedgelet_', 'Weapon_Mount_Foot_PETG', 'Disconnect_Collar_PETG',
          '3S_850mAh_LiPo_ENVELOPE', 'Upright_6061', 'Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Weapon_Motor_Mount',
          'Nose_Skid', 'Hall_Post_PETG', 'Hinge_Pin_D3', 'Eclip_')
r3_m06 = find_body(root, 'Weapon_Motor_Mount_3mm').shape
r3_brace = find_body(root, 'Weapon_Top_Brace_3mm').shape
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
# axle stack: uprights 3.2 instead of 6 -> head and nut move 2.8 mm in; M8x75 instead of x80
modified = []
for name, dx in (('Dead_Shaft_M8_12.9', 2.8), ('Weapon_Axle_Washer_L', 2.8), ('Weapon_Axle_Washer_R', -2.8), ('Weapon_Axle_Nut_M8', -2.8)):
    b = find_body(root, name)
    b.shape = b.shape.translate(cq.Vector(dx, 0, 0))
    modified.append(b)
shaft = find_body(root, 'Dead_Shaft_M8_12.9')
shaft.shape = shaft.shape.cut(block(UPR_OUT + 1.6 + 75.0 - 2 * (UPR_OUT + 1.6), 100, 0, 200, 0, 100)).clean()   # 75 under the head
for b in root.bodies():
    if b.name.startswith('Beater_Disc'):
        b.material = 'steel'                              # SS400 6 mm instead of Hardox: same density, same outline

# ---------------------------------------------------------------- new geometry
grp = group(root, '01_R4_DIY')
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
    return add(name, s, 'steel_tube', WALL)


def tube_y(name, x0, y0, y1, z0):
    s = block(x0, x0 + TUBE, y0, y1, z0, z0 + TUBE).cut(block(x0 + WALL, x0 + TUBE - WALL, y0 - 1, y1 + 1, z0 + WALL, z0 + TUBE - WALL))
    return add(name, s, 'steel_tube', WALL)


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
    x_in, x_out = (HINGE_TAB_X[0][0], 88.0) if s > 0 else (-88.0, -HINGE_TAB_X[0][0])
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
    add('Tab_Drive_Motor_' + side, t, 'steel_tab', TAB)
    # wedgelet hinge tabs on the beam front, D3.2 for the M3 bolt pin; top-front corner one straight chamfer
    for i, (a, b) in enumerate(HINGE_TAB_X):
        x0, x1 = (a, b) if s > 0 else (-b, -a)
        t = block(x0, x1, BEAM_Y[1], 105.0, 8.0, 17.7)
        cham = cq.Workplane(cq.Plane(origin=(x0 - 1, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))).polyline(
            [(100.6, 17.8), (105.1, 17.8), (105.1, 14.6)]).close().extrude(x1 - x0 + 2).val()
        add('Tab_Hinge_%s%d' % (side, i + 1), hole_x(t.cut(cham), PIN_Y, PIN_Z, 1.6), 'steel_tab', TAB)
    for y in (-50.0, -31.0):
        x0 = RAIL_X[0] - TAB if s > 0 else -RAIL_X[0]
        add('Tab_Battery_Strap_%s_%d' % (side, -y), block(x0, x0 + TAB, y - 4, y + 4, RAIL_Z0, RAIL_Z1), 'steel_tab', TAB)
    # wall tabs (NoCage): stand on the rear cross / bulkhead stub ends, flush with the wall's inner face, one M3 each
    for nm, y0 in (('Rear', REAR_CROSS_Y), ('Front', CROSS_Y[0])):
        x0 = WALL_X - SKIN - TAB if s > 0 else -(WALL_X - SKIN)
        t = hole_x(block(x0, x0 + TAB, y0, y0 + TUBE, RAIL_Z1, RAIL_Z1 + 8), y0 + TUBE / 2, RAIL_Z1 + 4, 1.7)
        add('Tab_Wall_%s_%s' % (nm, side), t, 'steel_tab', TAB)
# drive-motor clamp (W1 pattern on the 12 tube): EPDM pads, a centre post and the clamp bar
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (6.2, 14.0) if s > 0 else (-14.0, -6.2)
    add('Pad_Drive_Motor_EPDM_' + side, block(x0, x1, 11.0, 21.0, RAIL_Z1, 19.5), 'EPDM')
    add('Pad_Drive_Motor_Top_EPDM_' + side, block(x0, x1, 11.0, 21.0, 44.5, 45.0), 'EPDM')
add('Clamp_Drive_Motor_Post', block(-TAB / 2, TAB / 2, MOTOR_CROSS_Y[0], MOTOR_CROSS_Y[1], RAIL_Z1, 45.0), 'steel_tab', TAB)

# -- weapon module, welded
# R3 upright outline (DXF Upright_6061_6mm_*, world y, z), bottom raised onto the base plate and a foot back to the
# bulkhead cross's front face (y 62) so the fore-aft hit goes straight into the cross
ZB = BASE_Z[1]
UPRIGHT_YZ = [(CROSS_Y[1], ZB), (98.0, ZB), (98.0, 25.0), (100.0, 25.0), (112.0, 16.5), (112.0, 40.0), (104.0, 57.0),
              (64.0, 57.0), (64.0, RAIL_Z1), (CROSS_Y[1], RAIL_Z1)]
for side, s in (('L', -1), ('R', 1)):
    u = prism([('poly', UPRIGHT_YZ)], 'x', UPR_IN, UPR_OUT)
    u = hole_x(u, 95.0, 32.0, 4.0)                        # D8.0 stack-drilled L+R before welding (M8 bolt, snug)
    u = mirror_x(u, s)
    if s < 0:                                             # R3 L relief for the weapon motor can (D35 + 1 about y 47.5, z 42.5)
        u = u.cut(cq.Solid.makeCylinder(18.5, 20, cq.Vector(-40, 47.5, 42.5), cq.Vector(1, 0, 0)))
    add('Upright_Steel_' + side, u, 'steel_tab', FLAT)
    # two rungs from the upright's outer face to the rail's inner face (W1 stations, clear of the axle nut y >= 87.5)
    for k, yr in enumerate((68.5, 81.0)):
        add('Rung_%d_%s' % (k + 1, side), mirror_x(block(UPR_OUT, RAIL_X[0], yr, yr + TAB, ZB, 28.0), s), 'steel_tab', TAB)
STACK_DRILL.append({'parts': ['Upright_Steel_L', 'Upright_Steel_R'], 'd_mm': 8.0, 'axis_yz': [95.0, 32.0],
                    'jig': 'tack on an M8 bolt with a 54.0 mm dummy spacer between the inner faces; full-weld with it in; never weld with bearings or the rotor fitted'})
STACK_DRILL.append({'parts': ['Tab_Drive_Motor_L', 'Tab_Drive_Motor_R'], 'd_mm': 8.5, 'axis_yz': [FRONT_Y, 32.0],
                    'jig': 'tack on a D8 rod through both plates'})
# base plate = R3 bottom brace + W1 floor flanges in one 1.6 sheet: centre y 62..74 (under the discs, which reach z 11
# at y 74), side arms |x| 27..46 to the hinge beam (y 86); welded to the cross front face and both rail inner faces
base = block(-RAIL_X[0], RAIL_X[0], CROSS_Y[1], 74.0, *BASE_Z)
for s in (-1, 1):
    base = base.fuse(mirror_x(block(UPR_IN, RAIL_X[0], 74.0 - 1, BEAM_Y[0], *BASE_Z), s))
for x in (-15.0, 15.0):                                   # nose-skid M3 countersunk screws + nut on top
    base = base.cut(cq.Solid.makeCylinder(1.7, 10, cq.Vector(x, 69.0, 0), cq.Vector(0, 0, 1)))
add('Weapon_Base_Plate', base, 'steel_tab', TAB)
# top brace: R3 outline on the upright tops (z 57), 1.6, same belt slot
slot = block(-3.5, 3.5, 64.0, 77.0, 57.0, 60.0).cut(r3_brace)
sb = slot.BoundingBox()
belt_slot = (round(sb.ymin, 2), round(sb.ymax, 2)) if slot.Volume() > 1 and sb.ylen < 12.9 else None
top = block(-UPR_OUT, UPR_OUT, 64.0, 77.0, 57.0, 57.0 + TAB)
if belt_slot:
    top = top.cut(block(-3.5, 3.5, belt_slot[0] - 0.01, belt_slot[1], 56, 60))
add('Weapon_Top_Brace_Steel', top, 'steel_tab', TAB)
# weapon motor mount: R3 M06 profile (motor holes, drive-motor relief) at 2.3, motor face kept at x -42; sits on the
# bulkhead cross (y 50..62, z 18) and on a foot strip (y 22..50) between the two crosses; rear edge on the motor cross
m06 = r3_m06.intersect(block(-42.0 - MOUNT_T, -42.0, -200, 200, -10, 100))
m06 = m06.cut(block(-46, -40, CROSS_Y[0], CROSS_Y[1], 0, RAIL_Z1))
m06 = m06.fuse(block(-42.0 - MOUNT_T, -42.0, MOTOR_CROSS_Y[1], CROSS_Y[0], ZB, 10.5)).clean()
add('Weapon_Motor_Mount_Steel', m06, 'steel_tab', MOUNT_T)
add('Weapon_Motor_Mount_Foot', block(-RAIL_X[0], -42.0, MOTOR_CROSS_Y[1], CROSS_Y[0], *BASE_Z), 'steel_tab', TAB)
# gusset: 1.6 triangle on the cross top at the mount's front end, under the can (the can's underside is z 31.4 at y 61)
gus = cq.Workplane('XZ', origin=(0, CROSS_Y[1], 0)).polyline(                   # 'XZ' extrudes toward -y: y 60.4..62
    [(-42.0, RAIL_Z1), (-31.0, RAIL_Z1), (-42.0, 28.0)]).close().extrude(TAB).val()
add('Weapon_Motor_Mount_Gusset', gus, 'steel_tab', TAB)
# shelf on the left rail top, butting the mount's outer face: the 1.7 mm slot between the mount and the rail is too
# narrow for a stick electrode, so the mount's out-of-plane moment is taken as a couple between this shelf (welded
# from above) and the foot (welded to the rail from underneath, frame turned over before the floor goes on).
# y 35..62 on the rail and the cross tops: clear of the drive-motor face plate (y <= 34), under the weapon can (bottom z 25)
add('Weapon_Motor_Mount_Shelf', block(-RAIL_X[1] + 0.5, -42.0 - MOUNT_T, 35.0, CROSS_Y[1], RAIL_Z1, RAIL_Z1 + TAB), 'steel_tab', TAB)
# Hall post: R3 post profile as a flat 1.6 L strip at x 8..9.6, standing on the bulkhead cross; sensor glued under the arm
hall = prism([('poly', [(55.5, RAIL_Z1), (61.5, RAIL_Z1), (61.5, 53.1), (45.5, 53.1), (45.5, 51.5), (55.5, 51.5)])], 'x', 8.0, 8.0 + TAB)
add('Hall_Post_Steel', hall, 'steel_tab', TAB)

# -- wedgelets: SS400 1.6, no countersunk holes; two ears bent 90 deg down from the blank carry the M3 bolt pin.
# Ear = convex hull of a R3.4 boss about the pin (stays put while swinging: 0.6 mm from the beam face at y 98) and the
# underside from the rear edge to 10 mm down-slope; the tongue's slot is cut beside the fold.
spec = dict(config['design']['wedgelet'], hole_x_mm=[])
a = math.atan2(spec['underside_rear_yz_mm'][1] - spec['underside_tip_yz_mm'][1], spec['underside_tip_yz_mm'][0] - spec['underside_rear_yz_mm'][0])
U = np.array([math.cos(a), -math.sin(a)]); N = np.array([math.sin(a), math.cos(a)])
REAR = np.array(spec['underside_rear_yz_mm'], float)
EAR_S, EAR_R, EAR_SLOT = 10.0, 3.4, 8.0
EARS_X = ((39.1, 39.1 + TAB, +1), (84.5 - TAB, 84.5, -1))   # (x0, x1, slot direction): next to tabs 1 and 3


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
    add('Wedgelet_DIY_' + side, w, 'steel_wedgelet', TAB)
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
    """Hub + 2 O-rings (W1 part, lathe shop). Built for the R side, mirrored for L."""
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
    add('Wheel_Hub_6061_Front_' + side, hub, 'al6061_hub')
    for k, r in enumerate(rings):
        add('Wheel_ORing_P52_Front_%s%d' % (side, k + 1), r, 'NBR')
    for y in (-52.0, -8.0):
        add('Standoff_Nylon_M3_%s_%d' % (side, -y), cq.Solid.makeCylinder(3.0, 37.6 - RAIL_Z1, cq.Vector(s * 55.0, y, RAIL_Z1), cq.Vector(0, 0, 1)), 'nylon_standoff')
bl, bw, bh = BATTERY
add('3S_650mAh_LiPo_ENVELOPE', block(-bl / 2, bl / 2, -40.5 - bw / 2, -40.5 + bw / 2, RAIL_Z0 + 1, RAIL_Z0 + 1 + bh), 'battery_650')
add('Rear_Skid_UHMW_3mm', block(-20, 20, REAR_Y + 2, REAR_Y + 14, 1.0, FLOOR_Z0), 'UHMW', 3.0)
add('Nose_Skid_UHMW_5mm', block(-20, 20, 64.0, 74.0, 1.0, BASE_Z[0]), 'UHMW', 5.0)

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

steel = [b for b in new if b.material in ('steel_tube', 'steel_tab')]
SHIFT = 0.05
welds, edge_only = [], []
for i, a_ in enumerate(steel):
    for o in steel[i + 1:]:
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
welded = {w['a'] for w in welds} | {w['b'] for w in welds}
floating = [b.name for b in steel if b.name not in welded]

# rotor gap: discs/hub/spacers must clear the uprights' inner faces and everything welded
rotor = [find_body(root, n) for n in ('Beater_Disc_L', 'Beater_Disc_R', 'Beater_Hub_6061_D30')]
sweep = cq.Solid.makeCylinder(29.7 + 1.0, 53.0, cq.Vector(-26.5, 95.0, 32.0), cq.Vector(1, 0, 0))     # tooth tip r 29.7 + 1 mm
sweep_hits = [o.name for o in new if o.material.startswith('steel_') and bbox_overlap(sweep, o.shape) and sweep.intersect(o.shape).Volume() > 0.01]

# ---------------------------------------------------------------- mass and balance
root.children.remove(grp)
mp = mass_properties(root, config, {})
root.children.append(grp)
rows = []
for r in mp['parts']:
    r = dict(r)
    if r['part'] == 'Dead_Shaft_M8_12.9':
        r['mass_g'] = round(r['mass_g'] * 75 / 80, 3); r['method'] += '; M8x75 (75/80 of the x80 set)'
    rows.append(r)
steel_g = 0.0
for b in new:
    c = list(b.shape.Center().toTuple())
    if b.material in EXPLICIT_G:
        g, method = EXPLICIT_G[b.material], 'ASSUMED bought-part mass'
    else:
        g, method = b.shape.Volume() * RHO[b.material], 'CAD volume x density'
        if b.material in ('steel_tube', 'steel_tab'): steel_g += g
    rows.append({'part': b.name, 'material': b.material, 'mass_g': round(g, 3), 'centroid_mm': c, 'method': method})
rows.append({'part': 'weld beads + spatter + paint (ARC)', 'mass_g': round(WELD * steel_g, 2), 'centroid_mm': [0, 0, 12], 'method': '%.0f %% of steel, ASSUMED' % (WELD * 100)})
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
    key = ('frame tubes' if m == 'steel_tube' else 'frame + weapon sheet/flat parts' if m == 'steel_tab' else 'PC skins' if m == 'PC'
           else r['part'] if r['part'].startswith('weld beads') else 'R3 parts kept + bought')
    groups[key] = round(groups.get(key, 0) + r['mass_g'], 1)
cut_list = [{'part': b.name, 'wall_mm': WALL, 'length_mm': round(max(b.shape.BoundingBox().xlen, b.shape.BoundingBox().ylen, b.shape.BoundingBox().zlen), 1)}
            for b in new if b.material == 'steel_tube']
sheet_list = []
for b in new:
    if b.material not in ('steel_tab', 'steel_wedgelet', 'PC', 'al5052_bracket', 'UHMW'): continue
    bb = b.shape.BoundingBox()
    dims = sorted((bb.xlen, bb.ylen, bb.zlen))
    sheet_list.append({'part': b.name, 'material': b.material, 'thickness_mm': THK.get(b.name), 'blank_mm': [round(dims[2], 1), round(dims[1], 1)],
                       'mass_g': round(b.shape.Volume() * RHO.get(b.material, 0), 2)})
report = {
    'status': 'CONCEPT_CAD', 'revision': 'V4 R4-DIY (set A + NoCage, 12x12x1.2, ARC, steel weapon module)', 'orientation': 'Z-up, nose +Y, mm',
    'removed_from_r3': sorted(removed), 'invalid_bodies': invalid, 'frame_members_not_touching': floating, 'edge_only_contacts': edge_only,
    'welds': welds, 'interference': interference, 'rotor_sweep_hits': sweep_hits, 'wedgelet_swing': swing,
    'belt_slot_y': belt_slot, 'ear_yz': EAR_YZ, 'stack_drill': STACK_DRILL, 'drill_after_weld': DRILL_AFTER_WELD,
    'skirt_brackets_flat': dict(skirt_brackets),
    'mass': {'total_g': round(total, 1), 'margin_to_2000_g': round(2000 - total, 1), 'margin_to_1950_target_g': round(1950 - total, 1),
             'cg_mm': cg.round(2).tolist(), 'groups_g': groups, 'steel_g': round(steel_g, 1)},
    'support_2wd': {'drive_axle_y': FRONT_Y, 'other_support': 'nose skid y 69' if other == NOSE_SKID_Y else 'rear skid y %.0f' % REAR_SKID_Y,
                    'drive_wheel_load_pct': round(100 * wheel_frac, 1), 'push_N_mu0.6': round(push, 2)},
    'tube_cut_list_mm': cut_list, 'tube_total_mm': round(sum(c['length_mm'] for c in cut_list), 1), 'sheet_list': sheet_list,
    'bounds_mm': {k: round(v, 2) for k, v in zip(('xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'),
                  (lambda bb: (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax))(cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).BoundingBox()))},
    'rows': rows,
}
(OUT / 'build_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')

# ---------------------------------------------------------------- exports + preview
cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).exportStep(str(OUT / 'ROBOT_V4_R4_DIY.step'))
asm = cq.Assembly(name='FRAME_WELDMENT_DIY')
for b in steel:
    asm.add(cq.Workplane().add(AS_WELDED.get(b.name, b.shape)), name=b.name, color=cq.Color(0.75, 0.22, 0.17) if b.material == 'steel_tube' else cq.Color(0.9, 0.5, 0.13))
asm.save(str(OUT / 'FRAME_WELDMENT_DIY.step'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
colors = {'steel_tube': '#c0392b', 'steel_tab': '#e67e22', 'steel_wedgelet': '#8e6b3f', 'steel_bolt': '#333', 'PC': '#56b2b0', '6061': '#a6b4c5',
          'steel': '#667381', 'purchased': '#566ca3', 'al6061_hub': '#a6b4c5', 'NBR': '#2c2c2c', 'al5052_bracket': '#7f8c8d', 'battery_650': '#566ca3',
          'belt': '#343b45', 'UHMW': '#eeeeee', 'EPDM': '#222', 'nylon_standoff': '#ddd'}
fig = plt.figure(figsize=(14, 7), facecolor='#fafafa')
for index, title, skins in ((1, 'Assembly (PC skins translucent)', True), (2, 'Welded frame + internals (no skins)', False)):
    ax = fig.add_subplot(1, 2, index, projection='3d')
    for b in bodies:
        if b.material == 'keepout' or (not skins and b.material == 'PC' and b.name.startswith('Skin')): continue
        v, t = b.shape.tessellate(0.5, 0.3)
        v = np.array([p.toTuple() for p in v])
        alpha = 0.18 if b.name.startswith('Skin') else 0.95
        ax.add_collection3d(Poly3DCollection(v[np.array(t)], facecolors=colors.get(b.material, '#aaa'), edgecolors='none', alpha=alpha))
    ax.set_xlim(-110, 110); ax.set_ylim(-125, 135); ax.set_zlim(0, 80); ax.set_box_aspect((220, 260, 80)); ax.view_init(elev=32, azim=-55)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Front +Y (mm)'); ax.set_zlabel('Z (mm)'); ax.set_title(title, fontsize=12)
fig.suptitle('V4 R4-DIY — CONCEPT CAD · %.1f g · CG y %.1f' % (total, cg[1]), fontsize=15, fontweight='bold')
fig.text(0.5, 0.03, 'tube 12x12x1.2: red | sheet/flat steel: orange | wedgelet: brown | PC 2 mm: teal | lathe 6061: grey | bought: blue/black', ha='center')
fig.tight_layout(rect=(0, 0.05, 1, 0.93)); fig.savefig(OUT / 'assembly_preview.png', dpi=150); plt.close(fig)

print(json.dumps({k: report[k] for k in ('invalid_bodies', 'frame_members_not_touching', 'edge_only_contacts', 'interference', 'rotor_sweep_hits',
                                          'wedgelet_swing', 'belt_slot_y', 'mass', 'support_2wd', 'tube_total_mm', 'bounds_mm')}, ensure_ascii=False, indent=1))
