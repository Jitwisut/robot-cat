"""V4 R4-steel set A (user decision 2026-10-21): welded steel frame, 2WD front drive, 2 mm PC skins, 3S 650 mAh.
Z-up, nose +Y, mm (AGENTS.md). CONCEPT CAD: geometry for checking fit, mass and balance before any shop file.

R3 is rebuilt in memory with revision.build() and nothing in r3/ is written. From it this script
  * removes the TPU tub + lid, both rear drive motors and every printed part (wheels, platform, standoffs,
    battery tray, wedgelet blocks, weapon mount feet, switch collar)
  * keeps every other R3 part where it is; the wedgelet carriers and hall post become machined 6061 (same shape,
    hall post trimmed to stand on the bulkhead cross) and M06 gets a notch over the bulkhead cross (M06 is HOLD anyway)
  * adds the welded frame (10x10x1.0 mild-steel square tube + 1.5 mm tabs), 2 mm PC skins, bought wheel / battery /
    standoff envelopes and a rear UHMW skid
  * checks every new body against every other body for volume interference, then mass, CG and 2WD support loads
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r4_steel/build_r4_steel.py
Writes robot_v4/print_revision/r4_steel/output/.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PR = HERE.parent                                        # robot_v4/print_revision
sys.path.insert(0, str(PR))

import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from cad_backend import Body, part, group, block, bore, find_body
from revision import build
from checks import mass_properties

OUT = HERE / 'output'
OUT.mkdir(exist_ok=True)
config = json.loads((PR / 'r3' / 'inputs_R3.json').read_text())
FRONT_Y = config['design']['front_axle_y_mm']           # 18: the only driven axle in 2WD

# ---------------------------------------------------------------- parameters (mm, g/mm^3)
TUBE, WALL = 10.0, 1.0      # square tube side / wall: MIG minimum, confirm with the welding shop
TAB = 1.5                   # laser-cut mild-steel tab thickness
SKIN = 2.0                  # PC skin thickness
FLOOR_Z0 = 4.0              # floor z 4..6, same underside as R3: ground clearance unchanged
RAIL_Z0 = FLOOR_Z0 + SKIN   # tubes stand on the floor: z 6..16
RAIL_Z1 = RAIL_Z0 + TUBE
LID_Z0 = 61.0               # lid z 61..63: 1 mm over the weapon motor can (top z 60), under the inverted-wheel plane z 64
WALL_X = 83.0               # outer face of the side walls (R3 PC body)
REAR_Y = -110.0             # outer face of the rear wall (R3)
CROSS_Y = (52.0, 62.0)      # bulkhead cross: 2 mm ahead of the front tyre (y 50), behind the uprights (y 64)
RAIL_X = (48.0, 58.0)       # |x|: inboard of the wheels (59.5), under the drive motors (z 19.5)
RAIL_WALL = 1.5             # rails run unbroken from the rear cross to the hinge beam and carry the weapon-hit moment;
                            # wall from calc/frame_strength_calc.py (thinnest stock wall with SF >= 1.5 fore-aft)
BEAM_Y = (88.0, 98.0)       # hinge beam behind the wedgelet hinge pins (y 100.5)
HINGE_TAB_X = ((37.0, 38.5), (62.0, 63.5), (85.0, 86.5))   # |x| gaps between the wedgelet carriers on the pin
# Bought wheel envelope, D64 keeps the ground at z 0. ORTECH JB361 (main's R3 drive motor): gearbox face at |x| 56,
# boss D7 x 2.5, shaft D4 x 11.5 with the D flat on the outer 8 mm -> the hub's inner face at |x| 59.5 gives 8 mm on the flat
WHEEL_R, WHEEL_X = 32.0, (59.5, 75.5)
MOTOR_HOLES_YZ = [(-8.5, 0.0), (8.5, 0.0)]                # JB361 face: 2 x M3 at +-8.5 about the shaft (catalogue R3-10)
GEARBOX_SCREWS_YZ = [(0.0, -8.5), (0.0, 8.5)]             # the two gearbox screw heads, 90 deg to the M3 holes (R3-10 drawing)
BATTERY = (57.0, 31.0, 24.0)                              # 3S 650 mAh envelope, ASSUMED until the pack is chosen
RHO = {'steel_tube': 0.00785, 'steel_tab': 0.00785, 'PC': 0.00120, 'UHMW': 0.00094, 'EPDM': 0.00130, 'al5052_bracket': 0.00268}
EXPLICIT_G = {'wheel_bought': 45.0, 'battery_650': 58.0, 'nylon_standoff': 0.6}   # ASSUMED, see calc/steel_r4_calc.py
RIVNUTS = 24                # M3 steel rivnut 0.9 g + screw 0.5 g
SWITCH_MOUNT_G = 3.0        # bought panel mount replacing the PETG collar
WELD = 0.04                 # weld beads + paint, fraction of steel

# ---------------------------------------------------------------- R3 in memory, minus what set A removes
root, _ = build(config)
REMOVE = ('Tub_TPU', 'Lid_TPU_2mm', 'JGA25_370_400rpm_Rear_', 'Wheel_', 'Battery_Tray_PETG', 'Electronics_Platform_PETG',
          'Electronics_Standoff_PETG', 'Wedgelet_Block_PETG', 'Weapon_Mount_Foot_PETG', 'Disconnect_Collar_PETG',
          '3S_850mAh_LiPo_ENVELOPE')
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
for side in ('L', 'R'):
    for n in ('1', '2'):
        find_body(root, 'Wedgelet_Carrier_PETG_%s%s' % (side, n)).material = '6061'
hall = find_body(root, 'Hall_Post_PETG')
hall.material = '6061'
hall.shape = hall.shape.cut(block(-100, 100, -200, 200, -10, RAIL_Z1)).clean()     # stands on the cross
m06 = find_body(root, 'Weapon_Motor_Mount_3mm')
m06.shape = m06.shape.cut(block(-46, -41, CROSS_Y[0] - 1, CROSS_Y[1] + 1, 0, RAIL_Z1 + 1)).clean()
for b in (hall, m06):
    if len(b.shape.Solids()) != 1:
        raise RuntimeError(b.name + ' split into several solids')

# ---------------------------------------------------------------- new geometry
grp = group(root, '01_R4_Steel')
new = []


def add(name, shape, material):
    comp = part(grp, name)
    comp.body = Body(shape.clean(), name, material, comp)
    new.append(comp.body)
    return comp.body


def tube_x(name, x0, x1, y0, z0):
    s = block(x0, x1, y0, y0 + TUBE, z0, z0 + TUBE).cut(block(x0 - 1, x1 + 1, y0 + WALL, y0 + TUBE - WALL, z0 + WALL, z0 + TUBE - WALL))
    return add(name, s, 'steel_tube')


TUBE_WALL = {}


def tube_y(name, x0, y0, y1, z0, wall=WALL):
    s = block(x0, x0 + TUBE, y0, y1, z0, z0 + TUBE).cut(block(x0 + wall, x0 + TUBE - wall, y0 - 1, y1 + 1, z0 + wall, z0 + TUBE - wall))
    TUBE_WALL[name] = wall
    return add(name, s, 'steel_tube')


def tube_z(name, x0, y0, z0, z1):
    s = block(x0, x0 + TUBE, y0, y0 + TUBE, z0, z1).cut(block(x0 + WALL, x0 + TUBE - WALL, y0 + WALL, y0 + TUBE - WALL, z0 - 1, z1 + 1))
    return add(name, s, 'steel_tube')


def hole_x(shape, y, z, r):
    return shape.cut(cq.Solid.makeCylinder(r, 400, cq.Vector(-200, y, z), cq.Vector(1, 0, 0)))


REAR_CROSS_Y = REAR_Y + SKIN                           # -108..-98
tube_x('Frame_Rear_Cross', -(WALL_X - SKIN), WALL_X - SKIN, REAR_CROSS_Y, RAIL_Z0)
# bulkhead cross in three pieces butting the unbroken rails: centre between the rails, outer stubs to the walls
tube_x('Frame_Bulkhead_Cross', -RAIL_X[0], RAIL_X[0], CROSS_Y[0], RAIL_Z0)
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (RAIL_X[1], WALL_X - SKIN) if s > 0 else (-(WALL_X - SKIN), -RAIL_X[1])
    tube_x('Frame_Bulkhead_Stub_' + side, x0, x1, CROSS_Y[0], RAIL_Z0)
    xr = RAIL_X[0] if s > 0 else -RAIL_X[1]
    tube_y('Frame_Rail_' + side, xr, REAR_CROSS_Y + TUBE, BEAM_Y[0], RAIL_Z0, wall=RAIL_WALL)
    x_in, x_out = (HINGE_TAB_X[0][0], 88.0) if s > 0 else (-88.0, -HINGE_TAB_X[0][0])
    tube_x('Frame_Hinge_Beam_' + side, x_in, x_out, BEAM_Y[0], RAIL_Z0)
    xp = WALL_X - SKIN - TUBE if s > 0 else -(WALL_X - SKIN)
    # rear posts only: the front posts and top stubs carried no hit or drop load and their 34 g pays for the 1.5 mm
    # rails; the lid's front corners sit on bought aluminium angles screwed to the PC walls
    tube_z('Frame_Post_Rear_' + side, xp, REAR_CROSS_Y, RAIL_Z1, LID_Z0 - TUBE)
tube_x('Frame_Top_Rear_Cross', -(WALL_X - SKIN), WALL_X - SKIN, REAR_CROSS_Y, LID_Z0 - TUBE)   # rests on both rear posts

# Tabs (1.5 mm mild steel, welded)
upr = find_body(root, 'Upright_6061_6mm_R')
taps = []
for f in upr.shape.Faces():
    if f.geomType() != 'CYLINDER': continue
    cyl = BRepAdaptor_Surface(f.wrapped).Cylinder()
    if abs(abs(cyl.Axis().Direction().X()) - 1) < 1e-6 and abs(cyl.Radius() - 1.25) < 0.01:
        p = (round(cyl.Location().Y(), 3), round(cyl.Location().Z(), 3))
        if p not in taps: taps.append(p)
taps.sort()
for side, s in (('L', -1), ('R', 1)):
    # drive motor face plate: gearbox face at |x| 56, wheel hub from 62; D8.5 boss hole on the axle
    x0 = 56.0 if s > 0 else -56.0 - TAB
    t = block(x0, x0 + TAB, FRONT_Y - 16, FRONT_Y + 16, RAIL_Z1, RAIL_Z1 + 32)
    t = hole_x(t, FRONT_Y, 32.0, 4.25)                    # D8.5 over the D7 boss
    for dy, dz in MOTOR_HOLES_YZ:
        t = hole_x(t, FRONT_Y + dy, 32.0 + dz, 1.7)      # M3 clearance D3.4
    for dy, dz in GEARBOX_SCREWS_YZ:
        t = hole_x(t, FRONT_Y + dy, 32.0 + dz, 2.75)     # D5.5 clearance for the gearbox screw heads
    add('Tab_Drive_Motor_' + side, t, 'steel_tab')
    # upright web on the upright outer face (|x| 33..34.5), M3 clearance holes on the 4 R3 upright taps; starts at
    # y 66 (weapon motor can reaches y 65), notched around the axle washer (y 95, z 32). It stands on a floor flange
    # that runs to the rail's inner face (|x| 48) and is tied to the rail by two vertical rungs (y 68.5, 81), so the
    # hit moment goes into the rail as a couple instead of bending a thin plate.
    x0 = 33.0 if s > 0 else -33.0 - TAB
    t = block(x0, x0 + TAB, 66.0, 96.0, RAIL_Z0 + TAB, 55.0)
    t = t.cut(cq.Solid.makeCylinder(9.0, 4, cq.Vector(x0 - 1, 95.0, 32.0), cq.Vector(1, 0, 0)))
    for y, z in taps:
        t = hole_x(t, y, z, 1.7)
    add('Tab_Upright_Web_' + side, t, 'steel_tab')
    fx0, fx1 = (33.0, RAIL_X[0]) if s > 0 else (-RAIL_X[0], -33.0)
    add('Tab_Upright_Flange_' + side, block(fx0, fx1, CROSS_Y[1], BEAM_Y[0], RAIL_Z0, RAIL_Z0 + TAB), 'steel_tab')
    rx0, rx1 = (33.0 + TAB, RAIL_X[0]) if s > 0 else (-RAIL_X[0], -33.0 - TAB)
    for k, yr in enumerate((68.5, 81.0)):          # clear of the M3 heads at y 75 / 90 and the axle nut (y >= 87.5)
        add('Tab_Upright_Rung%d_%s' % (k + 1, side), block(rx0, rx1, yr, yr + TAB, RAIL_Z0 + TAB, 28.0), 'steel_tab')
    # wedgelet hinge tabs on the beam front, D3.2 for the D3 pin at (y 102, z 14.5)
    for i, (a, b) in enumerate(HINGE_TAB_X):
        x0, x1 = (a, b) if s > 0 else (-b, -a)
        # inside the R3 wedgelet block envelope (y <= 105.2, z <= 17.7); top-front corner follows the wedgelet's
        # 35.5 deg underside with 0.3 mm clearance
        t = block(x0, x1, BEAM_Y[1], 105.0, 8.0, 17.7)
        t = t.cut(find_body(root, 'Wedgelet_' + side).shape.translate(cq.Vector(0, 0, -0.3)))
        add('Tab_Hinge_%s%d' % (side, i + 1), hole_x(t, 102.0, 14.5, 1.6), 'steel_tab')
    # battery strap tabs on the rail inner faces
    for y in (-50.0, -31.0):
        x0 = RAIL_X[0] - TAB if s > 0 else -RAIL_X[0]
        add('Tab_Battery_Strap_%s_%d' % (side, -y), block(x0, x0 + TAB, y - 4, y + 4, RAIL_Z0, RAIL_Z1), 'steel_tab')
# weapon motor mount (M06, x -45..-42): two L tabs from the left rail inner face, under the left drive motor (z 19.5)
for i, (y0, ztop) in enumerate(((24.0, 19.0), (40.0, 30.0))):
    base = block(-RAIL_X[0], -45.0, y0, y0 + 10, RAIL_Z0, RAIL_Z0 + TAB)
    leg = block(-45.0 - TAB, -45.0, y0, y0 + 10, RAIL_Z0 + TAB, ztop)
    add('Tab_Weapon_Mount_%d' % (i + 1), base.fuse(leg), 'steel_tab')
# drive motor cross between the rails under the motors' inner ends (a 1.5 mm strap fails in the 250 g drop:
# calc/frame_strength_calc.py), EPDM pads up to the motor underside (z 19.5)
tube_x('Frame_Motor_Cross', -RAIL_X[0], RAIL_X[0], 10.0, RAIL_Z0)
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (6.2, 14.0) if s > 0 else (-14.0, -6.2)
    add('Pad_Drive_Motor_EPDM_' + side, block(x0, x1, 11.0, 19.0, RAIL_Z1, 19.5), 'EPDM')
# clamp over the inner motor ends (R3's lid bar): a centre plate between the motor ends (|x| < 6.2) up from the strap,
# a top bar across both motors and EPDM pads, so an inverted drop does not hang the motors off their face plates
add('Clamp_Drive_Motor_Post', block(-TAB / 2, TAB / 2, 10.0, 20.0, RAIL_Z1, 45.0), 'steel_tab')
tube_x('Frame_Clamp_Bar', -14.0, 14.0, 10.0, 45.0)
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (6.2, 14.0) if s > 0 else (-14.0, -6.2)
    add('Pad_Drive_Motor_Top_EPDM_' + side, block(x0, x1, 11.0, 19.0, 44.5, 45.0), 'EPDM')

# Skirt brackets: 1.5 mm 5052, laser cut + one bend, bolted (not welded). The released R3 skirts keep their M2 holes
# (side y -82.5/-27.5/32.5/62.5, rear x -57.5/0/57.5). Each bracket = a wall leg on the PC wall's outer face (one M3)
# + a seat leg in the skirt's inner-face plane with a D2.4 hole for the skirt's M2 screw. Side brackets bolt through
# the PC wall (nut inside); rear brackets bolt through the rear wall into a rivnut in the rear cross.
SKIRT_N = (-0.819, -0.574)          # skirt inner-face normal in (u outward, z), from the R3 skirt CAD
SKIRT_FACE = {'side': (96.83, 8.26), 'rear': (118.83, 8.27)}      # a point on each skirt's inner face, (u, z)
SKIRT_HOLE = {'side': (94.43, 15.17), 'rear': (116.43, 15.17)}    # skirt M2 hole axis point, (u, z)
WALL_OUT = {'side': WALL_X, 'rear': -REAR_Y}
BRACKET_W = 15.0
skirt_brackets = []


def skirt_bracket(kind, sgn, station, bolt_dt=0.0, bolt_z=16.0):
    """kind 'side' (u = sgn*x, t = y) or 'rear' (u = -y, t = x). Returns the world solid and its flat data."""
    n = np.array(SKIRT_N); n /= np.linalg.norm(n)
    face = np.array(SKIRT_FACE[kind]); hole = np.array(SKIRT_HOLE[kind])
    u_leg = WALL_OUT[kind] + TAB
    z_top = face[1] + n[0] * (u_leg - face[0]) / -n[1]            # seat plane at the wall leg's outer face
    p0 = hole - np.dot(hole - face, n) * n                         # skirt hole on the inner face
    s_dir = np.array([n[1], -n[0]]) * (-1 if n[1] * 1 < 0 else 1)  # in-plane, pointing outward/down
    if s_dir[0] < 0: s_dir = -s_dir
    a = np.array([u_leg, z_top]) + n * 0.1; b = p0 + s_dir * 4.0 + n * 0.1      # 0.1 mm off the skirt (rounded normal)
    z_low = min(bolt_z - 4.0, z_top - 8.0)
    prof_leg = [(WALL_OUT[kind], z_low), (u_leg, z_low), (u_leg, z_top), (WALL_OUT[kind], z_top)]
    prof_seat = [tuple(a), tuple(b), tuple(b + n * TAB), tuple(a + n * TAB)]

    def solid(prof, t0, t1):   # (u, z) polygon extruded along t
        wp = cq.Workplane('XZ').polyline([(u, z) for u, z in prof]).close().extrude(-(t1 - t0))
        return wp.val().translate(cq.Vector(0, t0, 0))
    t0, t1 = station - BRACKET_W / 2, station + BRACKET_W / 2
    sh = solid(prof_leg, t0, t1).fuse(solid(prof_seat, t0, t1)).clean()
    # holes in local (u, t, z): M3 through the wall leg along u, D2.4 through the seat along n
    sh = sh.cut(cq.Solid.makeCylinder(1.7, 10, cq.Vector(WALL_OUT[kind] - 2, station + bolt_dt, bolt_z), cq.Vector(1, 0, 0)))
    sh = sh.cut(cq.Solid.makeCylinder(1.2, 10, cq.Vector(p0[0] - 5 * n[0], station, p0[1] - 5 * n[1]), cq.Vector(n[0], 0, n[1])))
    # local frame: x = u, y = t, z = z  ->  world
    if kind == 'side':
        world = sh if sgn > 0 else sh.mirror('YZ')
    else:
        world = sh.rotate(cq.Vector(), cq.Vector(0, 0, 1), -90)    # (u, t) -> (t, -u)
    leg_len = z_top - z_low
    seat_len = float(np.linalg.norm(b - a))
    flat = {'width': BRACKET_W, 'wall_leg': round(leg_len, 2), 'seat_leg': round(seat_len, 2),
            'bend_deg_from_flat': round(float(np.degrees(np.arctan2(s_dir[0], -s_dir[1]))), 1),
            'm3_hole_from_leg_end': round(bolt_z - z_low, 2), 'm3_hole_offset_from_centre': bolt_dt,
            'm2_hole_from_bend': round(float(np.linalg.norm(p0 - a)), 2)}
    return world, flat


for side, s in (('L', -1), ('R', 1)):
    for y in (-82.5, -27.5, 32.5, 62.5):
        dt = 0.0
        sh, flat = skirt_bracket('side', s, y, bolt_dt=dt)
        b = add('Skirt_Bracket_%s_y%+.0f' % (side, y), sh, 'al5052_bracket')
        skirt_brackets.append((b.name, flat))
for x in (-57.5, 0.0, 57.5):
    sh, flat = skirt_bracket('rear', 1, x, bolt_z=11.0)                # M3 into a rivnut in the rear cross (z 6..16)
    b = add('Skirt_Bracket_Rear_x%+.0f' % x, sh, 'al5052_bracket')
    skirt_brackets.append((b.name, flat))

# PC 2 mm skins
floor = block(-WALL_X, WALL_X, REAR_Y + SKIN, CROSS_Y[1], FLOOR_Z0, RAIL_Z0)
for s in (-1, 1):
    x0, x1 = (34.0, WALL_X) if s > 0 else (-WALL_X, -34.0)
    floor = floor.fuse(block(x0, x1, CROSS_Y[1] - 1, BEAM_Y[1], FLOOR_Z0, RAIL_Z0))
    floor = floor.cut(block(*((WHEEL_X[0] - 1, WHEEL_X[1] + 1) if s > 0 else (-WHEEL_X[1] - 1, -WHEEL_X[0] + 1)), FRONT_Y - 21, FRONT_Y + 21, 0, 10))
add('Skin_PC2_Floor', floor, 'PC')
add('Skin_PC2_Rear_Wall', block(-WALL_X, WALL_X, REAR_Y, REAR_Y + SKIN, FLOOR_Z0, LID_Z0), 'PC')
for side, s in (('L', -1), ('R', 1)):
    x0 = WALL_X - SKIN if s > 0 else -WALL_X
    add('Skin_PC2_Side_Wall_' + side, block(x0, x0 + SKIN, REAR_Y + SKIN, BEAM_Y[0], RAIL_Z0, LID_Z0), 'PC')
lid = block(-WALL_X, WALL_X, REAR_Y, CROSS_Y[1], LID_Z0, LID_Z0 + SKIN)
for s in (-1, 1):
    lid = lid.cut(block(*((WHEEL_X[0] - 1, WHEEL_X[1] + 1) if s > 0 else (-WHEEL_X[1] - 1, -WHEEL_X[0] + 1)), FRONT_Y - 23, FRONT_Y + 23, 0, 80))
add('Skin_PC2_Lid', lid, 'PC')
add('Electronics_Plate_PC2', block(-58, 58, -56, -1.5, 37.6, 39.6), 'PC')      # IMU foam sits on it (z 39.6); |x| 58 clears the wheels (59.5)

# bought / other parts
for side, s in (('L', -1), ('R', 1)):
    x0 = WHEEL_X[0] if s > 0 else -WHEEL_X[1]
    add('Wheel_Bought_Front_' + side, cq.Solid.makeCylinder(WHEEL_R, WHEEL_X[1] - WHEEL_X[0], cq.Vector(x0, FRONT_Y, 32), cq.Vector(1, 0, 0)), 'wheel_bought')
    for y in (-52.0, -8.0):
        add('Standoff_Nylon_M3_%s_%d' % (side, -y), cq.Solid.makeCylinder(3.0, 37.6 - RAIL_Z1, cq.Vector(s * 55.0, y, RAIL_Z1), cq.Vector(0, 0, 1)), 'nylon_standoff')
bl, bw, bh = BATTERY
add('3S_650mAh_LiPo_ENVELOPE', block(-bl / 2, bl / 2, -40.5 - bw / 2, -40.5 + bw / 2, RAIL_Z0 + 1, RAIL_Z0 + 1 + bh), 'battery_650')
add('Rear_Skid_UHMW_3mm', block(-20, 20, REAR_Y + 2, REAR_Y + 14, 1.0, FLOOR_Z0), 'UHMW')

# ---------------------------------------------------------------- checks
def bbox_overlap(a, b, tol=0.01):
    A, B = a.BoundingBox(), b.BoundingBox()
    return A.xmin < B.xmax - tol and B.xmin < A.xmax - tol and A.ymin < B.ymax - tol and B.ymin < A.ymax - tol and A.zmin < B.zmax - tol and B.zmin < A.zmax - tol


bodies = root.bodies()
interference = []
for i, a in enumerate(new + [hall, m06]):
    for o in bodies:
        if o is a or (o in new and new.index(o) <= i and a in new): continue
        if not bbox_overlap(a.shape, o.shape): continue
        v = a.shape.intersect(o.shape).Volume()
        if v > 0.01:
            interference.append({'a': a.name, 'b': o.name, 'volume_mm3': round(v, 3)})
invalid = [b.name for b in new if not b.shape.isValid() or len(b.shape.Solids()) != 1]

# Wedgelet swing: rotate each wedgelet + its two carriers about the hinge pin axis (y 102, z 14.5) and find the free
# range before anything (frame, tabs, skirt brackets, floor plane z 0 excluded) is hit. MuJoCo uses -2..+14 deg.
def rotated(shape, deg):
    return shape.rotate(cq.Vector(0, 102.0, 14.5), cq.Vector(1, 102.0, 14.5), deg)


swing = {}
for side in ('L', 'R'):
    moving = [find_body(root, n) for n in ('Wedgelet_' + side, 'Wedgelet_Carrier_PETG_%s1' % side, 'Wedgelet_Carrier_PETG_%s2' % side)]
    others = [o for o in bodies if o not in moving and not o.name.startswith(('Hinge_Pin', 'Eclip')) and o.material != 'keepout']
    res = {}
    for direction, steps in (('up', range(1, 61)), ('down', range(-1, -21, -1))):
        free = 0
        for deg in steps:
            hit = None
            for m_ in moving:
                r = rotated(m_.shape, deg)
                for o in others:
                    if bbox_overlap(r, o.shape) and r.intersect(o.shape).Volume() > 0.05:
                        hit = o.name; break
                if hit: break
            if hit:
                res[direction] = {'free_deg': free, 'first_contact': hit, 'at_deg': deg}
                break
            free = deg
        else:
            res[direction] = {'free_deg': free, 'first_contact': None}
    res['sim_range_deg'] = [-2.0, 14.0]
    res['ok'] = res['up']['free_deg'] >= 14 and abs(res['down']['free_deg']) >= 2
    swing[side] = res

# every tube and tab must touch another frame member (a weld needs contact); straps/tabs touching only skins fail
# Weld list: every pair of frame members that share a face. Contact area = common volume after a 0.05 mm shift
# toward the other member / 0.05. A pair that only touches along an edge (area < 5 mm^2) cannot be welded properly.
steel = [b for b in new if b.material.startswith('steel')]
SHIFT = 0.05
welds, edge_only = [], []
for i, a in enumerate(steel):
    for o in steel[i + 1:]:
        if a.shape.distance(o.shape) > 0.01: continue
        best = (0.0, None)
        for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            c = a.shape.intersect(o.shape.translate(cq.Vector(*d) * SHIFT))
            v = c.Volume()
            if v > best[0]: best = (v, c)
        area = best[0] / SHIFT
        if area < 5.0:
            edge_only.append({'a': a.name, 'b': o.name, 'area_mm2': round(area, 2)})
            continue
        bb = best[1].BoundingBox()
        welds.append({'a': a.name, 'b': o.name, 'contact_area_mm2': round(area, 1),
                      'contact_centre_mm': [round(v, 1) for v in best[1].Center().toTuple()],
                      'contact_size_mm': sorted((round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)), reverse=True)[:2]})
welded = {w['a'] for w in welds} | {w['b'] for w in welds}
floating = [b.name for b in steel if b.name not in welded]

# upright screws: M3 button head D5.7 x 1.65 outside the tab must clear the fork arm
screw_hits = []
for side, s in (('L', -1), ('R', 1)):
    for y, z in taps:
        x = s * (33.0 + TAB)
        head = cq.Solid.makeCylinder(2.85, 1.65, cq.Vector(x, y, z), cq.Vector(s, 0, 0))
        for o in bodies:
            if bbox_overlap(head, o.shape, 0) and head.intersect(o.shape).Volume() > 0.01:
                screw_hits.append({'tap_yz': [y, z], 'side': side, 'body': o.name})

# ---------------------------------------------------------------- mass and balance
root.children.remove(grp)
mp = mass_properties(root, config, {})
root.children.append(grp)
rows = [dict(r) for r in mp['parts']]
steel_g = 0.0
for b in new:
    c = list(b.shape.Center().toTuple())
    if b.material in EXPLICIT_G:
        g, method = EXPLICIT_G[b.material], 'ASSUMED bought-part mass'
    else:
        g, method = b.shape.Volume() * RHO[b.material], 'CAD volume x density'
        if b.material.startswith('steel'): steel_g += g
    rows.append({'part': b.name, 'material': b.material, 'mass_g': round(g, 3), 'centroid_mm': c, 'method': method})
rows.append({'part': 'weld beads + paint', 'mass_g': round(WELD * steel_g, 2), 'centroid_mm': [0, -6.5, 12], 'method': '%.0f %% of steel' % (WELD * 100)})
rows.append({'part': 'rivnuts M3 + screws x%d' % RIVNUTS, 'mass_g': RIVNUTS * 1.4, 'centroid_mm': [0, -20, 33], 'method': 'ASSUMED 0.9 + 0.5 g'})
rows.append({'part': 'switch panel mount (bought)', 'mass_g': SWITCH_MOUNT_G, 'centroid_mm': [80, -30, 25], 'method': 'ASSUMED'})
rows.append({'part': 'lid front corner angles, bought Al x2', 'mass_g': 3.0, 'centroid_mm': [0, 57, 58], 'method': 'ASSUMED'})
total = sum(r['mass_g'] for r in rows)
cg = sum(np.array(r['centroid_mm']) * r['mass_g'] for r in rows) / total
NOSE_SKID_Y, REAR_SKID_Y = 69.0, REAR_Y + 8
other = NOSE_SKID_Y if cg[1] > FRONT_Y else REAR_SKID_Y
wheel_frac = (cg[1] - other) / (FRONT_Y - other)
F_MOTOR, MU = 3.99144375, 0.6                          # r3_calc 12.6 V motor limit per wheel, tyre mu ASSUMED
w = total / 1000 * 9.81
push = 2 * min(F_MOTOR, MU * w * wheel_frac / 2) - 0.2 * w * (1 - wheel_frac)

groups = {}
for r in rows:
    key = ('frame tubes' if r.get('material') == 'steel_tube' else 'frame tabs/strap' if r.get('material') == 'steel_tab'
           else 'PC skins' if r.get('material') == 'PC' else r['part'] if r['part'] in ('weld beads + paint',) else 'R3 parts kept + bought')
    groups[key] = round(groups.get(key, 0) + r['mass_g'], 1)
cut_list = [{'part': b.name, 'wall_mm': TUBE_WALL.get(b.name, WALL), 'length_mm': round(max(b.shape.BoundingBox().xlen, b.shape.BoundingBox().ylen, b.shape.BoundingBox().zlen), 1)}
            for b in new if b.material == 'steel_tube']
report = {
    'status': 'CONCEPT_CAD', 'revision': 'V4 R4-steel set A (2WD + PC 2 mm + 3S 650)', 'orientation': 'Z-up, nose +Y, mm',
    'removed_from_r3': sorted(removed), 'invalid_bodies': invalid, 'frame_members_not_touching': floating, 'edge_only_contacts': edge_only, 'welds': welds, 'interference': interference, 'upright_screw_head_hits': screw_hits,
    'upright_taps_yz': taps, 'skirt_brackets_flat': dict(skirt_brackets), 'wedgelet_swing': swing,
    'mass': {'total_g': round(total, 1), 'margin_to_2000_g': round(2000 - total, 1), 'margin_to_1950_target_g': round(1950 - total, 1),
             'cg_mm': cg.round(2).tolist(), 'groups_g': groups},
    'support_2wd': {'drive_axle_y': FRONT_Y, 'other_support': 'nose skid y 69' if other == NOSE_SKID_Y else 'rear skid y %.0f' % REAR_SKID_Y,
                    'drive_wheel_load_pct': round(100 * wheel_frac, 1), 'push_N_mu0.6': round(push, 2)},
    'tube_cut_list_mm': cut_list, 'tube_total_mm': round(sum(c['length_mm'] for c in cut_list), 1),
    'bounds_mm': {k: round(v, 2) for k, v in zip(('xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'),
                  (lambda bb: (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax))(cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).BoundingBox()))},
    'rows': rows,
}
(OUT / 'build_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')

# ---------------------------------------------------------------- exports + preview
cq.Compound.makeCompound([b.shape for b in bodies if b.material != 'keepout']).exportStep(str(OUT / 'ROBOT_V4_R4_STEEL.step'))
cq.Compound.makeCompound([b.shape for b in new if b.material.startswith('steel')]).exportStep(str(OUT / 'FRAME_WELDMENT.step'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
colors = {'steel_tube': '#c0392b', 'steel_tab': '#e67e22', 'PC': '#56b2b0', '6061': '#a6b4c5', 'steel': '#667381', 'purchased': '#566ca3',
          'wheel_bought': '#2c2c2c', 'al5052_bracket': '#7f8c8d', 'battery_650': '#566ca3', 'belt': '#343b45', 'UHMW': '#eeeeee', 'EPDM': '#222', 'nylon_standoff': '#ddd'}
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
fig.suptitle('V4 R4-steel set A — CONCEPT CAD · %.1f g · CG y %.1f' % (total, cg[1]), fontsize=15, fontweight='bold')
fig.text(0.5, 0.03, 'steel tube: red | steel tabs: orange | PC 2 mm: teal | 6061: grey | bought: blue/black', ha='center')
fig.tight_layout(rect=(0, 0.05, 1, 0.93)); fig.savefig(OUT / 'assembly_preview.png', dpi=150); plt.close(fig)

print(json.dumps({k: report[k] for k in ('invalid_bodies', 'frame_members_not_touching', 'edge_only_contacts', 'wedgelet_swing', 'interference', 'upright_screw_head_hits', 'mass', 'support_2wd', 'tube_total_mm', 'bounds_mm')},
                 ensure_ascii=False, indent=1))
