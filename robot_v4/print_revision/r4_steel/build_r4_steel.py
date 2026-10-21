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
RAIL_X = (50.0, 60.0)       # |x|: inboard of the wheels (62), under the drive motors (z 19.5)
ARM_X = (37.0, 47.0)        # |x|: fork arm 2.5 mm outside the upright tab: room for the M3 heads on the z 14 taps
BEAM_Y = (88.0, 98.0)       # hinge beam behind the wedgelet hinge pins (y 100.5)
HINGE_TAB_X = ((37.0, 38.5), (62.0, 63.5), (85.0, 86.5))   # |x| gaps between the wedgelet carriers on the pin
WHEEL_R, WHEEL_X = 32.0, (62.0, 78.0)                     # bought wheel envelope = R3 wheel (keeps ground at z 0)
BATTERY = (57.0, 31.0, 24.0)                              # 3S 650 mAh envelope, ASSUMED until the pack is chosen
RHO = {'steel_tube': 0.00785, 'steel_tab': 0.00785, 'PC': 0.00120, 'UHMW': 0.00094, 'EPDM': 0.00130}
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


def tube_y(name, x0, y0, y1, z0):
    s = block(x0, x0 + TUBE, y0, y1, z0, z0 + TUBE).cut(block(x0 + WALL, x0 + TUBE - WALL, y0 - 1, y1 + 1, z0 + WALL, z0 + TUBE - WALL))
    return add(name, s, 'steel_tube')


def tube_z(name, x0, y0, z0, z1):
    s = block(x0, x0 + TUBE, y0, y0 + TUBE, z0, z1).cut(block(x0 + WALL, x0 + TUBE - WALL, y0 + WALL, y0 + TUBE - WALL, z0 - 1, z1 + 1))
    return add(name, s, 'steel_tube')


def hole_x(shape, y, z, r):
    return shape.cut(cq.Solid.makeCylinder(r, 400, cq.Vector(-200, y, z), cq.Vector(1, 0, 0)))


REAR_CROSS_Y = REAR_Y + SKIN                           # -108..-98
tube_x('Frame_Rear_Cross', -(WALL_X - SKIN), WALL_X - SKIN, REAR_CROSS_Y, RAIL_Z0)
tube_x('Frame_Bulkhead_Cross', -(WALL_X - SKIN), WALL_X - SKIN, CROSS_Y[0], RAIL_Z0)
for side, s in (('L', -1), ('R', 1)):
    xr = RAIL_X[0] if s > 0 else -RAIL_X[1]
    tube_y('Frame_Rail_' + side, xr, REAR_CROSS_Y + TUBE, CROSS_Y[0], RAIL_Z0)
    xa = ARM_X[0] if s > 0 else -ARM_X[1]
    tube_y('Frame_Fork_Arm_' + side, xa, CROSS_Y[1], BEAM_Y[0], RAIL_Z0)
    x_in, x_out = (ARM_X[0], 88.0) if s > 0 else (-88.0, -ARM_X[0])
    tube_x('Frame_Hinge_Beam_' + side, x_in, x_out, BEAM_Y[0], RAIL_Z0)
    xp = WALL_X - SKIN - TUBE if s > 0 else -(WALL_X - SKIN)
    for loc, y0 in (('Rear', REAR_CROSS_Y), ('Front', CROSS_Y[0])):
        tube_z('Frame_Post_%s_%s' % (loc, side), xp, y0, RAIL_Z1, LID_Z0 - TUBE)
    # top front stub: post to |x| 50, clear of the weapon motor can (x -41..13.5) and the belt
    x0, x1 = (50.0, WALL_X - SKIN - TUBE) if s > 0 else (-(WALL_X - SKIN - TUBE), -50.0)
    tube_x('Frame_Top_Front_Stub_' + side, x0, x1, CROSS_Y[0], LID_Z0 - TUBE)
tube_x('Frame_Top_Rear_Cross', -(WALL_X - SKIN - TUBE), WALL_X - SKIN - TUBE, REAR_CROSS_Y, LID_Z0 - TUBE)

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
    add('Tab_Drive_Motor_' + side, hole_x(t, FRONT_Y, 32.0, 4.25), 'steel_tab')
    # upright plate on the upright outer face (|x| 33..34.5), M3 clearance holes on the 4 R3 upright taps; starts at
    # y 66 (weapon motor can reaches y 65), notched around the axle washer (y 95, z 32); its bottom flange runs out to
    # the fork arm and back to the bulkhead cross, under the screw heads
    x0 = 33.0 if s > 0 else -33.0 - TAB
    t = block(x0, x0 + TAB, 66.0, 96.0, RAIL_Z0, 55.0)
    t = t.cut(cq.Solid.makeCylinder(9.0, 4, cq.Vector(x0 - 1, 95.0, 32.0), cq.Vector(1, 0, 0)))
    for y, z in taps:
        t = hole_x(t, y, z, 1.7)
    fx0, fx1 = (33.0, ARM_X[0]) if s > 0 else (-ARM_X[0], -33.0)
    t = t.fuse(block(fx0, fx1, CROSS_Y[1], 96.0, RAIL_Z0, RAIL_Z0 + TAB))
    add('Tab_Upright_' + side, t, 'steel_tab')
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
# drive motor support strap between the rails, EPDM pads up to the motor underside (z 19.5) at the inner ends
add('Strap_Drive_Motor', block(-RAIL_X[0], RAIL_X[0], 10.0, 21.0, RAIL_Z1 - TAB, RAIL_Z1), 'steel_tab')
for side, s in (('L', -1), ('R', 1)):
    x0, x1 = (4.0, 14.0) if s > 0 else (-14.0, -4.0)
    add('Pad_Drive_Motor_EPDM_' + side, block(x0, x1, 12.0, 20.0, RAIL_Z1, 19.5), 'EPDM')

# PC 2 mm skins
floor = block(-WALL_X, WALL_X, REAR_Y + SKIN, CROSS_Y[1], FLOOR_Z0, RAIL_Z0)
for s in (-1, 1):
    x0, x1 = (ARM_X[0], WALL_X) if s > 0 else (-WALL_X, -ARM_X[0])
    floor = floor.fuse(block(x0, x1, CROSS_Y[1] - 1, BEAM_Y[1], FLOOR_Z0, RAIL_Z0))
    floor = floor.cut(block(*((61.0, 79.0) if s > 0 else (-79.0, -61.0)), FRONT_Y - 21, FRONT_Y + 21, 0, 10))
add('Skin_PC2_Floor', floor, 'PC')
add('Skin_PC2_Rear_Wall', block(-WALL_X, WALL_X, REAR_Y, REAR_Y + SKIN, FLOOR_Z0, LID_Z0), 'PC')
for side, s in (('L', -1), ('R', 1)):
    x0 = WALL_X - SKIN if s > 0 else -WALL_X
    add('Skin_PC2_Side_Wall_' + side, block(x0, x0 + SKIN, REAR_Y + SKIN, BEAM_Y[0], RAIL_Z0, LID_Z0), 'PC')
lid = block(-WALL_X, WALL_X, REAR_Y, CROSS_Y[1], LID_Z0, LID_Z0 + SKIN)
for s in (-1, 1):
    lid = lid.cut(block(*((61.0, 79.0) if s > 0 else (-79.0, -61.0)), FRONT_Y - 23, FRONT_Y + 23, 0, 80))
add('Skin_PC2_Lid', lid, 'PC')
add('Electronics_Plate_PC2', block(-61, 61, -56, -1.5, 37.6, 39.6), 'PC')      # IMU foam sits on it (z 39.6)

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

# every tube and tab must touch another frame member (a weld needs contact); straps/tabs touching only skins fail
steel = [b for b in new if b.material.startswith('steel')]
floating = [a.name for a in steel if min(a.shape.distance(o.shape) for o in steel if o is not a) > 0.01]

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
cut_list = [{'part': b.name, 'length_mm': round(max(b.shape.BoundingBox().xlen, b.shape.BoundingBox().ylen, b.shape.BoundingBox().zlen), 1)}
            for b in new if b.material == 'steel_tube']
report = {
    'status': 'CONCEPT_CAD', 'revision': 'V4 R4-steel set A (2WD + PC 2 mm + 3S 650)', 'orientation': 'Z-up, nose +Y, mm',
    'removed_from_r3': sorted(removed), 'invalid_bodies': invalid, 'frame_members_not_touching': floating, 'interference': interference, 'upright_screw_head_hits': screw_hits,
    'upright_taps_yz': taps,
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
          'wheel_bought': '#2c2c2c', 'battery_650': '#566ca3', 'belt': '#343b45', 'UHMW': '#eeeeee', 'EPDM': '#222', 'nylon_standoff': '#ddd'}
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

print(json.dumps({k: report[k] for k in ('invalid_bodies', 'frame_members_not_touching', 'interference', 'upright_screw_head_hits', 'mass', 'support_2wd', 'tube_total_mm', 'bounds_mm')},
                 ensure_ascii=False, indent=1))
