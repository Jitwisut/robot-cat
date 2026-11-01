"""V5 concept CAD: SawBlaze-style hammersaw, budget-cut build (robot_v5/budget_cuts_v5.py).

CONCEPT CAD for fit, swing, mass and balance. No print/laser files are released from it.
Axes per AGENTS.md: Z up, nose +Y, X lateral, mm; floor is Z = 0 under the wheels.
Key sizes come from robot_v5/design_v5.py; the solids measured here (disc and arm inertia,
arm first moment, mass, CG, self-righting lever) are written to output/v5_cad_props.json,
which design_v5.py reads back so the calculations and the CAD use the same numbers.

Arrangement chosen for a home stick welder (user has one, r4_diy):
  * welded steel arm module: 1.6 base plate bolted through the TPU tub floor, two 1.6 towers, a 1.6 nose
    tongue out through the tub front wall onto a UHMW floor skid, and two 3.2 x 12 hard-stop posts on the
    tongue directly under the arbor (a stop near the pivot would make the arm a lever)
  * arm = two 2.3 steel plates; 608s sit in welded steel housing rings at the pivot (sheet cannot
    seat a 7 mm bearing); 3 mm steel doubler rings at the arbor
  * saw motor A2212 on the right arm plate, 55 mm from the pivot (the pivot axis carries the
    servo + slip clutch on the left, so the motor cannot also be coaxial)
  * disc: 3 teeth (design_v5 recommendation), 3 mm SS400, on a 6061 hub with 2 x 608 on an M8 12.9 arbor
  * forks: 12 x 4 mild flat bar standing ON EDGE (12 tall), hinged on one M4 pin, resting on the floor
Run from the repository root:
  .venv-v4-print/bin/python robot_v5/cad/build_v5.py
Writes robot_v5/cad/output/.
"""
import json
import math
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt

HERE = Path(__file__).resolve().parent
V5 = HERE.parent
sys.path.insert(0, str(V5))
import design_v5 as d5  # noqa: E402

OUT = HERE / 'output'
OUT.mkdir(exist_ok=True)

# ---------------------------------------------------------------- layout (mm)
ARM_L = d5.ARM_L_MM                       # pivot -> arbor, 100
PIVOT = (-2.0, d5.PIVOT_H_MM)             # (y, z); z 90 so the arm clears the lid in the strike pose
DISC_TIP_R, DISC_BASE_R, DISC_T, DISC_BORE_R, TEETH = 42.5, 34.0, 3.0, 12.0, 3
STRIKE_DISC_BOTTOM_Z = 10.0               # tooth tips 10 mm above the floor, between the inner forks
PHI_STRIKE = -math.degrees(math.asin((PIVOT[1] - STRIKE_DISC_BOTTOM_Z - DISC_TIP_R) / ARM_L))
PHI_PARK = 110.0                          # arm up and slightly back
ARM_HALF_W, PLATE_T = 13.5, 1.6          # PLATE_T: towers, base, tongue
ARM_PLATE_T = 2.3                         # arm plates: 1.6 gave SF 1.01 at the stop under a 69 J hit (load_path)
PLATE_X = (10.0, 10.0 + ARM_PLATE_T)      # |x| of each arm plate
MOTOR_U = 55.0                            # saw motor centre along the arm
TOWER_X = (19.6, 19.6 + PLATE_T)
TOWER_BASE_Y = (-12.0, 14.0)
# Hard stop UNDER THE ARBOR (u ~ 90 of 100): a stop near the pivot turns the arm into a lever (stop ~3.4 F,
# pivot ~2.4 F, arm plates far past yield). Posts stand on a steel nose tongue that rests on a UHMW floor skid,
# so a downward hit on the disc goes post -> tongue -> skid -> arena floor.
POST_Y = (70.0, 82.0)                     # 3.2 x 12 flat-bar posts, one under each arm plate
POST_X = (10.0, 13.2)
TONGUE = dict(x=13.6, y0=42.0, y1=82.0)   # 1.6 sheet, welded to the base plate, out through a slot in the tub front wall
SKID = dict(y0=66.0, z0=1.0)              # UHMW under the tongue, 1 mm off the floor (V4 nose-skid rule)
WHEEL_R, WHEEL_W, WHEEL_X0 = d5.WHEEL_D_MM / 2, 16.0, 80.0
AXLES_Y = (-62.0, -62.0 + d5.WHEELBASE_MM)    # front motor (r 12.5) clears the tub front wall
AXLE_Z = WHEEL_R
TUB = dict(x=77.0, y0=-95.0, y1=45.0, z0=4.0, z1=52.0, wall=3.0, floor=3.0)
LID_T = 2.0
LID_CUT = (23.0, -18.0, 42.0)             # |x|, y0, y1: opening for towers (boss r 15 about y -2) and posts
BASE_Z = (TUB['z0'] + TUB['floor'], TUB['z0'] + TUB['floor'] + PLATE_T)
PIN = (52.0, 14.0)                        # fork hinge pin (y, z), M4 along X
FORK_T, FORK_H, FORK_L = 4.0, 12.0, 72.0  # on edge: 4 wide in X, 12 tall
FORK_XC = (-56.0, -17.0, 17.0, 56.0)      # inner forks clear the stop posts (x 13.2) by 1.8 mm
BLOCKS_X = ((21.0, 30.0), (40.0, 52.0), (62.0, 74.0))

RHO = {'steel': 7.85e-3, 'al6061': 2.70e-3, 'tpu': 1.21e-3 * 0.85, 'petg': 1.27e-3 * 0.80, 'uhmw': 0.94e-3}
WELD = 0.08                               # ARC beads, fraction of welded steel (r4_diy assumption)
EXPLICIT_G = {                            # bought parts: envelope solids carry these masses (ASSUMED unless noted)
    'jga25': 110.0,                      # R3 inputs nominal
    'wheel': 45.0,                       # PETG hub 20 + TPU tyre 25 (V4 design_v4.py)
    'battery': 80.0,                     # R3 inputs nominal (GNB 850 3S)
    'esp32': 10.0, 'drv8871': 5.0, 'esc30': 25.0, 'ubec': 15.0,
    'servo_ds3218': 60.0, 'clutch': 20.0, 'a2212': 52.0, 'motor_mount': 6.0, 'pulley': 3.0,
    'bearing_608': 12.0,                 # R3 inputs nominal
}
EXTRA_ROWS = [('wiring + connectors', 50.0, (0, -30, 30)), ('sensors IMU/hall/temp', 12.0, (0, -40, 40)),
              ('switch + fuse', 15.0, (60, -80, 30)), ('fasteners + inserts', 40.0, (0, -20, 20))]

bodies = []          # dicts: name, shape, kind ('made'/'bought'), material, group ('arm' or 'frame')


def add(name, shape, material, group='frame'):
    bodies.append({'name': name, 'shape': shape, 'material': material, 'group': group})
    return shape


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cyl_x(x0, x1, y, z, r):
    return cq.Solid.makeCylinder(r, x1 - x0, cq.Vector(x0, y, z), cq.Vector(1, 0, 0))


def ring_x(x0, x1, y, z, ro, ri):
    return cyl_x(x0, x1, y, z, ro).cut(cyl_x(x0 - 1, x1 + 1, y, z, ri))


def yz_prism(pts, x0, x1):
    return cq.Workplane('YZ', origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def hull2d(pts):
    pts = sorted(set(pts))

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(pts[::-1])
    return lo[:-1] + up[:-1]


def mirror_x(shape):
    return shape.mirror('YZ')


# ---------------------------------------------------------------- tub, lid, drive
t = TUB
tub = box(-t['x'], t['x'], t['y0'], t['y1'], t['z0'], t['z1']).cut(
    box(-t['x'] + t['wall'], t['x'] - t['wall'], t['y0'] + t['wall'], t['y1'] - t['wall'], t['z0'] + t['floor'], t['z1'] + 1))
for y in AXLES_Y:
    tub = tub.cut(cyl_x(-t['x'] - 1, t['x'] + 1, y, AXLE_Z, 4.0))
tub = tub.cut(box(-TONGUE['x'], TONGUE['x'], t['y1'] - t['wall'] - 1, t['y1'] + 1, BASE_Z[0], BASE_Z[1]))   # tongue slot
add('Tub_TPU', tub, 'tpu')
lid = box(-t['x'], t['x'], t['y0'], t['y1'], t['z1'], t['z1'] + LID_T).cut(
    box(-LID_CUT[0], LID_CUT[0], LID_CUT[1], LID_CUT[2], t['z1'] - 1, t['z1'] + LID_T + 1))
add('Lid_TPU_2mm', lid, 'tpu')
for y, tag in zip(AXLES_Y, ('Rear', 'Front')):
    for s, side in ((-1, 'L'), (1, 'R')):
        x_in, x_out = 22.0, t['x'] - t['wall']
        add(f'JGA25_370_{tag}_{side}', cyl_x(*sorted((s * x_in, s * x_out)), y, AXLE_Z, 12.5), 'jga25')
        add(f'Wheel_{tag}_{side}', cyl_x(*sorted((s * WHEEL_X0, s * (WHEEL_X0 + WHEEL_W))), y, AXLE_Z, WHEEL_R), 'wheel')

# ---------------------------------------------------------------- electronics (envelopes)
add('Battery_3S_850', box(-17.5, 17.5, -90.0, -15.0, 7.0, 32.0), 'battery')
add('Shelf_PETG_2mm', box(-21.0, 21.0, -90.0, -15.0, 33.0, 35.0), 'petg')
add('ESP32_DevKit', box(-14.15, 14.15, -82.0, -30.5, 35.0, 49.0), 'esp32')
add('ESC_30A', box(30.0, 54.0, -35.0, 10.0, 7.0, 15.0), 'esc30')
add('UBEC_5A', box(-54.0, -34.0, -35.0, 5.0, 7.0, 17.0), 'ubec')
add('DRV8871_L', box(-60.0, -40.0, -42.0, -22.0, 30.0, 36.0), 'drv8871')
add('DRV8871_R', box(40.0, 60.0, -42.0, -22.0, 30.0, 36.0), 'drv8871')

# ---------------------------------------------------------------- welded arm module (frame side)
add('Arm_Module_Base_1.6', box(-TOWER_X[1], TOWER_X[1], TOWER_BASE_Y[0], TUB['y1'] - TUB['wall'], *BASE_Z), 'steel')
add('Nose_Tongue_1.6', box(-TONGUE['x'], TONGUE['x'], TONGUE['y0'], TONGUE['y1'], *BASE_Z), 'steel')
add('Nose_Skid_UHMW', box(-TONGUE['x'], TONGUE['x'], SKID['y0'], TONGUE['y1'], SKID['z0'], BASE_Z[0]), 'uhmw')
circ = [(PIVOT[0] + 15 * math.cos(a), PIVOT[1] + 15 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
tower_prof = hull2d([(TOWER_BASE_Y[0], BASE_Z[1]), (TOWER_BASE_Y[1], BASE_Z[1])] + circ)
tower = yz_prism(tower_prof, *TOWER_X).cut(cyl_x(0, 30, PIVOT[0], PIVOT[1], 4.1))
add('Tower_R_1.6', tower, 'steel')
add('Tower_L_1.6', mirror_x(tower), 'steel')
add('Pivot_Shaft_M8_12.9', cyl_x(-TOWER_X[1], TOWER_X[1] + 6.5, PIVOT[0], PIVOT[1], 4.0), 'steel')
add('Slip_Clutch', cyl_x(-TOWER_X[1] - 6.0, -TOWER_X[1], PIVOT[0], PIVOT[1], 15.0), 'clutch')
add('Servo_DS3218', box(-TOWER_X[1] - 46.0, -TOWER_X[1] - 6.0, PIVOT[0] - 10.0, PIVOT[0] + 30.5, PIVOT[1] - 10.0, PIVOT[1] + 10.0), 'servo_ds3218')


def arm_edge_z(y, phi):
    """z of the arm plates' lower edge at global y, arm at angle phi (deg)."""
    p = math.radians(phi)
    u = (y - PIVOT[0] - ARM_HALF_W * math.sin(p)) / math.cos(p)
    return PIVOT[1] + u * math.sin(p) - ARM_HALF_W * math.cos(p)


post_prof = [(POST_Y[0], BASE_Z[1]), (POST_Y[1], BASE_Z[1]),
             (POST_Y[1], arm_edge_z(POST_Y[1], PHI_STRIKE)), (POST_Y[0], arm_edge_z(POST_Y[0], PHI_STRIKE))]
post = yz_prism(post_prof, *POST_X)
add('Stop_Post_R_3.2x12', post, 'steel')
add('Stop_Post_L_3.2x12', mirror_x(post), 'steel')

# ---------------------------------------------------------------- arm group, built horizontal (phi = 0), then rotated
Y0, Z0 = PIVOT
arm = []


def add_arm(name, shape, material):
    arm.append({'name': name, 'shape0': shape, 'material': material, 'group': 'arm'})


plate = box(PLATE_X[0], PLATE_X[1], Y0, Y0 + ARM_L, Z0 - ARM_HALF_W, Z0 + ARM_HALF_W)
plate = plate.fuse(cyl_x(*PLATE_X, Y0, Z0, ARM_HALF_W)).fuse(cyl_x(*PLATE_X, Y0 + ARM_L, Z0, ARM_HALF_W))
plate = plate.cut(cyl_x(0, 30, Y0, Z0, 7.0)).cut(cyl_x(0, 30, Y0 + ARM_L, Z0, 4.1))
plate_r = plate.cut(cyl_x(0, 30, Y0 + MOTOR_U, Z0, 3.0))                 # motor shaft passes through the right plate
add_arm('Arm_Plate_R_2.3', plate_r, 'steel')
add_arm('Arm_Plate_L_2.3', mirror_x(plate), 'steel')
for s, side in ((1, 'R'), (-1, 'L')):
    xa, xb = sorted((s * PLATE_X[1], s * (PLATE_X[1] + 7.0)))
    add_arm(f'Pivot_Housing_Ring_{side}', ring_x(xa, xb, Y0, Z0, 13.0, 11.0), 'steel')
    add_arm(f'Pivot_608_{side}', ring_x(xa, xb, Y0, Z0, 11.0, 4.0), 'bearing_608')
    xa, xb = sorted((s * PLATE_X[1], s * (PLATE_X[1] + 3.0)))
    add_arm(f'Arbor_Doubler_{side}', ring_x(xa, xb, Y0 + ARM_L, Z0, 10.0, 4.1), 'steel')
    xa, xb = sorted((s * 8.0, s * PLATE_X[0]))
    add_arm(f'Arbor_Spacer_{side}', ring_x(xa, xb, Y0 + ARM_L, Z0, 5.5, 4.0), 'al6061')
add_arm('Arbor_M8_12.9', cyl_x(-PLATE_X[1] - 3.0, PLATE_X[1] + 9.5, Y0 + ARM_L, Z0, 4.0), 'steel')
hub = cyl_x(-8.0, -DISC_T / 2, Y0 + ARM_L, Z0, 17.0).fuse(cyl_x(DISC_T / 2, 8.0, Y0 + ARM_L, Z0, 17.0)) \
    .fuse(cyl_x(-DISC_T / 2, DISC_T / 2, Y0 + ARM_L, Z0, DISC_BORE_R))
hub = hub.cut(cyl_x(-9, 9, Y0 + ARM_L, Z0, 11.0))
groove = ring_x(4.0, 6.0, Y0 + ARM_L, Z0, 18.0, 15.0)                     # 5 mm PU round-belt groove
hub = hub.cut(groove)
add_arm('Saw_Hub_6061', hub, 'al6061')
add_arm('Hub_608_A', ring_x(-8.0, -1.0, Y0 + ARM_L, Z0, 11.0, 4.0), 'bearing_608')
add_arm('Hub_608_B', ring_x(1.0, 8.0, Y0 + ARM_L, Z0, 11.0, 4.0), 'bearing_608')


def disc_profile():
    """3-tooth saw disc (u, w in the disc plane), leading face radial, back slope to the base circle."""
    pts = []
    for k in range(TEETH):
        a0 = 2 * math.pi * k / TEETH
        for a in np.linspace(a0, a0 + math.radians(28), 8):          # tooth top at the tip radius
            pts.append((DISC_TIP_R * math.cos(a), DISC_TIP_R * math.sin(a)))
        for a in np.linspace(a0 + math.radians(30), a0 + math.radians(75), 10):   # back slope down to base
            f = (a - a0 - math.radians(30)) / math.radians(45)
            r = DISC_TIP_R - f * (DISC_TIP_R - DISC_BASE_R)
            pts.append((r * math.cos(a), r * math.sin(a)))
        for a in np.linspace(a0 + math.radians(76), a0 + 2 * math.pi / TEETH, 12, endpoint=False):
            pts.append((DISC_BASE_R * math.cos(a), DISC_BASE_R * math.sin(a)))
        a1 = a0 + 2 * math.pi / TEETH                                    # radial leading face of the next tooth
        pts.append((DISC_BASE_R * math.cos(a1), DISC_BASE_R * math.sin(a1)))
    return pts


# w is mirrored so the radial faces LEAD under +X spin (front teeth up, design_v5 rule_spin_direction);
# disc_profile() as written has them trailing a counter-clockwise turn.
dp = [(Y0 + ARM_L + u, Z0 - w) for u, w in disc_profile()]
disc = yz_prism(dp, -DISC_T / 2, DISC_T / 2).cut(cyl_x(-5, 5, Y0 + ARM_L, Z0, DISC_BORE_R))
for k in range(3):
    a = math.radians(90 + 120 * k)
    disc = disc.cut(cyl_x(-5, 5, Y0 + ARM_L + 14.5 * math.cos(a), Z0 + 14.5 * math.sin(a), 1.6))
add_arm('Saw_Disc_3T_SS400_3mm', disc, 'steel')
add_arm('Saw_Motor_A2212', cyl_x(PLATE_X[1] + 3.0, PLATE_X[1] + 29.0, Y0 + MOTOR_U, Z0, 14.0), 'a2212')
add_arm('Saw_Motor_Mount', cyl_x(PLATE_X[1], PLATE_X[1] + 3.0, Y0 + MOTOR_U, Z0, 14.0), 'motor_mount')
add_arm('Saw_Motor_Shaft', cyl_x(3.0, PLATE_X[1], Y0 + MOTOR_U, Z0, 1.6), 'steel')
add_arm('Motor_Pulley', ring_x(3.0, 7.0, Y0 + MOTOR_U, Z0, 5.0, 1.6), 'pulley')


def arm_at(phi):
    return [dict(b, shape=b['shape0'].rotate(cq.Vector(0, Y0, Z0), cq.Vector(1, Y0, Z0), phi)) for b in arm]


# ---------------------------------------------------------------- forks + hinge
def solve_alpha():
    lo, hi = 0.0, 30.0
    for _ in range(60):
        a = (lo + hi) / 2
        z = PIN[1] - FORK_L * math.sin(math.radians(a)) - FORK_H / 2 * math.cos(math.radians(a))
        lo, hi = (a, hi) if z > 0.5 else (lo, a)
    return (lo + hi) / 2


ALPHA = solve_alpha()                     # rest angle: front-bottom corner 0.5 mm above the floor
fork_local = yz_prism([(PIN[0] - 6.0, PIN[1] - FORK_H / 2), (PIN[0] + FORK_L, PIN[1] - FORK_H / 2),
                       (PIN[0] + FORK_L, PIN[1] - FORK_H / 2 + 1.0), (PIN[0] + 38.0, PIN[1] + FORK_H / 2),
                       (PIN[0] - 6.0, PIN[1] + FORK_H / 2)], -FORK_T / 2, FORK_T / 2)
fork_local = fork_local.fuse(cyl_x(-FORK_T / 2, FORK_T / 2, PIN[0], PIN[1], 6.0)).cut(cyl_x(-5, 5, PIN[0], PIN[1], 2.1))
forks = []
for i, xc in enumerate(FORK_XC):
    f = fork_local.translate(cq.Vector(xc, 0, 0)).rotate(cq.Vector(0, *PIN), cq.Vector(1, *PIN), -ALPHA)
    name = f'Fork_{i + 1}_mild_12x4_on_edge'
    add(name, f, 'steel')
    forks.append(name)
add('Fork_Pin_M4', cyl_x(-74.0, 74.0, PIN[0], PIN[1], 2.0), 'steel')
for x0, x1 in BLOCKS_X:
    for s in (1, -1):
        blk = box(*sorted((s * x0, s * x1)), TUB['y1'], PIN[0] + 6.0, TUB['z0'], PIN[1] + 8.0).cut(cyl_x(-80, 80, PIN[0], PIN[1], 2.1))
        add(f'Hinge_Block_PETG_{"R" if s > 0 else "L"}_{int(x0)}', blk, 'petg')


# ---------------------------------------------------------------- mass properties
def vol_props(shape, axis_pt=None):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, g)
    out = {'V': g.Mass(), 'c': np.array([g.CentreOfMass().X(), g.CentreOfMass().Y(), g.CentreOfMass().Z()])}
    if axis_pt is not None:
        out['Ix'] = g.MomentOfInertia(gp_Ax1(gp_Pnt(0, axis_pt[0], axis_pt[1]), gp_Dir(1, 0, 0)))
    return out


def body_mass(b, axis_pt=None):
    p = vol_props(b['shape'], axis_pt)
    m = EXPLICIT_G[b['material']] if b['material'] in EXPLICIT_G else p['V'] * RHO[b['material']]
    rho = m / p['V']
    return m, p['c'], (p['Ix'] * rho if axis_pt is not None else None)


def overlap(a, b, tol=0.01):
    A, B = a.BoundingBox(), b.BoundingBox()
    return (A.xmin < B.xmax - tol and B.xmin < A.xmax - tol and A.ymin < B.ymax - tol and B.ymin < A.ymax - tol
            and A.zmin < B.zmax - tol and B.zmin < A.zmax - tol)


def interferences(items, skip=()):
    res = []
    for i, a in enumerate(items):
        for b in items[i + 1:]:
            if {a['name'], b['name']} in skip or not overlap(a['shape'], b['shape']):
                continue
            v = a['shape'].intersect(b['shape']).Volume()
            if v > 0.01:
                res.append({'a': a['name'], 'b': b['name'], 'volume_mm3': round(v, 2)})
    return res


def assembly(phi):
    return bodies + arm_at(phi)


# designed contacts: shaft in bores, bearings in housings/hub are line-to-line, so volume 0; nothing to skip
static = {}
for pose, phi in (('park', PHI_PARK), ('strike', PHI_STRIKE)):
    static[pose] = interferences(assembly(phi))

# arm swing: from the strike pose upward to park and on toward the back (self-right direction)
frame_items = [b for b in bodies]


def first_contact(phis, items_fn=lambda arm_b: arm_b):
    for phi in phis:
        for a in arm_at(phi):
            for o in frame_items:
                if overlap(a['shape'], o['shape']) and a['shape'].intersect(o['shape']).Volume() > 0.05:
                    return {'phi_deg': round(phi, 1), 'arm_part': a['name'], 'hits': o['name']}
    return None


swing = {
    'strike_phi_deg': round(PHI_STRIKE, 2), 'park_phi_deg': PHI_PARK,
    'below_strike (must hit the stop posts at once)': first_contact([PHI_STRIKE - 0.5, PHI_STRIKE - 1.0]),
    'strike_to_park': first_contact(np.arange(PHI_STRIKE + 1, PHI_PARK + 0.1, 1.0)),
    'park_onward_back': first_contact(np.arange(PHI_PARK + 1, 260.1, 1.0)),
}

# disc sweep: tip radius + 1 mm, thickness + 1 mm, at every degree strike -> park
sweep_hits = []
for phi in np.arange(PHI_STRIKE, PHI_PARK + 0.1, 2.0):
    p = math.radians(phi)
    c = (Y0 + ARM_L * math.cos(p), Z0 + ARM_L * math.sin(p))
    env = cyl_x(-DISC_T / 2 - 0.5, DISC_T / 2 + 0.5, c[0], c[1], DISC_TIP_R + 1.0)
    for o in frame_items:
        if overlap(env, o['shape']) and env.intersect(o['shape']).Volume() > 0.05:
            sweep_hits.append({'phi_deg': round(phi, 1), 'hits': o['name']})
strike_clear_to_forks = min(
    a['shape'].distance(o['shape']) for a in arm_at(PHI_STRIKE) for o in bodies if o['name'].startswith('Fork_'))

# fork hinge swing -2 .. +14 deg about the pin, arm in the strike pose (closest case)
fork_swing = {}
others_strike = [b for b in assembly(PHI_STRIKE)]
for name in forks:
    fb = next(b for b in bodies if b['name'] == name)
    res = {}
    for direction, steps in (('up', np.arange(1, 15)), ('down', np.arange(-1, -3, -1))):
        hit = None
        for deg in steps:
            r = fb['shape'].rotate(cq.Vector(0, *PIN), cq.Vector(1, *PIN), float(deg))
            for o in others_strike:
                if o['name'] in (name, 'Fork_Pin_M4'):
                    continue
                if overlap(r, o['shape']) and r.intersect(o['shape']).Volume() > 0.05:
                    hit = {'deg': float(deg), 'hits': o['name']}
                    break
            if hit:
                break
        res[direction] = hit or 'free'
    fork_swing[name] = res

# ---------------------------------------------------------------- mass, CG, arm properties
def mass_table(phi):
    rows, steel_welded = [], 0.0
    for b in assembly(phi):
        m, c, _ = body_mass(b)
        rows.append({'part': b['name'], 'material': b['material'], 'group': b['group'], 'mass_g': round(m, 2),
                     'centroid_mm': [round(v, 2) for v in c],
                     'method': 'ASSUMED bought-part mass' if b['material'] in EXPLICIT_G else 'CAD volume x density'})
        if b['material'] == 'steel' and any(k in b['name'] for k in ('Tower', 'Base', 'Post', 'Housing', 'Doubler', 'Arm_Plate', 'Tongue')):
            steel_welded += m
    rows.append({'part': 'weld beads (ARC)', 'mass_g': round(WELD * steel_welded, 2), 'centroid_mm': [0, 0, 50],
                 'method': '%.0f %% of welded steel, ASSUMED' % (WELD * 100), 'group': 'frame'})
    for n, g, c in EXTRA_ROWS:
        rows.append({'part': n, 'mass_g': g, 'centroid_mm': list(c), 'method': 'ASSUMED', 'group': 'frame'})
    total = sum(r['mass_g'] for r in rows)
    cg = sum(np.array(r['centroid_mm']) * r['mass_g'] for r in rows) / total
    return rows, total, cg


rows_park, total, cg_park = mass_table(PHI_PARK)
_, _, cg_strike = mass_table(PHI_STRIKE)

arm0 = [dict(b, shape=b['shape0']) for b in arm]
arm_m, arm_I, arm_first = 0.0, 0.0, 0.0
for b in arm0:
    m, c, ix = body_mass(b, axis_pt=PIVOT)
    arm_m += m
    arm_I += ix
    arm_first += m * (c[1] - Y0)                                         # g mm, arm horizontal
disc_b = next(b for b in arm0 if b['name'].startswith('Saw_Disc'))
disc_m, _, disc_I = body_mass(disc_b, axis_pt=(Y0 + ARM_L, Z0))
hub_parts = [b for b in arm0 if b['name'] in ('Saw_Hub_6061', 'Hub_608_A', 'Hub_608_B')]
spin_I = disc_I + sum(body_mass(b, axis_pt=(Y0 + ARM_L, Z0))[2] for b in hub_parts) * 0.5  # bearings: outer half spins, ASSUMED

# wheel loads, arm parked and striking (4WD, forks carry ~half their own weight)
fork_g = sum(r['mass_g'] for r in rows_park if r['part'].startswith('Fork_'))


def axle_split(cg):
    w_frac = (cg[1] - AXLES_Y[0]) / (AXLES_Y[1] - AXLES_Y[0])
    return {'cg_y_mm': round(cg[1], 1), 'cg_z_mm': round(cg[2], 1), 'front_axle_share': round(w_frac, 3),
            'rear_axle_share': round(1 - w_frac, 3), 'cg_ahead_of_front_axle_mm': round(cg[1] - AXLES_Y[1], 1)}


# ---------------------------------------------------------------- self-righting (2D, side view)
def side_points(items, step=1.5):
    pts = []
    for b in items:
        v, _ = b['shape'].tessellate(step, 0.5)
        pts += [(round(p.y, 2), round(p.z, 2)) for p in v]
    return pts


def self_right():
    """Inverted (gravity +Z in the body frame). Rest on the side hull edge closest to the CG, arm in the strike pose.
    The arm then swings toward park; where the disc tips first reach that floor line it pushes, and the robot rolls
    about the far end of the rest edge. Static moment balance at first contact gives the servo torque."""
    items = assembly(PHI_STRIKE)
    tags = {}
    pts = []
    for b in items:
        for p in side_points([b]):
            pts.append(p)
            tags.setdefault(p, b['group'])
    h = hull2d(pts)
    cg = np.array([cg_strike[1], cg_strike[2]])
    best = None
    for i in range(len(h)):
        a, b = np.array(h[i]), np.array(h[(i + 1) % len(h)])
        e = b - a
        L = np.linalg.norm(e)
        if L < 1e-6:
            continue
        n = np.array([e[1], -e[0]]) / L                                   # outward normal for a CCW hull
        if n[1] <= 0.2:                                                    # only top-side edges
            continue
        tpar = np.dot(cg - a, e) / L
        if not (0 <= tpar <= L):
            continue
        dist = abs(np.dot(cg - a, n))
        if best is None or dist < best[0]:
            best = (dist, a, b, e / L, n)
    if best is None:
        return {'status': 'no stable top edge found'}
    _, a, b, ehat, n = best
    m_kg = total / 1000
    out = {'rest_edge_yz': [a.round(1).tolist(), b.round(1).tolist()],
           'edge_groups': [tags.get(tuple(a), '?'), tags.get(tuple(b), '?')],
           'tilt_deg': round(math.degrees(math.atan2(ehat[1], ehat[0])), 1)}
    contact = None
    for phi in np.arange(PHI_STRIKE, 270.0, 0.5):                        # disc tip circle vs the floor line
        p = math.radians(phi)
        c = np.array([Y0 + ARM_L * math.cos(p), Z0 + ARM_L * math.sin(p)])
        if np.dot(c - a, n) + DISC_TIP_R >= 0:
            contact = (phi, c + n * DISC_TIP_R)
            break
    if contact is None:
        out['status'] = 'disc never reaches the floor line'
        return out
    phi_c, pc = contact
    other = a if abs(np.dot(pc - a, ehat)) > abs(np.dot(pc - b, ehat)) else b    # roll about the far end
    s = lambda p: float(np.dot(np.array(p) - other, ehat))
    lever_cg, lever_tip, lever_pivot = abs(s(cg)), abs(s(pc)), abs(s(pc) - s(PIVOT))
    out['arm_phi_at_floor_contact_deg'] = round(phi_c, 1)
    if lever_tip <= lever_cg:
        out['status'] = 'contact is between the roll point and the CG: the arm cannot lift the CG over the roll point'
        out.update({'d_cg_mm': round(lever_cg, 1), 'tip_lever_mm': round(lever_tip, 1)})
        return out
    f = m_kg * 9.81 * lever_cg / lever_tip
    tau = f * lever_pivot / 1000
    out.update({'roll_point_yz': other.round(1).tolist(), 'arm_contact_yz': pc.round(1).tolist(),
                'd_cg_mm': round(lever_cg, 1), 'tip_lever_mm': round(lever_tip, 1), 'pivot_to_contact_mm': round(lever_pivot, 1),
                'floor_force_N': round(f, 2), 'servo_torque_needed_Nm': round(tau, 3),
                'margin_DS3218': round(1.96 / tau, 2) if tau > 0 else None, 'status': 'ok'})
    return out


sr = self_right()


def roll_sweep(sr0):
    """Keep pushing after first contact: roll the body about the roll point until the CG passes over it.
    At each roll step the disc is re-solved tangent to the floor line; torque from static moment balance."""
    if sr0.get('status') != 'ok':
        return None
    R = np.array(sr0['roll_point_yz'])
    a, b = (np.array(p) for p in sr0['rest_edge_yz'])
    far = a if np.linalg.norm(a - R) > 1e-6 else b                       # the end that lifts off
    e0 = (far - R) / np.linalg.norm(far - R)
    cg = np.array([cg_strike[1], cg_strike[2]])
    m_kg, peak, rows = total / 1000, (0.0, None), []
    for sgn in (1, -1):                                                    # pick the roll sense that lifts 'far'
        rows = []
        for beta in np.arange(0.0, 120.0, 2.0):
            c_, s_ = math.cos(math.radians(sgn * beta)), math.sin(math.radians(sgn * beta))
            e = np.array([c_ * e0[0] - s_ * e0[1], s_ * e0[0] + c_ * e0[1]])
            n = np.array([e[1], -e[0]])
            if np.dot(cg - R, n) > 0:
                n = -n                                                      # n points from the body to the floor
            if beta > 0 and np.dot(far - R, n) >= 0:
                break                                                       # wrong sense: 'far' would go through the floor
            lever_cg = np.dot(cg - R, e)
            if lever_cg <= 0:
                break                                                       # CG is over the roll point: it falls over
            hit = None
            for phi in np.arange(PHI_STRIKE, 300.0, 0.5):
                p = math.radians(phi)
                c = np.array([Y0 + ARM_L * math.cos(p), Z0 + ARM_L * math.sin(p)])
                if np.dot(c - R, n) + DISC_TIP_R >= 0:
                    hit = (phi, c + n * DISC_TIP_R)
                    break
            if hit is None:
                rows.append({'roll_deg': beta, 'status': 'disc cannot reach the floor'})
                break
            lt, lp = np.dot(hit[1] - R, e), abs(np.dot(hit[1] - np.array(PIVOT), e))
            if lt <= lever_cg:
                rows.append({'roll_deg': beta, 'status': 'contact inside the CG lever'})
                break
            tau = m_kg * 9.81 * lever_cg / lt * lp / 1000
            rows.append({'roll_deg': beta, 'arm_phi_deg': round(hit[0], 1), 'torque_Nm': round(tau, 3)})
        if len(rows) > 1:
            break
    ok = [r for r in rows if 'torque_Nm' in r]
    pk = max(ok, key=lambda r: r['torque_Nm']) if ok else None
    finished = bool(rows) and 'status' not in rows[-1]
    return {'steps': rows, 'peak': pk, 'reaches_cg_over_roll_point': finished,
            'margin_vs_clutch_1.3Nm': round(1.3 / pk['torque_Nm'], 2) if pk else None,
            'margin_vs_DS3218_stall': round(1.96 / pk['torque_Nm'], 2) if pk else None}


sr['roll_sweep'] = roll_sweep(sr)


def load_path(energy_j):
    """Downward hit on the disc: arm as a beam pinned at the pivot (u=0), resting on the stop posts (u_s),
    loaded at the arbor (u=L). Same 1 ms impulse model as V4/design_v5 (1.5 kg opponent, eta 0.4)."""
    p = math.radians(PHI_STRIKE)
    yc = (POST_Y[0] + POST_Y[1]) / 2
    u_s = (yc - Y0 - ARM_HALF_W * math.sin(p)) / math.cos(p)
    v = math.sqrt(2 * 0.4 * energy_j / 1.5)
    F = 1.5 * v / 1e-3
    R_s = F * ARM_L / u_s
    R_p = R_s - F
    z_arm = 2 * ARM_PLATE_T * (2 * ARM_HALF_W) ** 2 / 6
    sig_arm = F * (ARM_L - u_s) / z_arm
    h_post = max(arm_edge_z(POST_Y[0], PHI_STRIKE), arm_edge_z(POST_Y[1], PHI_STRIKE)) - BASE_Z[1]
    wpost, tpost = POST_Y[1] - POST_Y[0], POST_X[1] - POST_X[0]
    i_post = wpost * tpost ** 3 / 12
    p_cr = math.pi ** 2 * 200e3 * i_post / (4 * h_post ** 2)
    p_post = R_s / 2
    sig_post = p_post / (wpost * tpost)
    c0 = 1370.0
    ratios = {'arm_plates_yield': 245 / sig_arm if sig_arm > 0 else 99, 'post_buckling': p_cr / p_post,
              'post_yield': 245 / sig_post, 'pivot_608_C0': c0 / max(abs(R_p) / 2, 1e-6)}
    worst = min(ratios.values())
    return {'energy_J': energy_j, 'F_peak_N': round(F), 'stop_u_mm': round(u_s, 1),
            'stop_reaction_N': round(R_s), 'pivot_reaction_N': round(R_p),
            'arm_plate_stress_at_stop_MPa': round(sig_arm), 'post_height_mm': round(h_post, 1),
            'post_load_N': round(p_post), 'post_buckling_N': round(p_cr), 'post_stress_MPa': round(sig_post),
            'pivot_load_per_608_N': round(abs(R_p) / 2),
            'safety_factors': {k: round(v_, 2) for k, v_ in ratios.items()},
            'survivable_hit_energy_J (all SF >= 1)': round(energy_j * worst ** 2, 1)}


ENERGY_J = d5.saw['B_steel_disc_85mm_3mm_3T']['energy_J'] if 'B_steel_disc_85mm_3mm_3T' in d5.saw else 69.0
load = load_path(ENERGY_J)

# ---------------------------------------------------------------- outputs
all_park = assembly(PHI_PARK)
comp_park = cq.Compound.makeCompound([b['shape'] for b in all_park])
comp_strike = cq.Compound.makeCompound([b['shape'] for b in assembly(PHI_STRIKE)])
bb = comp_park.BoundingBox()
bb_s = comp_strike.BoundingBox()
step_path = OUT / 'ROBOT_V5_CONCEPT_PARK.step'
comp_park.exportStep(str(step_path))
# Named, coloured assembly for Fusion (one component per part, park pose); geometry identical to the compound
_COL = {'steel': (0.90, 0.49, 0.13), 'tpu': (0.34, 0.70, 0.69), 'petg': (0.85, 0.85, 0.85), 'uhmw': (0.96, 0.96, 0.96),
        'al6061': (0.65, 0.71, 0.77), 'wheel': (0.17, 0.17, 0.17), 'a2212': (0.56, 0.27, 0.68), 'servo_ds3218': (0.12, 0.23, 0.58)}
asm = cq.Assembly(name='ROBOT_V5_CONCEPT')
for _b in all_park:
    asm.add(cq.Workplane().add(_b['shape']), name=_b['name'], color=cq.Color(*_COL.get(_b['material'], (0.34, 0.42, 0.64))))
asm.save(str(OUT / 'ROBOT_V5_CONCEPT_ASSEMBLY.step'))
comp_strike.exportStep(str(OUT / 'ROBOT_V5_CONCEPT_STRIKE.step'))
re = cq.importers.importStep(str(step_path)).val().BoundingBox()
bounds = lambda b: [round(v, 2) for v in (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)]


def tight(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    bx = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, bx, False, False)
    return cq.BoundBox(bx)


bb, bb_s, re = tight(comp_park), tight(comp_strike), tight(cq.importers.importStep(str(step_path)).val())
wheel_floor = min(tight(b['shape']).zmin for b in bodies if b['name'].startswith('Wheel_'))

props = {
    'source': 'robot_v5/cad/build_v5.py', 'pivot_yz_mm': list(PIVOT), 'arm_L_mm': ARM_L,
    'phi_strike_deg': round(PHI_STRIKE, 2), 'phi_park_deg': PHI_PARK, 'swing_deg': round(PHI_PARK - PHI_STRIKE, 2),
    'disc': {'teeth': TEETH, 'tip_r_mm': DISC_TIP_R, 'base_r_mm': DISC_BASE_R, 't_mm': DISC_T, 'bore_r_mm': DISC_BORE_R,
             'mass_g': round(disc_m, 2), 'I_kgm2': disc_I * 1e-9, 'spinning_I_kgm2 (disc + hub + half bearings)': spin_I * 1e-9},
    'arm': {'mass_g': round(arm_m, 1), 'I_pivot_kgm2': arm_I * 1e-9, 'first_moment_kgm (horizontal)': arm_first * 1e-6},
    'mass_total_g': round(total, 1), 'self_right': sr, 'load_path': load,
}
(OUT / 'v5_cad_props.json').write_text(json.dumps(props, indent=1) + '\n')

report = {
    'status': 'CONCEPT_CAD', 'orientation': 'Z-up, nose +Y, X lateral, mm; floor Z=0 under the wheels',
    'wheel_bottom_z_mm': round(wheel_floor, 3),
    'bounds_park_mm': bounds(bb), 'bounds_strike_mm': bounds(bb_s), 'step_reimport_bounds_park_mm': bounds(re),
    'invalid_bodies': [b['name'] for b in all_park if not b['shape'].isValid()],
    'interference': static, 'arm_swing': swing, 'disc_sweep_hits': sweep_hits,
    'arm_to_fork_min_gap_strike_mm': round(strike_clear_to_forks, 2),
    'fork_rest_angle_deg': round(ALPHA, 2), 'fork_swing': fork_swing, 'load_path': load,
    'mass': {'total_g': round(total, 1), 'margin_to_2000_g': round(2000 - total, 1),
             'vs_budget_cuts_estimate_1929_g': round(total - 1929, 1), 'fork_g': round(fork_g, 1),
             'groups_g': {g: round(sum(r['mass_g'] for r in rows_park if r.get('group') == g), 1) for g in ('arm', 'frame')}},
    'balance': {'park': axle_split(cg_park), 'strike': axle_split(cg_strike)},
    'props': props, 'rows_park': rows_park,
}
(OUT / 'build_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')

# ---------------------------------------------------------------- preview
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
COL = {'steel': '#e67e22', 'tpu': '#56b2b0', 'petg': '#d9d9d9', 'uhmw': '#f5f5f5', 'al6061': '#a6b4c5', 'jga25': '#566ca3', 'wheel': '#2c2c2c',
       'battery': '#566ca3', 'esp32': '#2e86de', 'esc30': '#566ca3', 'ubec': '#566ca3', 'drv8871': '#c0392b',
       'servo_ds3218': '#1f3a93', 'clutch': '#7f8c8d', 'a2212': '#8e44ad', 'motor_mount': '#7f8c8d', 'pulley': '#7f8c8d',
       'bearing_608': '#95a5a6'}


def draw(ax, items, title, elev, azim):
    for b in items:
        v, tri = b['shape'].tessellate(0.6, 0.4)
        if not tri:
            continue
        v = np.array([p.toTuple() for p in v])
        alpha = 0.25 if b['material'] == 'tpu' else 0.95
        ax.add_collection3d(Poly3DCollection(v[np.array(tri)], facecolors=COL.get(b['material'], '#aaa'), edgecolors='none', alpha=alpha))
    ax.set_xlim(-110, 110); ax.set_ylim(-110, 150); ax.set_zlim(0, 240); ax.set_box_aspect((220, 260, 240))
    ax.view_init(elev=elev, azim=azim); ax.set_xlabel('X'); ax.set_ylabel('Y (nose)'); ax.set_zlabel('Z (up)'); ax.set_title(title)


fig = plt.figure(figsize=(18, 6.5), facecolor='#fafafa')
draw(fig.add_subplot(1, 3, 1, projection='3d'), assembly(PHI_PARK), 'Park (arm up, %.0f°)' % PHI_PARK, 24, -55)
draw(fig.add_subplot(1, 3, 2, projection='3d'), assembly(PHI_STRIKE), 'Strike (on the stops, %.1f°)' % PHI_STRIKE, 24, -55)
draw(fig.add_subplot(1, 3, 3, projection='3d'), assembly(PHI_STRIKE), 'Strike, side view', 0, 0)
fig.suptitle('ROBOT V5 — CONCEPT CAD · %.0f g · Z up, nose +Y' % total, fontsize=15, fontweight='bold')
fig.text(0.5, 0.03, 'steel: orange | TPU tub/lid: teal | PETG: grey | bought parts: blue/purple/black', ha='center')
fig.tight_layout(rect=(0, 0.05, 1, 0.93))
fig.savefig(OUT / 'v5_preview.png', dpi=130)
plt.close(fig)

print(json.dumps({k: report[k] for k in ('wheel_bottom_z_mm', 'bounds_park_mm', 'bounds_strike_mm', 'step_reimport_bounds_park_mm',
                                          'invalid_bodies', 'interference', 'arm_swing', 'disc_sweep_hits', 'arm_to_fork_min_gap_strike_mm',
                                          'fork_rest_angle_deg', 'fork_swing', 'mass', 'balance')}, ensure_ascii=False, indent=1))
print(json.dumps(props, indent=1))
