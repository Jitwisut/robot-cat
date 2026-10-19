"""V4 R3 with the printed TPU tub + lid replaced by a 3 mm polycarbonate (PC) sheet body. Z-up, nose +Y, mm.

DRAFT body variant. R3 itself (r3/output_R3) is not touched: this script rebuilds the R3 geometry in memory with
revision.build(), removes Tub_TPU and Lid_TPU_2mm, and adds
  * 7 flat PC 3 mm panels for a CNC router (DXF + STEP): floor, side walls L/R, rear wall, bulkheads L/R, lid
  * PETG prints that take over what the tub carried: 4 drive-motor cradles, 2 weapon side blocks, 14 joining
    cleats with heat-set inserts (3 mm PC cannot take edge threads or T-nuts) and 11 skirt brackets
  * 3 R3 prints re-issued as PC-body variants: hall post 1 mm shorter, wedgelet blocks with 2 floor inserts each
  * every new screw / insert as an explicit mass row at its position
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r3/pc_body/build_pc_body.py
Writes robot_v4/print_revision/r3/pc_body/output/.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PR = HERE.parents[1]                                    # robot_v4/print_revision
sys.path.insert(0, str(PR))

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
import ezdxf
import numpy as np
from cad_backend import Body, part, group, block, bore, prism, find_body
from revision import build, value, fit, HUB_NOSE_R
from legacy_geometry import SKIRT_T
from checks import mass_properties, build_signature

INPUTS = PR / 'r3' / 'inputs_R3.json'
OUT = HERE / 'output'
config = json.loads(INPUTS.read_text())
design = config['design']
FRONT_Y, REAR_Y = design['front_axle_y_mm'], design['rear_axle_y_mm']
MOTOR_Y, MOTOR_Z = design['weapon_motor_y_mm'], design['weapon_motor_z_mm']
WHEEL_Y = (FRONT_Y, REAR_Y)

# ---------------------------------------------------------------- body parameters (mm)
T = 3.0                    # PC sheet thickness
PC_DENSITY = 0.00120       # g/mm^3, solid general-purpose PC (datasheets 1.19-1.21 g/cm^3)
WALL_IN = 80.0             # side-wall inner face |x|: same as the tub, so the wedgelet rail and switch collar keep their faces
WALL_OUT = WALL_IN + T     # 83
FLOOR_Z0 = 4.0             # floor z 4..7, same as the tub: every R3 part above it stays where it is
TOP_Z = 59.0               # wall tops (tub: 58); lid z 59..62
LID_TOP = TOP_Z + T        # 62: 2 mm under the inverted-wheel plane (z 64); R3 lid arch reached 63
LID_POCKET_DEPTH = 2.0     # router pocket under the lid over the weapon motor: 1 mm skin, 1 mm motor clearance
REAR_OUT = -110.0          # rear wall outer face, same as the tub
T_BULK = 2.0               # bulkheads: internal partitions, not armour (CALC_PC_BODY.md section 4)
BULK_Y0 = 64.0 - T_BULK    # bulkhead y 62..64, the R3 tub bulkhead's place: uprights start at 64, R3 hall post ends at 61.5
WALL_FRONT = 97.0          # side walls end where the tub walls ended (wedgelet swing room)
WELL_FLOOR = 24.0          # floor wheel notch half-length; tyre chord at the floor top (z 7) is +-20.0
WELL_LID = 23.0            # lid wheel hole half-length; tyre chord at the lid underside (z 59) is +-17.2
CORNER_R = 1.5             # internal corner radius = D3 router bit
HOLE_R = 1.7               # M3 clearance D3.4
SELF_TAP_R = 1.45          # D2.9 clearance for the R3 M2.5 self-tapping screws (switch collar, hall post)
CRADLE_INSERT_X = 54.0     # |x| of the 2 cradle floor inserts, right under the wall root
CRADLE_TOP = 40.0          # cradle wall top (tub motor walls: 58); head recesses end at z 35.75
CRADLE_FOOT = 50.0         # cradle foot inner edge |x|
PAD_T = 0.5                # EPDM/silicone pad in the twin-saddle and lid-bar cups (adhesive backed)
SUPPORT_X = (4.0, 10.0)    # |x| range where the twin saddle / lid bar touch each motor's inner end (motor ends at |x| 4)
PETG_E = 2000.0            # MPa, printed PETG (ASSUMED)
SADDLE_X, SADDLE_C = SUPPORT_X, PAD_T  # rev 3: the cradle calc's support is the twin saddle at the motor's inner end

# Fastener sizes. M3 where the drop / weapon loads go (cradles, side blocks, wedgelet rail); M2.5 for the lightly
# loaded cleats and skirt brackets (CALC_PC_BODY.md section 6 has the load on every joint).
SIZES = {
    'M3': {'clear_r': 1.7, 'insert_r': fit(config, 'insert_hole_diameter_mm', design['insert_trial_hole_diameter_mm']) / 2,
           'insert_depth': value(config, 'insert_m3', 'length_mm') + 1, 'csk_d': 6.3, 'head_d': 5.7, 'head_k': 1.65,
           'insert': 'M3_heatset_insert_L4_OD5'},
    'M2.5': {'clear_r': 1.45, 'insert_r': 1.6, 'insert_depth': 5.0, 'csk_d': 5.4, 'head_d': 4.7, 'head_k': 1.4,
             'insert': 'M2.5_heatset_insert_L4_OD3.6'},     # insert hole D3.2 chosen 2026-10-19 (0.2 mm bite per side on OD 3.6), not coupon-tested
}
INSERT_R = SIZES['M3']['insert_r']
INSERT_DEPTH = SIZES['M3']['insert_depth']
# g each. M3 ISO 7380 from the ISO 7380-1 mass table (0.42 / 0.59 g per piece, fasten.it); the rest estimated from
# steel/brass volume (ISO 10642 M3x6 0.36, M2.5 button 0.30, M2.5 csk 0.26) or a 100-pc listing (M3 insert 0.33).
# All still to be weighed: release blocker in the work order.
FASTENER_G = {'M3x6_button_ISO7380': 0.42, 'M3x10_button_ISO7380': 0.59, 'M3x6_csk_ISO10642': 0.36,
              'M2.5x6_button': 0.30, 'M2.5x6_csk_ISO10642': 0.26,
              'M3_heatset_insert_L4_OD5': 0.33, 'M2.5_heatset_insert_L4_OD3.6': 0.18, 'EPDM_pad_0.5mm': 0.08}
SCREW_GEOM = {'M3x6_button_ISO7380': (6.0, 'button', 'M3'), 'M3x10_button_ISO7380': (10.0, 'button', 'M3'), 'M3x6_csk_ISO10642': (6.0, 'csk', 'M3'),
              'M2.5x6_button': (6.0, 'button', 'M2.5'), 'M2.5x6_csk_ISO10642': (6.0, 'csk', 'M2.5')}

# Material data for the stiffness sheet (ASSUMED handbook values, grade not yet chosen)
PC_E, PC_NU, PC_YIELD = 2350.0, 0.37, 62.0
TPU_E, TPU_NU = 50.0, 0.45                              # same TPU 95A assumption as body_material_calc.py
PETG_COMP_YIELD = (40.0, 50.0)                          # printed PETG, MPa (ASSUMED range)
TPU_BEARING = (5.0, 10.0)                               # CALC_R3 section 7 assumption, MPa


def cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def signed_area(pts):
    return 0.5 * sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1] for i in range(len(pts)))


def filleted(pts, radius, cutout=False):
    """Closed polygon -> (start, segments) with arcs on the corners that are concave seen from the material.
    Segments: ('L', end) or ('A', mid, end, bulge)."""
    ccw = signed_area(pts) > 0
    n, corners = len(pts), []
    for i in range(n):
        a, p, b = (np.array(pts[k], float) for k in (i - 1, i, (i + 1) % n))
        turn = cross2(p - a, b - p)
        convex = (turn > 0) == ccw
        if abs(turn) < 1e-9 or (convex if not cutout else not convex):
            corners.append((p, None))
            continue
        d1, d2 = (a - p) / np.linalg.norm(a - p), (b - p) / np.linalg.norm(b - p)
        half = math.acos(max(-1.0, min(1.0, float(np.dot(d1, d2))))) / 2
        t = min(radius / math.tan(half), np.linalg.norm(a - p) / 2, np.linalg.norm(b - p) / 2)
        r = t * math.tan(half)
        t1, t2 = p + d1 * t, p + d2 * t
        bis = (d1 + d2) / np.linalg.norm(d1 + d2)
        c = p + bis * (r / math.sin(half))
        mid = c + (p - c) / np.linalg.norm(p - c) * r
        bulge = math.tan((math.pi - 2 * half) / 4) * (1 if cross2(t1 - c, t2 - c) > 0 else -1)
        corners.append((p, (t1, mid, t2, bulge)))
    start, segs = None, []
    for p, arc in corners:
        first = arc[0] if arc else p
        if start is None: start = first
        else: segs.append(('L', first))
        if arc: segs.append(('A', arc[1], arc[2], arc[3]))
    segs.append(('L', start))
    return tuple(float(c) for c in start), [(s[0],) + tuple(tuple(float(c) for c in v) if isinstance(v, np.ndarray) else v for v in s[1:]) for s in segs]


def rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


class Panel:
    """Flat sheet: world P = O + u*U + v*V + w*N with V = N x U and w in 0..T. (u, v) is the DXF plane as seen
    from the 'face up' side (N points at the viewer). Countersinks are cut on that face."""

    def __init__(self, name, origin, U, N, face_up, outline, cutouts=(), pockets=(), note='', t=T):
        self.name, self.O, self.U, self.N = name, np.array(origin, float), np.array(U, float), np.array(N, float)
        self.t = t
        self.V = np.cross(self.N, self.U)
        self.face_up, self.note = face_up, note
        self.outline = [self.uv(p) for p in outline]
        self.cutouts = [[self.uv(p) for p in c] for c in cutouts]
        self.pockets = [([self.uv(p) for p in c], depth, face) for c, depth, face in pockets]   # face: 'up' or 'down'
        self.holes = []                                   # (u, v, r, countersink diameter or None)

    def uv(self, p):
        d = np.array(p, float) - self.O
        return (round(float(np.dot(d, self.U)), 4), round(float(np.dot(d, self.V)), 4))

    def world(self, u, v, w=0.0):
        return self.O + u * self.U + v * self.V + w * self.N

    def add_hole(self, p, r, csk=None):
        u, v = self.uv(p)
        if any(abs(u - h[0]) < 1e-6 and abs(v - h[1]) < 1e-6 for h in self.holes): return
        self.holes.append((u, v, r, csk))

    def prism(self, pts, cutout, w0, w1):
        start, segs = filleted(pts, CORNER_R, cutout)
        plane = cq.Plane(origin=tuple(self.world(0, 0, w0)), xDir=tuple(self.U), normal=tuple(self.N))
        wp = cq.Workplane(plane).moveTo(*start)
        for s in segs:
            wp = wp.lineTo(*s[1]) if s[0] == 'L' else wp.threePointArc(s[1], s[2])
        return wp.close().extrude(w1 - w0).val()

    def solid(self):
        T = self.t
        shape = self.prism(self.outline, False, 0, T)
        for c in self.cutouts:
            shape = shape.cut(self.prism(c, True, -1, T + 1))
        for c, depth, face in self.pockets:
            w0, w1 = (-1, depth) if face == 'down' else (T - depth, T + 1)
            shape = shape.cut(self.prism(c, True, w0, w1))
        for u, v, r, csk in self.holes:
            shape = shape.cut(cq.Solid.makeCylinder(r, T + 2, cq.Vector(*self.world(u, v, -1)), cq.Vector(*self.N)))
            if csk:   # 90 deg cone, diameter csk at the face-up surface
                shape = shape.cut(cq.Solid.makeCone(0, csk / 2 + 1, csk / 2 + 1,
                                                    cq.Vector(*self.world(u, v, T - csk / 2)), cq.Vector(*self.N)))
        return shape.clean()


# ---------------------------------------------------------------- R3 geometry, TPU tub + lid removed
root, constraints = build(config)
r3_mass = mass_properties(root, config, {})
tpu_group = next(c for c in root.children if c.name == '01_Printed_Chassis_TPU')
root.children.remove(tpu_group)
tpu_rows = [r for r in r3_mass['parts'] if r['part'] in ('Tub_TPU', 'Lid_TPU_2mm')]
tpu_body_g = sum(r['mass_g'] for r in tpu_rows)

# Re-issued R3 prints.
# The R3 hall post is used as is (ends at y 61.5, bulkhead starts at 62); its screw goes through the floor here.
HALL_HOLE = (15.0, (MOTOR_Y + 8 + 61.5) / 2)
variant_bodies = []

body_grp = group(root, '01_PC_Body_PETG_parts')
new_bodies, fasteners = [], []
panel_holes = {k: [] for k in ('floor', 'wall_L', 'wall_R', 'rear', 'bulk_L', 'bulk_R', 'lid')}
PANEL_T = {k: (T_BULK if k.startswith('bulk') else T) for k in panel_holes}


def add_body(name, shape, material='PETG'):
    comp = part(body_grp, name)
    comp.body = Body(shape.clean(), name, material, comp)
    new_bodies.append(comp.body)
    return comp.body


def joint(target, panel_key, face_point, direction, label, size='M3', head='button', drill=True, group=None):
    """Heat-set insert in `target` from face_point along `direction`; clearance hole in the panel; screw head
    on the far side of the panel. `group` ties the joint to a load path in the safety-factor table."""
    z = SIZES[size]
    if drill:
        bore(target, face_point, direction, z['insert_r'], 0, z['insert_depth'])
    d = np.array(direction, float)
    panel_holes[panel_key].append((tuple(face_point), z['clear_r'], z['csk_d'] if head == 'csk' else None))
    hp = np.array(face_point, float) - d * PANEL_T[panel_key]
    screw = '%sx6_%s' % (size, 'button' + ('_ISO7380' if size == 'M3' else '') if head == 'button' else 'csk_ISO10642')
    fasteners.append({'kind': screw, 'label': label, 'pos': [round(float(c), 3) for c in hp + d * 2], 'size': size,
                      'axis': d.tolist(), 'head_point': hp.tolist(), 'host': target.name, 'panel': panel_key, 'group': group})
    fasteners.append({'kind': z['insert'], 'label': label, 'pos': [round(float(c), 3) for c in np.array(face_point) + d * 2],
                      'host': target.name, 'group': group})
    return {'point': tuple(face_point), 'dir': tuple(direction), 'r': z['insert_r'], 'depth': z['insert_depth']}


def pad(label, pos):
    fasteners.append({'kind': 'EPDM_pad_0.5mm', 'label': label, 'pos': [round(float(c), 3) for c in pos]})


# Wedgelet blocks: the rail (x 70..80, y 64..98, z 7..18) now also screws down into the PC floor wing.
for side, sgn in (('L', -1), ('R', 1)):
    b = find_body(root, 'Wedgelet_Block_PETG_' + side)
    b.name = 'Wedgelet_Block_PETG_%s_PCbody' % side
    variant_bodies.append(b)
    for y in (70.0, 86.0):
        joint(b, 'floor', (sgn * 75, y, 7), (0, 0, 1), 'wedgelet rail %s floor' % side, group='rail_floor')

# 1) drive-motor mounts, rev 3. Each JGA25 is held at both ends:
#    * face: 4 mm PETG wall at |x| 56..60 (motor face screws, as the tub's motor walls), 6 mm foot, 2 M3 inserts
#      under the wall root
#    * inner end (|x| 4..10): a twin saddle on the floor under both motors of an axle, and a twin bar under the lid
#      above them, both with a 0.5 mm EPDM pad in the cup. Wires leave the motor end caps through the x +-4 window.
#    Upright landing: inner ends press on the saddle -> floor (ground-backed after 4 mm). Inverted landing: they press
#    on the lid bar -> lid (ground-backed after 2 mm). The wall root only carries the face shear (CALC section 10).
for loc, wy in (('Front', FRONT_Y), ('Rear', REAR_Y)):
    r_motor = value(config, 'drive_%s_L' % loc, 'diameter_mm') / 2
    for side, sgn in (('L', -1), ('R', 1)):
        name = 'drive_%s_%s' % (loc, side)
        fl = (loc, side) == ('Front', 'L')                # FL foot stops at y 21.5: weapon-mount foot 1 sits at y 22..35
        y0, y1 = wy - 13, wy + 13
        shape = block(*sorted((sgn * 56, sgn * 60)), y0, y1, 7, CRADLE_TOP).fuse(
            block(*sorted((sgn * CRADLE_FOOT, sgn * 56)), y0, 21.5 if fl else y1, 7, 13))
        c = add_body('Motor_Cradle_PETG_%s_%s' % (loc, side), shape)
        xa, xb = sorted((sgn * 55, sgn * 61))
        bore(c, (xa, wy, 32), (1, 0, 0), max(value(config, name, 'boss_diameter_mm') / 2 + 0.25, 4.0), 0, xb - xa)
        # counterbore for the wheel-hub nose (revision.py: the nose starts where the shaft's D-flat starts)
        nose = 56 + max(value(config, name, 'boss_projection_mm') + 0.3,
                        value(config, name, 'shaft_projection_mm') - value(config, name, 'shaft_flat_length_mm') + 0.2)
        ca, cb = sorted((sgn * (nose - 0.3), sgn * 61))
        bore(c, (ca, wy, 32), (1, 0, 0), HUB_NOSE_R + 0.5, 0, cb - ca)
        for dy, dz in value(config, name, 'mount_holes_yz_mm'):
            bore(c, (xa, wy + dy, 32 + dz), (1, 0, 0), value(config, name, 'mount_thread_diameter_mm') / 2 + 0.2, 0, xb - xa)
            ha, hb = sorted((sgn * 59, sgn * 61))
            bore(c, (ha, wy + dy, 32 + dz), (1, 0, 0), 3.75, 0, hb - ha)     # button head + washer recess, as R3
        for y in ((9.5, 17.5) if fl else (wy - 8, wy + 8)):
            joint(c, 'floor', (sgn * CRADLE_INSERT_X, y, 7), (0, 0, 1), 'cradle %s %s' % (loc, side), group='cradle_floor')
    cup = prism([('circle', (wy, 32, r_motor + PAD_T))], 'x', -SUPPORT_X[1] - 1, SUPPORT_X[1] + 1)
    window_lo = block(-SUPPORT_X[0], SUPPORT_X[0], wy - 14, wy + 14, 15, 32)
    sad = block(-SUPPORT_X[1], SUPPORT_X[1], wy - 11, wy + 11, 7, 26).cut(cup).cut(window_lo)
    c = add_body('Motor_Twin_Saddle_PETG_%s' % loc, sad)
    for y in (wy - 6, wy + 6):
        joint(c, 'floor', (0, y, 7), (0, 0, 1), 'twin saddle %s' % loc, size='M2.5', group='saddle_floor')
    window_hi = block(-SUPPORT_X[0], SUPPORT_X[0], wy - 5, wy + 5, 40, 52)
    bar = block(-SUPPORT_X[1], SUPPORT_X[1], wy - 4, wy + 4, 40, TOP_Z).cut(cup).cut(window_hi)
    c = add_body('Motor_Lid_Bar_PETG_%s' % loc, bar)
    for x in (-7.0, 7.0):
        joint(c, 'lid', (x, wy, TOP_Z), (0, 0, -1), 'lid bar %s' % loc, size='M2.5', head='csk', group='lid_bar')
    for x in (-7.0, 7.0):
        pad('saddle pad %s' % loc, (x, wy, 32 - r_motor)); pad('lid bar pad %s' % loc, (x, wy, 32 + r_motor))

# 2) weapon side blocks: 6 mm web against the upright (4 M3 x 10 into the upright's taps, heads seated at |x| 38),
#    6 mm floor flange with 4 M3 inserts, 5 mm bulkhead flange with 2. R3 TPU block: solid 16 x 32 x 43 mm.


def upright_taps(body):
    """(y, z) of the M3 tap holes through an upright, read from the R3 CAD (tracks any change to the upright)."""
    taps = []
    for f in body.shape.Faces():
        if f.geomType() != 'CYLINDER': continue
        cyl = BRepAdaptor_Surface(f.wrapped).Cylinder()
        ax = cyl.Axis().Direction()
        if abs(abs(ax.X()) - 1) > 1e-6 or abs(cyl.Radius() - 1.25) > 0.01: continue
        c = cyl.Location()
        p = (round(c.Y(), 3), round(c.Z(), 3))
        if p not in taps: taps.append(p)
    return sorted(taps)


UPRIGHT_SCREWS = upright_taps(find_body(root, 'Upright_6061_6mm_R'))
if len(UPRIGHT_SCREWS) != 4 or upright_taps(find_body(root, 'Upright_6061_6mm_L')) != UPRIGHT_SCREWS:
    raise ValueError('expected 4 matching side taps in each upright, got %s' % UPRIGHT_SCREWS)
SIDE_BLOCK_FLOOR_Y = (67.5, 76.0, 84.5, 92.5)
for side, sgn in (('L', -1), ('R', 1)):
    def xr(a, b): return sorted((sgn * a, sgn * b))
    shape = block(*xr(34, 40), 64, 96, 7, 50).fuse(block(*xr(40, 50), 64, 96, 7, 13), block(*xr(40, 55), 64, 69, 7, 50))
    c = add_body('Weapon_Side_Block_PETG_' + side, shape)
    for y, z in UPRIGHT_SCREWS:
        bore(c, (sgn * 41, y, z), (-sgn, 0, 0), SIZES['M3']['clear_r'], 0, 8)
        bore(c, (sgn * 41, y, z), (-sgn, 0, 0), 3.25, 0, 3)
    a, b = xr(33, 56)
    bore(c, (a, 95, 32), (1, 0, 0), 10.5, 0, b - a)                       # M8 socket access, as R3
    if sgn < 0:                                                           # weapon-motor can reaches y 65: R3 tub relief
        bore(c, (-42, MOTOR_Y, MOTOR_Z), (1, 0, 0), value(config, 'weapon_motor', 'diameter_mm') / 2 + 1, 0,
             value(config, 'weapon_motor', 'length_mm') + 2)
    for y, z in UPRIGHT_SCREWS:                                           # M3x10 button, head seated at |x| 38
        fasteners.append({'kind': 'M3x10_button_ISO7380', 'label': 'upright %s y%g z%g' % (side, y, z), 'pos': [sgn * 33, y, z],
                          'axis': [-sgn, 0, 0], 'head_point': [sgn * 38, y, z], 'host': 'Upright_6061_6mm_' + side, 'group': 'upright_web',
                          'counted_in_R3_allowance': True})
    for y in SIDE_BLOCK_FLOOR_Y:
        joint(c, 'floor', (sgn * 45, y, 7), (0, 0, 1), 'side block %s floor' % side, group='side_block_floor')
    for z in (28, 45):
        joint(c, 'bulk_' + side, (sgn * 51, 64, z), (0, 1, 0), 'side block %s bulkhead' % side, group='side_block_bulkhead')

# 3) joining cleats with M2.5 inserts (D3.2 holes, >= 1.2 mm PETG everywhere). R side given, L mirrored. Low
#    cleats stay under z 19 and high ones above z 47: the tyre band. Faces: (panel, point on the face, direction).
CLEATS = [
    ('Rear_Low', (69, 80, -107, -100.5, 7, 18), [('wall', (80, -103.75, 15), (-1, 0, 0)), ('rear', (72, -107, 15), (0, 1, 0)), ('floor', (72, -103.75, 7), (0, 0, 1))]),
    ('Rear_High', (69, 80, -107, -100.5, 48, 59), [('wall', (80, -103.75, 51), (-1, 0, 0)), ('rear', (72, -107, 51), (0, 1, 0)), ('lid', (72, -103.75, 59), (0, 0, -1))]),
    ('Mid_Low', (71, 80, -26, -14, 7, 16), [('wall', (80, -17, 11.5), (-1, 0, 0)), ('floor', (74, -23, 7), (0, 0, 1))]),
    ('Mid_High', (71, 80, -26, -14, 48, 59), [('wall', (80, -17, 51.5), (-1, 0, 0)), ('lid', (74, -23, 59), (0, 0, -1))]),
    ('Front_Low', (71, 80, 50.5, BULK_Y0, 7, 16), [('wall', (80, 54, 11.5), (-1, 0, 0)), ('floor', (74, 59, 7), (0, 0, 1))]),
    ('Front_High', (69, 80, 50.5, BULK_Y0, 48, 59), [('wall', (80, 53.75, 51), (-1, 0, 0)), ('bulk', (72, BULK_Y0, 51), (0, -1, 0)), ('lid', (72, 53.75, 59), (0, 0, -1))]),
]
cleat_inserts = {}
for name, (x0, x1, y0, y1, z0, z1), faces in CLEATS:
    for side, sgn in (('L', -1), ('R', 1)):
        c = add_body('Cleat_%s_PETG_%s' % (name, side), block(*sorted((sgn * x0, sgn * x1)), y0, y1, z0, z1))
        cleat_inserts[c.name] = []
        for panel, (px, py, pz), (dx, dy, dz) in faces:
            key = {'wall': 'wall_' + side, 'bulk': 'bulk_' + side}.get(panel, panel)
            cleat_inserts[c.name].append(joint(c, key, (sgn * px, py, pz), (sgn * dx, dy, dz), 'cleat %s %s' % (name, side),
                                               size='M2.5', head='csk' if panel == 'lid' else 'button', group='cleat_' + panel))
for name, box_, faces in [
        ('Rear_Centre_Low', (8, 19, -107, -100.5, 7, 18), [('rear', (16, -107, 15), (0, 1, 0)), ('floor', (11, -103.75, 7), (0, 0, 1))]),
        ('Bulkhead_Centre_Low', (-17, -6, 50.5, BULK_Y0, 7, 18), [('bulk_R', (-9, BULK_Y0, 15), (0, -1, 0)), ('floor', (-14, 53.75, 7), (0, 0, 1))])]:
    c = add_body('Cleat_%s_PETG' % name, block(*box_))
    cleat_inserts[c.name] = [joint(c, panel, p, d, 'cleat ' + name, size='M2.5', group='cleat_' + panel.split('_')[0]) for panel, p, d in faces]

# 4) skirt brackets as separate PETG parts. Same 35 deg seat as the tub-printed ones (R3 skirt plates and their
#    M2 holes unchanged). Side: base u 83..88 carries one M2.5 insert. Rear: base above the plate top (z 16.5).
S35, C35 = math.sin(math.radians(35)), math.cos(math.radians(35))
SEAT_LOW = 90.5 + 6.8 / C35 * S35                                   # 95.26, R3 bracket bottom corner
SIDE_PROFILE = [(83, 8), (88, 8), (88, 10.5), (SEAT_LOW, 10.5), (90.5, 17.3), (88, 17.3), (88, 24), (83, 24)]
REAR_PROFILE = [(88, 10.5), (SEAT_LOW, 10.5), (90.5, 17.3), (93, 17.3), (93, 24), (88, 24)]
HOLE_UZ = (90.5 + 4.0 * S35, 16.8 - 4.0 * C35 + 0.5)               # R3 M2 pilot centre on the seat
# (bracket y range, R3 skirt-hole centre, wall screw (y, z)). Screws avoid the switch opening, cleats, bulkhead end and rail.
SIDE_CLIPS = [((-90, -75), -82.5, (-82.5, 20.5)), ((-35, -20), -27.5, (-31, 13)), ((25, 40), 32.5, (32.5, 20.5)), ((59, 72), 62.5, (67.5, 20.5))]
REAR_CLIPS = [((-65, -50), -57.5), ((-8, 8), 0.0), ((50, 65), 57.5)]
for side, sgn in (('L', -1), ('R', 1)):
    for (c0, c1), cy, (sy, sz) in SIDE_CLIPS:
        c = add_body('Skirt_Bracket_PETG_%s_y%+d' % (side, round(cy)), prism([('poly', [(sgn * u, z) for u, z in SIDE_PROFILE])], 'y', c0, c1))
        bore(c, (sgn * HOLE_UZ[0], cy, HOLE_UZ[1]), (sgn * C35, 0, S35), 0.8, 4.0, 4.0)
        # countersunk M2.5 from inside the robot (flush: 4 of these sit beside a tyre)
        joint(c, 'wall_' + side, (sgn * WALL_OUT, sy, sz), (sgn, 0, 0), 'skirt bracket %s' % side, size='M2.5', head='csk', group='skirt')
for (c0, c1), cx in REAR_CLIPS:
    c = add_body('Skirt_Bracket_PETG_Rear_x%+d' % round(cx), prism([('poly', [(-(u + 22), z) for u, z in REAR_PROFILE])], 'x', c0, c1))
    bore(c, (cx, -(HOLE_UZ[0] + 22), HOLE_UZ[1]), (0, -C35, S35), 0.8, 4.0, 4.0)
    joint(c, 'rear', (cx, REAR_OUT, 20.5), (0, -1, 0), 'skirt bracket rear', size='M2.5', group='skirt')

# ---------------------------------------------------------------- PC panels
batt = value(config, 'battery', 'dimensions_mm')
battery_y = -55.5 + batt[1] / 2
r3_floor_holes = [((x, y, 7), HOLE_R) for x in (-batt[0] / 2 - 2.5, batt[0] / 2 + 2.5)
                  for y in (battery_y - batt[1] / 2 + 3, battery_y + batt[1] / 2 - 3)]          # battery tray
r3_floor_holes += [((x, y, 7), HOLE_R) for x in (-57, 57) for y in (-52, -8)]                  # electronics standoffs
r3_floor_holes += [((-49, fy, 7), HOLE_R) for fy in (25, 59)]                                  # weapon-mount feet
r3_floor_holes += [((HALL_HOLE[0], HALL_HOLE[1], 7), SELF_TAP_R)]                              # hall post

f0, f1 = FRONT_Y - WELL_FLOOR, FRONT_Y + WELL_FLOOR
r0, r1 = REAR_Y - WELL_FLOOR, REAR_Y + WELL_FLOOR
floor_xy = [(-80, -107), (80, -107), (80, r0), (60, r0), (60, r1), (80, r1), (80, f0), (60, f0), (60, f1), (80, f1),
            (80, 100), (35, 100), (35, 64), (-35, 64), (-35, 100), (-80, 100), (-80, f1), (-60, f1), (-60, f0), (-80, f0),
            (-80, r1), (-60, r1), (-60, r0), (-80, r0)]
panels = {}
panels['floor'] = Panel('PC3_Floor', (0, 0, FLOOR_Z0), (1, 0, 0), (0, 0, 1), 'top (inside the robot)',
                        [(x, y, FLOOR_Z0) for x, y in floor_xy], note='wheel notches at the sides; centre gap |x|<35 in front of y 64 is for the bottom brace (x +-33)')
for side, sgn in (('L', -1), ('R', 1)):
    wall = [(sgn * WALL_OUT, y, z) for y, z in rect(REAR_OUT, WALL_FRONT, FLOOR_Z0, TOP_Z)]
    cut = []
    if side == 'R':          # removable-link opening, as the R3 tub (link 14 x 8 x 10 nominal)
        dd = value(config, 'disconnect', 'dimensions_mm')
        cut = [[(WALL_OUT, y, z) for y, z in rect(-30 - dd[1] / 2 - 0.5, -30 + dd[1] / 2 + 0.5, 20 - 0.5, 20 + dd[2] + 0.5)]]
    panels['wall_' + side] = Panel('PC3_Side_Wall_' + side, (sgn * WALL_OUT, 0, 0), (0, -sgn, 0), (-sgn, 0, 0),
                                   'inner face (countersinks)', wall, cut)
panels['rear'] = Panel('PC3_Rear_Wall', (0, REAR_OUT + T, 0), (1, 0, 0), (0, -1, 0), 'outer face (rear)',
                       [(x, REAR_OUT, z) for x, z in rect(-WALL_IN, WALL_IN, FLOOR_Z0, TOP_Z)])
panels['bulk_L'] = Panel('PC2_Bulkhead_L', (0, BULK_Y0, 0), (-1, 0, 0), (0, 1, 0), 'front face (toward the weapon)',
                         [(x, BULK_Y0, z) for x, z in rect(-WALL_IN, -45.5, 7, TOP_Z)], t=T_BULK)
# R: motor-can notch (relief r = can + 1 mm crosses y 62..64 between z 31.0 and 54.0) + belt/pulley slot x +-6
panels['bulk_R'] = Panel('PC2_Bulkhead_R', (0, BULK_Y0, 0), (-1, 0, 0), (0, 1, 0), 'front face (toward the weapon)',
                         [(x, BULK_Y0, z) for x, z in [(-41.5, 7), (WALL_IN, 7), (WALL_IN, TOP_Z), (6, TOP_Z), (6, 12), (-6, 12), (-6, 29.5), (-41.5, 29.5)]], t=T_BULK)
lid_xy = [(-WALL_OUT, REAR_OUT), (WALL_OUT, REAR_OUT), (WALL_OUT, 64), (3.5, 64), (3.5, 44), (-3.5, 44), (-3.5, 64), (-WALL_OUT, 64)]
wells = [rect(*xs, wy - WELL_LID, wy + WELL_LID) for wy in WHEEL_Y for xs in ((60, 80), (-80, -60))]
mr = value(config, 'weapon_motor', 'diameter_mm') / 2
motor_x1 = -41 + value(config, 'weapon_motor', 'length_mm')
half = math.sqrt((mr + 1) ** 2 - (TOP_Z - MOTOR_Z) ** 2)          # can + 1 mm crosses the lid underside over |dy| < half
pocket = rect(-42, motor_x1 + 1.5, MOTOR_Y - half - 0.5, MOTOR_Y + half + 0.5)
panels['lid'] = Panel('PC3_Lid', (0, 0, TOP_Z), (1, 0, 0), (0, 0, 1), 'top (countersinks); motor pocket on the underside',
                      [(x, y, TOP_Z) for x, y in lid_xy], [[(x, y, TOP_Z) for x, y in w] for w in wells],
                      [([(x, y, TOP_Z) for x, y in pocket], LID_POCKET_DEPTH, 'down')],
                      note='belt slot x +-3.5 open at the front edge; %.1f mm deep pocket from the underside over the weapon motor' % LID_POCKET_DEPTH)
for key, holes in panel_holes.items():
    for p, r, csk in holes: panels[key].add_hole(p, r, csk)
for p, r in r3_floor_holes: panels['floor'].add_hole(p, r)
for side, sgn in (('L', -1), ('R', 1)):
    for y, z in ((80, 13.5), (92, 13.5)):                          # R3 wedgelet-rail wall screws
        panels['wall_' + side].add_hole((sgn * WALL_OUT, y, z), HOLE_R)
dd = value(config, 'disconnect', 'dimensions_mm')
for y in (-30 - dd[1] / 2 - 2, -30 + dd[1] / 2 + 2):              # R3 switch-collar screws (M2.5 self-tapping)
    panels['wall_R'].add_hole((WALL_OUT, y, 20 + dd[2] / 2), SELF_TAP_R)

pc_group = group(root, '01_PC_Body_Panels')
pc_bodies = {}
for key, p in panels.items():
    comp = part(pc_group, p.name)
    comp.body = Body(p.solid(), p.name, 'PC', comp)
    pc_bodies[key] = comp.body

# ---------------------------------------------------------------- checks
def bbox_overlap(a, b, tol=0.01):
    A, B = a.BoundingBox(), b.BoundingBox()
    return A.xmin < B.xmax - tol and B.xmin < A.xmax - tol and A.ymin < B.ymax - tol and B.ymin < A.ymax - tol and A.zmin < B.zmax - tol and B.zmin < A.zmax - tol


all_bodies = [b for b in root.bodies()]
checked = list(pc_bodies.values()) + new_bodies + variant_bodies
interference = []
for i, a in enumerate(checked):
    for o in all_bodies:
        if o is a or (o in checked and checked.index(o) < i): continue
        if not bbox_overlap(a.shape, o.shape): continue
        v = a.shape.intersect(o.shape).Volume()
        if v > 0.01:
            interference.append({'a': a.name, 'b': o.name, 'volume_mm3': round(v, 3), 'b_material': o.material})

# Screw envelopes: shank D3 over the full length + button head D5.7 x 1.65. Zero overlap with any body means the
# panel holes and insert holes line up and every head has room.
envelope_hits = []
for f in fasteners:
    if 'axis' not in f: continue
    L, head, size = SCREW_GEOM[f['kind']]
    z = SIZES[size]
    hp, ax = cq.Vector(*f['head_point']), cq.Vector(*f['axis'])
    env = cq.Solid.makeCylinder(z['clear_r'] - 0.25, L - 0.05, hp + ax * 0.05, ax)
    if head == 'button':
        env = env.fuse(cq.Solid.makeCylinder(z['head_d'] / 2, z['head_k'], hp - ax * (z['head_k'] + 0.05), ax))
    for o in all_bodies:
        if o.name == f.get('host', '') and o.name.startswith('Upright'): continue   # threads into the upright's tap
        if not bbox_overlap(env, o.shape, 0): continue
        v = env.intersect(o.shape).Volume()
        if v > 0.01:
            envelope_hits.append({'screw': f['label'], 'kind': f['kind'], 'head_point': f['head_point'], 'body': o.name, 'volume_mm3': round(v, 3)})

# Material between heat-set inserts in the same cleat (insert hole D, depth INSERT_DEPTH)
insert_gaps = []
for name, ins in cleat_inserts.items():
    cyl = [cq.Solid.makeCylinder(j['r'], j['depth'], cq.Vector(*j['point']), cq.Vector(*j['dir'])) for j in ins]
    host = next(b for b in new_bodies if b.name == name)
    for i in range(len(cyl)):
        for j in range(i + 1, len(cyl)):
            insert_gaps.append({'cleat': name, 'pair': [i, j], 'gap_mm': round(cyl[i].distance(cyl[j]), 2)})
    for i, cy in enumerate(cyl):            # wall left to the cleat's outer faces (except the entry face)
        bb, cb = host.shape.BoundingBox(), cy.BoundingBox()
        p, d = ins[i]['point'], ins[i]['dir']
        sides = []
        for axis, lo, hi, clo, chi in (('x', bb.xmin, bb.xmax, cb.xmin, cb.xmax), ('y', bb.ymin, bb.ymax, cb.ymin, cb.ymax), ('z', bb.zmin, bb.zmax, cb.zmin, cb.zmax)):
            k = 'xyz'.index(axis)
            if abs(d[k]) > 0.5:
                sides.append(hi - chi if d[k] > 0 else clo - lo)   # bottom of the blind hole
            else:
                sides += [clo - lo, hi - chi]
        insert_gaps.append({'cleat': name, 'insert': i, 'min_outer_wall_mm': round(min(sides), 2)})

# ---------------------------------------------------------------- mass and balance
root.children.remove(pc_group)               # mass_properties has no PC density; panels are added below
mp = mass_properties(root, config, {})
root.children.append(pc_group)
rows = [dict(r) for r in mp['parts']]
pc_rows = []
for key, b in pc_bodies.items():
    vol = b.shape.Volume()
    pc_rows.append({'part': b.name, 'material': 'PC', 'mass_g': round(vol * PC_DENSITY, 4), 'volume_mm3': round(vol, 1),
                    'centroid_mm': list(b.shape.Center().toTuple()), 'method': 'CAD volume x 1.20 g/cm^3 (solid sheet, full density)'})
fast_rows = [{'part': 'PC body %s (%s)' % (f['kind'], f['label']), 'material': 'steel/brass', 'mass_g': FASTENER_G[f['kind']],
              'centroid_mm': f['pos'], 'method': 'ASSUMED catalogue mass, explicit position'} for f in fasteners if not f.get('counted_in_R3_allowance')]
rows += pc_rows + fast_rows
total = sum(r['mass_g'] for r in rows)
cg = sum(np.array(r['centroid_mm']) * r['mass_g'] for r in rows) / total
rear_load = (FRONT_Y - cg[1]) / (FRONT_Y - REAR_Y)
new_names = {b.name for b in new_bodies}
variant_names = {b.name for b in variant_bodies}
petg_new_g = sum(r['mass_g'] for r in rows if r['part'] in new_names)
pc_g = sum(r['mass_g'] for r in pc_rows)
fast_g = sum(r['mass_g'] for r in fast_rows)
r3_variant_g = sum(r['mass_g'] for r in r3_mass['parts'] if r['part'] in {n.replace('_PCbody', '') for n in variant_names})
pc_variant_g = sum(r['mass_g'] for r in rows if r['part'] in variant_names)
body_g = pc_g + petg_new_g + fast_g + (pc_variant_g - r3_variant_g)
rest_g = r3_mass['total_g'] - tpu_body_g
r3_by = {r['part']: r['mass_g'] for r in r3_mass['parts']}
mass_audit = [{'part': r['part'], 'r3_g': r3_by[r['part']], 'pc_body_g': r['mass_g']} for r in rows
              if r['part'] in r3_by and abs(r3_by[r['part']] - r['mass_g']) > 1e-3]
mass_audit += [{'part': n, 'r3_g': m, 'pc_body_g': None} for n, m in r3_by.items()
               if n not in {r['part'] for r in rows} and n not in ('Tub_TPU', 'Lid_TPU_2mm') and n.replace('_PCbody', '') == n
               and n + '_PCbody' not in {r['part'] for r in rows}]


def summary(total_g, cg_y):
    rl = (FRONT_Y - cg_y) / (FRONT_Y - REAR_Y)
    return {'total_g': round(total_g, 2), 'cg_y_mm': round(cg_y, 3), 'rear_load_pct': round(100 * rl, 3),
            'margin_to_2000_g': round(2000 - total_g, 2), 'margin_to_1900_g': round(1900 - total_g, 2),
            'rear_margin_to_10_pct_points': round(100 * rl - 10, 3), 'rear_gap_to_15_pct_points': round(15 - 100 * rl, 3),
            'passes_hard_2000g_10pct': bool(total_g <= 2000 and rl >= 0.10), 'passes_targets_1900g_15pct': bool(total_g <= 1900 and rl >= 0.15)}


def scenario(scale):
    """scale: dict group -> mass multiplier, CG positions unchanged."""
    m = {'pc': scale.get('pc', 1), 'petg': scale.get('petg', 1), 'fast': scale.get('fast', 1)}
    m['petg_small'] = scale.get('petg_small', m['petg'])
    tot, mom = 0.0, 0.0
    for r in rows:
        small = r['part'].startswith(('Cleat_', 'Skirt_Bracket', 'Motor_Twin_Saddle', 'Motor_Lid_Bar'))
        k = 'pc' if r in pc_rows else 'fast' if r in fast_rows else ('petg_small' if small else 'petg') if r['part'] in new_names else None
        w = r['mass_g'] * (m[k] if k else 1)
        tot += w; mom += w * r['centroid_mm'][1]
    return summary(tot, mom / tot)


# required CG shift for 15 %: CG_y <= FRONT_Y - 0.15 * (FRONT_Y - REAR_Y)
cg15 = FRONT_Y - 0.15 * (FRONT_Y - REAR_Y)
balance = {
    'r3_tpu': summary(r3_mass['total_g'], r3_mass['centroid_mm'][1]),
    'pc_body': summary(total, cg[1]),
    'cg_mm': cg.round(3).tolist(),
    'moment_to_reach_15pct_g_mm': round(total * (cg[1] - cg15), 1),
    # Cleats, skirt brackets, twin saddles and lid bars are small blocks around insert bosses: they print essentially solid.
    'expected_small_parts_solid': scenario({'petg_small': 1 / 0.8}),
    'sensitivity': {
        'small parts solid (cleats, skirt brackets, twin saddles, lid bars), cradles/side blocks 80 % (expected case)': scenario({'petg_small': 1 / 0.8}),
        'new PETG printed 100 % instead of the R3 80 % rule': scenario({'petg': 1 / 0.8}),
        'PC sheet 3.3 mm (+10 % thickness tolerance)': scenario({'pc': 1.1}),
        'PC density 1.22 g/cm^3': scenario({'pc': 1.22 / 1.20}),
        'fasteners +25 %': scenario({'fast': 1.25}),
        'all four together': scenario({'petg': 1 / 0.8, 'pc': 1.1 * 1.22 / 1.20, 'fast': 1.25}),
    },
}

# ---------------------------------------------------------------- stiffness / strength (per panel)
def D(E, nu, t): return E * t ** 3 / (12 * (1 - nu ** 2))


tpu_t = {'floor': (3.0, T), 'side walls': (8.0, T), 'rear wall': (6.0, T), 'bulkhead': (2.0, T_BULK), 'lid': (2.0, T)}
D_pc = D(PC_E, PC_NU, T)
stiff = {'D_PC3_N_mm': round(D_pc, 1), 'D_PC2_N_mm': round(D(PC_E, PC_NU, T_BULK), 1),
         'PC_yield_moment_N_mm_per_mm': {'3mm': round(PC_YIELD * T ** 2 / 6, 1), '2mm': round(PC_YIELD * T_BULK ** 2 / 6, 1)},
         'vs_R3_TPU': {k: {'tpu_t_mm': t, 'pc_t_mm': tp, 'D_TPU_N_mm': round(D(TPU_E, TPU_NU, t), 1), 'stiffness_ratio_PC/TPU': round(D(PC_E, PC_NU, tp) / D(TPU_E, TPU_NU, t), 1)} for k, (t, tp) in tpu_t.items()}}
# Lid: simply supported 160 x 168 mm plate (wall inner faces x +-80, rear wall y -107 to bulkhead y 61), central point load.
a_l, b_l = 2 * WALL_IN, BULK_Y0 + 107
alpha = 0.01160 + (0.01265 - 0.01160) * ((b_l / a_l) - 1.0) / 0.1                  # Timoshenko table, b/a 1.0..1.1
esp_top = max(b.shape.BoundingBox().zmax for b in all_bodies if b.name.startswith(('ESP32', 'Skywalker', 'DRV8871', 'IMU_GY')))
gap = TOP_Z - esp_top
stiff['lid_centre_point_load'] = {'plate_mm': [a_l, b_l], 'alpha': round(alpha, 5), 'deflection_per_100N_mm': round(alpha * 100 * a_l ** 2 / D_pc, 2),
                                  'gap_to_highest_electronics_mm': round(gap, 2), 'load_to_touch_electronics_N': round(gap * D_pc / (alpha * a_l ** 2), 0),
                                  'R3_TPU_2mm_deflection_per_100N_mm': round(alpha * 100 * a_l ** 2 / D(TPU_E, TPU_NU, 2.0), 0)}
# Screw heads on PC: bearing stress at assembly torque. Preload F = T / (K d), K 0.2.
head_area = {'button D5.7': math.pi / 4 * (5.7 ** 2 - 3.4 ** 2), 'csk 90deg D6.3': math.pi / 4 * (6.3 ** 2 - 3.4 ** 2) / math.sin(math.radians(45))}
stiff['screw_bearing_on_PC'] = {k: {'area_mm2': round(a, 1), **{'stress_MPa_at_%.1fNm' % tq: round(tq * 1000 / (0.2 * 3) / a, 1) for tq in (0.3, 0.5)}} for k, a in head_area.items()}
# Weapon upright joint: 4 M3 per side bearing on the block, CALC_R3 section 7 rated it 1.1-2.2 kN at the tooth with
# TPU 5-10 MPa over a 16 mm block. Same geometry factor, PETG web 6 mm at 40-50 MPa:
ratio = [(6 * p) / (16 * t) for p, t in zip(PETG_COMP_YIELD, TPU_BEARING)]
stiff['upright_joint_tooth_force_kN'] = {'R3_TPU_block': [1.1, 2.2], 'PETG_web_6mm': [round(1.1 * ratio[0], 1), round(2.2 * ratio[1], 1)],
                                          'next_weakest_R3_item': '608 bearing nearest the struck disc, 1.1 kN (CALC_R3 section 7)'}

# ---------------------------------------------------------------- impact load cases and safety factors
# Design target (CALC_PC_BODY.md section 10):
#   DT1  0.5 m drop, upright, flat: n = h / stopping distance. The skid touches 1 mm after the tyres, the floor
#        4 mm after; with ~1 mm of skid/structure compliance the stop is ~2 mm -> 250 g.
#   DT2  0.5 m drop, inverted: wheel tops first, the lid 2 mm later -> same 250 g, reversed.
#   DT3  weapon hit: CALC_R3 section 6 tooth force, calibrated half-sine, 1 ms contact 2.6 kN (design),
#        0.5 ms 5.2 kN (check only).
# The 4 mm drive shafts are the deliberate weak link: a body path that holds the shaft's plastic collapse load
# (upper-bound steel) plus its own parts' inertia cannot be the first thing to break.
g = 9.81
H_DROP, STOP = 500.0, 2.0
N_DROP = H_DROP / STOP
TOOTH_DESIGN, TOOTH_CHECK = 2600.0, 5200.0
SF_REQUIRED = 1.5
CAP = {  # capacities, ASSUMED until tested (work-order release blockers)
    'M3_insert_pullout_N': 500.0, 'M2.5_insert_pullout_N': 300.0, 'M2_selftap_PETG_pullout_N': 200.0,
    'PETG_bearing_MPa': PETG_COMP_YIELD[0], 'PETG_shear_MPa': 25.0, 'PC_bearing_MPa': PC_YIELD, 'PC_shear_MPa': 40.0,
    'EPDM_E_MPa': 5.0, 'shaft_yield_MPa': (355.0, 640.0), 'Al5052_yield_MPa': 193.0}


def d_shaft_moduli(d=4.0, flat_depth=0.5, n=400):
    """Elastic and plastic section moduli of a D-shaft, bending about the weaker axis (flat in compression/tension)."""
    r = d / 2
    ys = np.linspace(-r, r, n); dy = ys[1] - ys[0]
    best = None
    for flat_axis in ('y', 'z'):
        pts = []
        for y in ys:
            for z in ys:
                if y * y + z * z > r * r: continue
                if (y if flat_axis == 'y' else z) > r - flat_depth: continue
                pts.append(z if flat_axis == 'y' else y)
        zs = np.array(pts); A = dy * dy
        zc = zs.mean(); I = ((zs - zc) ** 2).sum() * A
        Zel = I / max(abs(zs.max() - zc), abs(zs.min() - zc))
        zm = np.median(zs); Zpl = np.abs(zs - zm).sum() * A
        if best is None or Zel < best[0]: best = (Zel, Zpl, flat_axis)
    return best


Zel, Zpl, _ = d_shaft_moduli(value(config, 'drive_Front_L', 'shaft_diameter_mm'), value(config, 'drive_Front_L', 'shaft_flat_depth_mm'))
x_face, x_wheel, x_motor_cg, x_support = 56.0, 70.0, 30.0, sum(SUPPORT_X) / 2
lever_wheel, lever_cg, d_support = x_wheel - x_face, x_face - x_motor_cg, x_face - x_support
F_shaft_yield = [round(sy * Zel / lever_wheel, 1) for sy in CAP['shaft_yield_MPa']]
F_cap = CAP['shaft_yield_MPa'][1] * Zpl / lever_wheel                  # upper bound the shaft can push into the body
m_motor = value(config, 'drive_Front_L', 'mass_g') / 1000
W_motor = m_motor * g * N_DROP
M_face = F_cap * lever_wheel + W_motor * lever_cg
R_support = M_face / d_support                                          # on the twin saddle (DT1) or the lid bar (DT2)
V_wall = F_cap + R_support - W_motor                                    # wall pulled up (DT1) / pushed down (DT2)
I_wall, Z_wall = 26 * 4 ** 3 / 12, 26 * 4 ** 2 / 6
k_wall = PETG_E * I_wall / ((32 - 13) * d_support ** 2)
pad_A = (SUPPORT_X[1] - SUPPORT_X[0]) * 15.0                            # pad contact per motor: 6 mm x ~15 mm arc
k_pad = CAP['EPDM_E_MPa'] * pad_A / PAD_T
wall_share = k_wall / (k_wall + k_pad)

mass_by = {r['part']: r for r in rows}
weapon_names = [n for n in mass_by if n.startswith(('Upright_', 'Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Nose_Skid', 'Beater_',
                'Bearing_608', 'Spacer_', 'Dead_Shaft', 'Weapon_Axle_', 'Belt_'))] + ['rotor screws']
m_weapon = sum(mass_by[n]['mass_g'] for n in weapon_names if n in mass_by) / 1000
M_kg = total / 1000
skid_y = 69.0
f_front = (cg[1] - REAR_Y) / (skid_y - REAR_Y)
S_dt1 = max(0.0, f_front * M_kg - m_weapon) * g * N_DROP / 2              # per side block, body -> weapon module
S_dt2 = m_weapon * g * N_DROP / 2                                       # inverted: weapon module hangs off the body
body_share = (M_kg - m_weapon) / M_kg
S_dt3 = TOOTH_DESIGN * body_share / 2
S_dt3c = TOOTH_CHECK * body_share / 2
floor_sys = ['3S_850mAh', 'Battery_Tray', 'Electronics_', 'Skywalker', 'ESP32', 'DRV8871', 'IMU_', 'D3536', 'Weapon_Motor_Mount',
             'Weapon_Mount_Foot', 'Motor_Pulley', 'Hall_', 'Power_Switch', 'Disconnect', 'wiring', 'straps', 'temperature']
m_floor_sys = (sum(r['mass_g'] for r in rows if r['part'].startswith(tuple(floor_sys))) + mass_by['PC3_Floor']['mass_g']) / 1000
low_cleat_shear_screws = [f for f in fasteners if 'axis' in f and f.get('group') in ('cleat_wall', 'cleat_rear', 'cleat_bulk') and f['pos'][2] < 30]
F_cleat = m_floor_sys * g * N_DROP / len(low_cleat_shear_screws)
skirt_mp = CAP['Al5052_yield_MPa'] * SKIRT_T ** 2 / 4                   # plastic moment of the 0.8 mm skirt, N mm/mm
skirt_lever = (HOLE_UZ[1] - 0.5) / C35
F_skirt = skirt_mp / skirt_lever * max(207 / len(SIDE_CLIPS), 176 / len(REAR_CLIPS))
rail_mass = sum(mass_by[n]['mass_g'] for n in mass_by if n.startswith(('Wedgelet_Block_PETG_R', 'Wedgelet_R', 'Wedgelet_Carrier_PETG_R', 'Hinge_Pin_D3_R'))) / 1000
F_rail = rail_mass * g * N_DROP / 4


def bearing(d, t, mpa): return d * t * mpa


def pull_through(head_d, hole_d, mpa): return math.pi / 4 * (head_d ** 2 - hole_d ** 2) * mpa


M3, M25 = SIZES['M3'], SIZES['M2.5']
paths = [
    # (path, case, load N, capacity N, note)
    ('Drive shaft (R3 bought part) elastic limit', 'DT1/DT2', F_cap, F_shaft_yield[0], 'weak link by design: bends first'),
    ('Cradle floor inserts M3 x2, tension', 'DT1', V_wall / 2, CAP['M3_insert_pullout_N'], 'insert pull-out ASSUMED'),
    ('PC floor under cradle screw heads, pull-through', 'DT1', V_wall / 2, pull_through(M3['head_d'], 2 * M3['clear_r'], CAP['PC_bearing_MPa']), ''),
    ('Motor face screws M3 x2, bearing in 4 mm PETG wall', 'DT1/DT2', V_wall / 2, bearing(3.0, 4.0, CAP['PETG_bearing_MPa']), ''),
    ('Cradle wall shear (26 x 4)', 'DT1/DT2', V_wall, 26 * 4 * CAP['PETG_shear_MPa'], ''),
    ('Cradle wall root bending (stress -> force-equivalent)', 'DT1/DT2', wall_share * M_face / Z_wall, CAP['PETG_bearing_MPa'], 'MPa vs MPa'),
    ('Twin saddle / lid bar contact (pad, per motor)', 'DT1/DT2', R_support / pad_A, CAP['PETG_bearing_MPa'], 'MPa vs MPa; then floor/lid ground-backed'),
    ('Side block web: 4 M3 into upright, PETG bearing, per side', 'DT1', S_dt1, 4 * bearing(3.0, 6.0, CAP['PETG_bearing_MPa']), ''),
    ('Side block floor inserts M3 x4 per side, tension', 'DT1', S_dt1 / 4, CAP['M3_insert_pullout_N'], ''),
    ('PC floor under side-block screw heads, pull-through', 'DT1', S_dt1 / 4, pull_through(M3['head_d'], 2 * M3['clear_r'], CAP['PC_bearing_MPa']), ''),
    ('Side block web, per side', 'DT2', S_dt2, 4 * bearing(3.0, 6.0, CAP['PETG_bearing_MPa']), ''),
    ('Side block floor inserts, per insert', 'DT2', S_dt2 / 4, CAP['M3_insert_pullout_N'], ''),
    ('Side block web, per side', 'DT3 2.6 kN', S_dt3, 4 * bearing(3.0, 6.0, CAP['PETG_bearing_MPa']), ''),
    ('Side block floor inserts, per insert (uplift)', 'DT3 2.6 kN', S_dt3 / 4, CAP['M3_insert_pullout_N'], ''),
    ('Side block floor screws M3 x4, PC 3 mm bearing (fore-aft shear)', 'DT3 2.6 kN', S_dt3 / 4, bearing(2 * M3['clear_r'], T, CAP['PC_bearing_MPa']), 'rearward push goes into the bulkhead in compression'),
    ('Low cleat wall screws M2.5, PC 3 mm bearing', 'DT2', F_cleat, bearing(2 * M25['clear_r'], T, CAP['PC_bearing_MPa']), 'floor + floor-mounted parts'),
    ('Low cleat bulkhead screw M2.5, PC 2 mm bearing', 'DT2', F_cleat, bearing(2 * M25['clear_r'], T_BULK, CAP['PC_bearing_MPa']), ''),
    ('Low cleat M2.5 insert, PETG bearing (OD3.6 x 4)', 'DT2', F_cleat, bearing(3.6, 4.0, CAP['PETG_bearing_MPa']), ''),
    ('Skirt bracket M2.5 insert (skirt plate collapse-limited)', 'DT1', F_skirt, CAP['M2.5_insert_pullout_N'], ''),
    ('Skirt plate M2 self-tapping screw into PETG bracket', 'DT1', F_skirt, CAP['M2_selftap_PETG_pullout_N'], ''),
    ('Wedgelet rail floor + wall screws, per screw', 'DT1', F_rail, CAP['M3_insert_pullout_N'], 'plates swing up freely'),
    ('Lid screws / lid-bar screws', 'DT1/DT2', 0.0, 1.0, 'compression only: lid presses onto walls / ground'),
]
check_paths = [
    ('Side block web, per side', 'DT3 5.2 kN check', S_dt3c, 4 * bearing(3.0, 6.0, CAP['PETG_bearing_MPa']), ''),
    ('Side block floor inserts, per insert', 'DT3 5.2 kN check', S_dt3c / 4, CAP['M3_insert_pullout_N'], ''),
]
sf_rows = []
for name, case, load, capa, note in paths + check_paths:
    sf = None if load <= 0 else capa / load
    sf_rows.append({'path': name, 'case': case, 'load': round(load, 1), 'capacity': round(capa, 1),
                    'sf': None if sf is None else round(sf, 2), 'note': note,
                    'design': case not in ('DT3 5.2 kN check',) and not name.startswith('Drive shaft')})
design_sf = [r['sf'] for r in sf_rows if r['design'] and r['sf'] is not None]
impact = {'N_drop_g': N_DROP, 'drop_mm': H_DROP, 'stop_mm': STOP,
          'n_g_for_stop_mm': {str(d): H_DROP / d for d in (1.0, 2.0, 4.0)},
          'shaft': {'Zel_mm3': round(Zel, 2), 'Zpl_mm3': round(Zpl, 2), 'lever_mm': lever_wheel, 'F_yield_N': F_shaft_yield,
                    'F_collapse_upper_N': round(F_cap, 1), 'static_front_wheel_N': round(M_kg * g * (1 - rear_load) / 2, 2)},
          'cradle': {'W_motor_N': round(W_motor, 1), 'M_face_Nmm': round(M_face, 0), 'R_support_N': round(R_support, 1), 'V_wall_N': round(V_wall, 1),
                     'k_wall_N_per_mm': round(k_wall, 2), 'k_pad_N_per_mm': round(k_pad, 0), 'wall_share': round(wall_share, 4)},
          'side_block': {'m_weapon_kg': round(m_weapon, 4), 'f_front_on_skid': round(f_front, 3), 'per_side_DT1_N': round(S_dt1, 1),
                         'per_side_DT2_N': round(S_dt2, 1), 'per_side_DT3_N': round(S_dt3, 1), 'per_side_DT3_check_N': round(S_dt3c, 1)},
          'cleat': {'m_floor_system_kg': round(m_floor_sys, 4), 'shear_screws': len(low_cleat_shear_screws), 'per_screw_N': round(F_cleat, 1)},
          'skirt': {'plastic_moment_Nmm_per_mm': round(skirt_mp, 2), 'lever_mm': round(skirt_lever, 2), 'per_bracket_N': round(F_skirt, 1)},
          'capacities_ASSUMED': CAP, 'sf_required': SF_REQUIRED, 'rows': sf_rows,
          'min_design_sf': round(min(design_sf), 2), 'min_design_path': next(r['path'] + ' / ' + r['case'] for r in sf_rows if r['design'] and r['sf'] == min(design_sf))}
stiff['impact'] = impact

# ---------------------------------------------------------------- exports
OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'PC_3mm').mkdir(exist_ok=True)
(OUT / 'PETG_STL').mkdir(exist_ok=True)
for stale in list((OUT / 'PC_3mm').glob('*')) + list((OUT / 'PETG_STL').glob('*')):
    stale.unlink()

assembly = cq.Assembly(name='V4_R3_PC_BODY')
for b in root.bodies():
    if b.material != 'keepout': assembly.add(b.shape, name=b.name)
assembly.save(str(OUT / 'ROBOT_V4_R3_PC_BODY.step'))


def dxf_entities(msp, p, du=0.0, dv=0.0, label=True):
    def loop(layer, pts, cutout):
        start, segs = filleted(pts, CORNER_R, cutout)
        verts = [[start[0] + du, start[1] + dv, 0.0]]
        for s in segs:
            if s[0] == 'A': verts[-1][2] = s[3]
            verts.append([s[-1][0] + du if s[0] == 'L' else s[2][0] + du, s[-1][1] + dv if s[0] == 'L' else s[2][1] + dv, 0.0])
        verts.pop()
        msp.add_lwpolyline([tuple(v) for v in verts], format='xyb', close=True, dxfattribs={'layer': layer})
    loop('CUT_OUTER', p.outline, False)
    for c in p.cutouts: loop('CUT_INNER', c, True)
    for c, depth, face in p.pockets: loop('POCKET_%.1fmm_%s' % (depth, 'UNDERSIDE' if face == 'down' else 'FACE_UP'), c, True)
    for u, v, r, csk in p.holes:
        msp.add_circle((u + du, v + dv), r, dxfattribs={'layer': 'CUT_INNER'})
        if csk: msp.add_circle((u + du, v + dv), csk / 2, dxfattribs={'layer': 'CSK90_FACE_UP'})
    if label:
        us, vs = [q[0] for q in p.outline], [q[1] for q in p.outline]
        msp.add_text('%s  PC %.1fmm  QTY 1  FACE UP: %s' % (p.name, p.t, p.face_up), height=3,
                     dxfattribs={'layer': 'NOTES'}).set_placement((min(us) + du, min(vs) + dv - 6))


def new_doc():
    doc = ezdxf.new('R2010')
    doc.header['$INSUNITS'] = 4
    for name, color in (('CUT_OUTER', 7), ('CUT_INNER', 1), ('CSK90_FACE_UP', 3), ('POCKET_2.0mm_UNDERSIDE', 4), ('NOTES', 5)):
        doc.layers.add(name, color=color)
    return doc


panel_manifest = []
for key, p in panels.items():
    doc = new_doc()
    us, vs = [q[0] for q in p.outline], [q[1] for q in p.outline]
    dxf_entities(doc.modelspace(), p, -min(us), -min(vs))
    doc.saveas(str(OUT / 'PC_3mm' / (p.name + '.dxf')))
    pc_bodies[key].shape.exportStep(str(OUT / 'PC_3mm' / (p.name + '.step')))
    b = pc_bodies[key]
    panel_manifest.append({'file': p.name + '.dxf', 'step': p.name + '.step', 'quantity': 1, 'material': 'PC solid sheet %.1f mm' % p.t, 'thickness_mm': p.t,
                           'flat_size_mm': [round(max(us) - min(us), 1), round(max(vs) - min(vs), 1)], 'face_up': p.face_up,
                           'holes': len(p.holes), 'countersinks': sum(1 for h in p.holes if h[3]), 'pockets': len(p.pockets),
                           'mass_g': round(b.shape.Volume() * PC_DENSITY, 1), 'note': p.note,
                           'sha256': hashlib.sha256((OUT / 'PC_3mm' / (p.name + '.dxf')).read_bytes()).hexdigest()})

# nest each sheet thickness (shelf packing, 6 mm gaps) for quoting
sheets = {}
for thick, fname, width in ((T, 'PC3_NEST_ALL.dxf', 520.0), (T_BULK, 'PC2_NEST_ALL.dxf', 200.0)):
    group_p = sorted([p for p in panels.values() if p.t == thick], key=lambda p: -(max(q[1] for q in p.outline) - min(q[1] for q in p.outline)))
    GAP = 6.0
    doc = new_doc(); msp = doc.modelspace()
    x = y = GAP; row_h = 0.0; used_w = 0.0
    for p in group_p:
        w = max(q[0] for q in p.outline) - min(q[0] for q in p.outline)
        h = max(q[1] for q in p.outline) - min(q[1] for q in p.outline)
        if x + w + GAP > width: x, y, row_h = GAP, y + row_h + GAP + 8, 0.0
        dxf_entities(msp, p, x - min(q[0] for q in p.outline), y + 8 - min(q[1] for q in p.outline))
        x += w + GAP; row_h = max(row_h, h); used_w = max(used_w, x)
    sheet = (round(used_w, 0), round(y + row_h + 8 + GAP, 0))
    msp.add_lwpolyline([(0, 0), (sheet[0], 0), (sheet[0], sheet[1]), (0, sheet[1])], close=True, dxfattribs={'layer': 'NOTES'})
    msp.add_text('NEST ONLY - sheet %.0f x %.0f mm, PC %.1f mm, all panels qty 1' % (sheet + (thick,)), height=4, dxfattribs={'layer': 'NOTES'}).set_placement((0, sheet[1] + 4))
    doc.saveas(str(OUT / 'PC_3mm' / fname))
    sheets['%.1f' % thick] = sheet
sheet = sheets


def print_shape(b):
    s, n = b.shape, b.name
    if n.startswith('Skirt_Bracket_PETG_Rear'): s = s.rotate((0, 0, 0), (0, 1, 0), -90)          # extrusion axis x -> z
    elif n.startswith('Skirt_Bracket'): s = s.rotate((0, 0, 0), (1, 0, 0), 90)                  # extrusion axis y -> z
    elif n.startswith('Weapon_Side_Block'):
        sgn = 1 if s.Center().x > 0 else -1
        web_out = sgn < 0
        s = s.rotate((0, 0, 0), (0, 1, 0), 90 if web_out else -90)                              # web face on the bed
    elif n.startswith('Hall_Post'): s = s.rotate((0, 0, 0), (0, 1, 0), -90)                      # as R3 build.print_shape
    bb = s.BoundingBox()
    return s.translate((-bb.xmin, -bb.ymin, -bb.zmin))


petg_manifest = []
for b in new_bodies + variant_bodies:
    s = print_shape(b)
    bb = s.BoundingBox()
    fname = 'V4_R3PC_' + b.name + '.stl'
    s.exportStl(str(OUT / 'PETG_STL' / fname), tolerance=0.02, angularTolerance=0.05, relative=False)
    m = next(r['mass_g'] for r in rows if r['part'] == b.name)
    petg_manifest.append({'file': fname, 'quantity': 1, 'material': 'PETG', 'dimensions_mm': [round(v, 2) for v in (bb.xlen, bb.ylen, bb.zlen)],
                          'mass_g_80pct_rule': round(m, 2), 'reissued_R3_part': b in variant_bodies, 'orientation': 'Z-up, bed Z=0',
                          'sha256': hashlib.sha256((OUT / 'PETG_STL' / fname).read_bytes()).hexdigest()})

import trimesh
for row in petg_manifest:
    mesh = trimesh.load(OUT / 'PETG_STL' / row['file'])
    row['watertight'] = bool(mesh.is_watertight)
for row in panel_manifest:
    audit = ezdxf.readfile(OUT / 'PC_3mm' / row['file']).audit()
    row['dxf_audit_errors'] = len(audit.errors)

# ---------------------------------------------------------------- previews (real CAD, not a concept render)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MPoly
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

colors = {'PC': '#9fd3f0', 'PETG': '#ed9d3e', '6061': '#a6b4c5', 'steel': '#667381', 'purchased': '#566ca3',
          'belt': '#343b45', 'foam': '#c6aada', 'TPU95A': '#56b2b0'}
fig = plt.figure(figsize=(15, 7), facecolor='#fafafa')
for index, title, hide in [(1, 'Assembly with PC body', ()), (2, 'Lid and right wall hidden: cleats, cradles, side blocks', ('PC3_Lid', 'PC3_Side_Wall_R'))]:
    ax = fig.add_subplot(1, 2, index, projection='3d')
    for b in root.bodies():
        if b.material == 'keepout' or b.name in hide: continue
        verts, tris = b.shape.tessellate(0.5, 0.3)
        v = np.array([p.toTuple() for p in verts])
        alpha = 0.35 if b.material == 'PC' else 0.95
        ax.add_collection3d(Poly3DCollection(v[np.array(tris)], facecolors=colors.get(b.material, '#aaa'), edgecolors='none', alpha=alpha))
    ax.set_xlim(-110, 110); ax.set_ylim(-125, 135); ax.set_zlim(0, 80); ax.set_box_aspect((220, 260, 80)); ax.view_init(elev=30, azim=-55)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Front +Y (mm)'); ax.set_zlabel('Z (mm)'); ax.set_title(title, fontsize=12)
fig.suptitle('V4 R3 + PC body (3 mm, bulkheads 2 mm) - DRAFT, unmeasured hardware', fontsize=15, fontweight='bold')
fig.text(0.5, 0.03, 'PC: light blue  |  PETG: orange  |  metal: grey  |  purchased envelopes: blue  |  TPU tyres: teal', ha='center', fontsize=10)
fig.tight_layout(rect=(0, 0.05, 1, 0.93)); fig.savefig(OUT / 'assembly_preview.png', dpi=150); plt.close(fig)

fig, axes = plt.subplots(2, 4, figsize=(18, 9), facecolor='white')
for ax, (key, p) in zip(axes.flat, panels.items()):
    def draw(pts, cutout, **kw):
        start, segs = filleted(pts, CORNER_R, cutout)
        xy = [start] + [s[1] if s[0] == 'L' else s[2] for s in segs]
        ax.add_patch(MPoly(xy, closed=True, **kw))
    draw(p.outline, False, facecolor='#d9eef9', edgecolor='k', lw=1)
    for c in p.cutouts: draw(c, True, facecolor='white', edgecolor='r', lw=1)
    for c, depth, face in p.pockets: draw(c, True, facecolor='#b8c9ff', edgecolor='b', lw=1, ls='--')
    for u, v, r, csk in p.holes:
        ax.add_patch(Circle((u, v), r, facecolor='white', edgecolor='r', lw=0.8))
        if csk: ax.add_patch(Circle((u, v), csk / 2, fill=False, edgecolor='g', lw=0.8))
    us, vs = [q[0] for q in p.outline], [q[1] for q in p.outline]
    ax.set_xlim(min(us) - 5, max(us) + 5); ax.set_ylim(min(vs) - 5, max(vs) + 5); ax.set_aspect('equal')
    ax.set_title('%s  (%.0f x %.0f)\nface up: %s' % (p.name, max(us) - min(us), max(vs) - min(vs), p.face_up), fontsize=9)
    ax.tick_params(labelsize=7)
axes.flat[-1].axis('off')
axes.flat[-1].text(0, 0.5, 'red: through cut / hole\ngreen: 90 deg countersink (D5.4 for M2.5) on the face-up side\n'
                   'blue dashed: 2.0 mm pocket from the underside\nall internal corners R1.5\nunits mm; PC solid 3.0 mm, bulkheads 2.0 mm', fontsize=11, va='center')
fig.tight_layout(); fig.savefig(OUT / 'panels_preview.png', dpi=130); plt.close(fig)

counts = {}
for f in fasteners: counts[f['kind']] = counts.get(f['kind'], 0) + 1
report = {
    'status': 'DRAFT_NOT_RELEASED', 'revision': 'V4_PRINT_R3 + PC 3 mm body variant', 'orientation': 'Z-up, ground XY at Z=0, nose +Y, mm',
    'r3_inputs': str(INPUTS.relative_to(PR.parents[1])),
    'r3_check': {'total_g': r3_mass['total_g'], 'rear_load_pct': round(100 * r3_mass['rear_static_load_fraction'], 3), 'tpu_tub_plus_lid_g': round(tpu_body_g, 2)},
    'body_allowance_g': round(2000 - rest_g, 2),
    'body_breakdown_g': {'PC panels': round(pc_g, 2), 'new PETG parts (80 % rule)': round(petg_new_g, 2), 'new fasteners': round(fast_g, 2),
                         're-issued R3 prints, delta': round(pc_variant_g - r3_variant_g, 2), 'total': round(body_g, 2)},
    'balance': balance, 'stiffness': stiff,
    'fastener_counts': counts, 'fastener_mass_g_each_ASSUMED': FASTENER_G,
    'mass_audit_vs_R3': mass_audit,
    'mass_check': {'rows_total': round(sum(r['mass_g'] for r in rows), 3), 'pc': round(pc_g, 3), 'petg_new': round(petg_new_g, 3), 'fast': round(fast_g, 3),
                   'variants': round(pc_variant_g, 3), 'r3_rest_in_rows': round(sum(r['mass_g'] for r in rows if r['part'] in r3_by), 3),
                   'other_rows': [(r['part'], r['mass_g']) for r in rows if r['part'] not in r3_by and r not in pc_rows and r not in fast_rows
                                  and r['part'] not in new_names and r['part'] not in variant_names],
                   'r3_rest_expected': round(rest_g - r3_variant_g, 3),
                   'r3_parts_not_in_rows': [(r['part'], r['mass_g']) for r in r3_mass['parts'] if r['part'] not in {q['part'] for q in rows}
                                            and r['part'] not in ('Tub_TPU', 'Lid_TPU_2mm')],
                   'r3_parts_sum': round(sum(r['mass_g'] for r in r3_mass['parts']), 3), 'r3_total': r3_mass['total_g'],
                   'r3_in_rows_detail': sorted([(q['part'], r3_by[q['part']], q['mass_g']) for q in rows if q['part'] in r3_by], key=lambda t: -abs(t[1]-t[2]))[:5],
                   'r3_multiplicity': {n: c for n, c in __import__('collections').Counter(r['part'] for r in r3_mass['parts']).items() if c > 1}},
    'interference': interference, 'screw_envelope_hits': envelope_hits, 'insert_spacing': insert_gaps,
    'panels': panel_manifest, 'nest_sheet_mm': sheet, 'petg_parts': petg_manifest,
    'pc_rows': pc_rows, 'new_petg_rows': [r for r in rows if r['part'] in new_names or r['part'] in variant_names],
    'fasteners': fasteners,
    'lid_top_mm': LID_TOP, 'inverted_wheel_plane_mm': 64.0, 'highest_electronics_mm': round(esp_top, 2),
}
# ---------------------------------------------------------------- M2.5 insert coupons (no R3 coupon covers M2.5)
trial = OUT / 'TRIAL_ONLY'
trial.mkdir(exist_ok=True)
for old in trial.glob('*.stl'): old.unlink()
for dmm in (3.2, 3.3, 3.4, 3.5, 3.6):
    cpn = block(0, 10, 0, 10, 0, 8)
    cpn = cpn.cut(cq.Solid.makeCylinder(dmm / 2, 5.2, cq.Vector(5, 5, 8 - 5), cq.Vector(0, 0, 1)))
    cq.Shape.exportStl(cpn, str(trial / ('Insert_M2.5_Hole_D%.1f_PETG.stl' % dmm)), 0.02, 0.05, False)

# ---------------------------------------------------------------- production drawings + Thai work order
import production_docs
groups, by_sig = [], {}
for b in new_bodies + variant_bodies:
    s3 = print_shape(b); bb = s3.BoundingBox()
    cc = s3.Center()
    sig = (b.name.split('_PETG')[0].replace('_PCbody', ''), round(s3.Volume(), 1), tuple(sorted(round(v, 2) for v in (bb.xlen, bb.ylen, bb.zlen))),
           round(abs(cc.x - bb.center.x), 2), round(cc.y - bb.center.y, 2), round(cc.z - bb.center.z, 2))   # L/R mirror is along print x
    row = next(r for r in petg_manifest if r['file'] == 'V4_R3PC_' + b.name + '.stl')
    ins = {}
    for f in fasteners:
        if f.get('host') == b.name and 'insert' in f['kind']: ins[f['kind']] = ins.get(f['kind'], 0) + 1
    if sig not in by_sig:
        name = b.name
        solid = name.startswith(('Cleat', 'Skirt', 'Motor_Twin', 'Motor_Lid'))
        by_sig[sig] = {'title': name.replace('_PETG', '').replace('_PCbody', ''), 'shape': s3, 'qty': 0, 'files': [], 'mass': row['mass_g_80pct_rule'],
                       'inserts': ', '.join('%s ×%d' % (k.replace('_heatset_insert', ''), n) for k, n in ins.items()) or '-',
                       'infill': 'เนื้อเต็ม (4 walls + 100%)' if solid else ('ตามข้อกำหนด R3 (บานพับ ≥1.6 มม.)' if 'Wedgelet' in name else 'infill ≤80% (คิดมวลไว้ 80%)'),
                       'note': {'Motor_Cradle': 'ผนังรับหน้ามอเตอร์ขับ JB361: รู Ø8 บ่า, counterbore Ø10 ลึก 0.6 รับจมูกดุมล้อ, Ø3.4 สกรูหน้ามอเตอร์ 2 รู, Ø7.5 ช่องหัวสกรู ลึก 1 · insert ใต้ฐาน',
                                'Motor_Twin_Saddle': 'รองปลายในของมอเตอร์ขับ 2 ตัว: cup R13.0 ใส่ EPDM 0.5 มม. · ช่อง x ±4 สำหรับขั้วและสาย',
                                'Motor_Lid_Bar': 'ยึดใต้ฝาด้วย M2.5 หัวจม 2 ตัว: cup R13.0 ใส่ EPDM 0.5 มม. · เว้น 0 มม. เมื่อประกอบ',
                                'Weapon_Side_Block': 'ผนัง 6 มม. รับแผ่นตั้งอาวุธ: M3×10 4 ตัว หัวจมที่ |x| 38 · ช่อง Ø21 สำหรับประแจ M8',
                                'Cleat': 'ตัวยึดมุมระหว่างแผ่น PC: insert M2.5 ทุกหน้าที่แนบแผ่น',
                                'Skirt_Bracket': 'บ่า 35° รับแผ่น skirt R3 (รู M2 Ø1.6 เกลียวปล่อย) + insert M2.5 ยึดผนัง',
                                'Wedgelet_Block': 'R3 wedgelet block + insert M3 ใต้ราง 2 ตัว (ยึดพื้น)'}.get(next((k for k in ('Motor_Cradle', 'Motor_Twin_Saddle', 'Motor_Lid_Bar', 'Weapon_Side_Block', 'Cleat', 'Skirt_Bracket', 'Wedgelet_Block') if name.startswith(k)), ''), '')}
        groups.append(by_sig[sig])
    by_sig[sig]['qty'] += 1
    by_sig[sig]['files'].append(row['file'])
for k, gr in enumerate(groups, 1):
    gr['id'] = 'P%02d' % k
    names = [f[len('V4_R3PC_'):-4].replace('_PETG', '').replace('_PCbody', '') for f in gr['files']]
    if len(set(names)) > 1:
        parts_ = [n.split('_') for n in names]
        common = []
        for i, t in enumerate(parts_[0]):
            if not all(len(q) > i and q[i] == t for q in parts_): break
            common.append(t)
        tails = sorted(set('_'.join(q[len(common):]) for q in parts_))
        gr['title'] = '_'.join(common) + ' ' + ' / '.join(tails)

hw = config['hardware']
blockers = []
keys = ('mount_holes_yz_mm', 'boss_diameter_mm', 'boss_projection_mm', 'diameter_mm', 'length_mm', 'shaft_projection_mm', 'mass_g')
unmeasured = [nm for nm in ('drive_Front_L', 'drive_Front_R', 'drive_Rear_L', 'drive_Rear_R') if any(hw[nm]['actual'].get(k) is None for k in keys)]
if unmeasured:
    blockers.append('มอเตอร์ขับ %d ตัว (%s) ยังไม่ได้วัดจริง: ระยะรูยึดหน้ามอเตอร์, บ่า Ø/ความยื่น, Ø ตัวมอเตอร์, ความยาวรวม+ขั้ว, ความยื่นเพลา, มวล '
                    '(รูยึด/บ่า → ผนัง cradle; Ø → cup ของ twin saddle / lid bar; ความยาว → ช่องขั้วที่ x ±4)' % (len(unmeasured), ', '.join(unmeasured)))
if config['fit'].get('insert_hole_diameter_mm') is None:
    blockers.append('ยังไม่ได้เลือกขนาดรู insert M3 จาก coupon R3 (ค่าทดลอง Ø4.6) และ M2.5 จาก coupon ใหม่ใน output/TRIAL_ONLY (ค่าทดลอง Ø3.4)')
blockers += ['ทดสอบดึง insert จริง: M3 ≥ 500 N, M2.5 ≥ 300 N, สกรู M2 เกลียวปล่อยใน PETG ≥ 200 N (ค่าที่ใช้คำนวณ SF ข้อ 3)',
             'วัดความหนาแผ่น PC 3 มม. และ 2 มม. ที่ร้านจะใช้ (4 จุด) และชั่งแผ่นหลังตัด แล้วรันสคริปต์ใหม่',
             'ชั่งสกรู insert และแผ่น EPDM จริงอย่างละ 10 ชิ้น แล้วแก้ FASTENER_G และรันใหม่',
             'น้ำหนักจาก slicer ของชิ้น PETG ทุกไฟล์ (แท่นมอเตอร์/บล็อกอาวุธ ≤80%) แล้วรันใหม่: ต้องเหลือเผื่อ ≥20 g',
             'รัน MuJoCo/การจำลองดวลใหม่ด้วยมวลและ inertia ของตัวถัง PC',
             'ใบเสนอราคาครบทุกหมวด รวมทั้งโครงการ ≤ 6,000 บาท (รวมเงินเผื่อ 300)']
use = {'M3x6_button_ISO7380': 'cradle, บล็อกอาวุธ, ราง wedgelet → พื้น/bulkhead', 'M3_heatset_insert_L4_OD5': 'cradle, บล็อกอาวุธ, ราง wedgelet',
       'M2.5x6_button': 'ตัวยึดมุม (ผนัง/พื้น/ผนังหลัง/bulkhead), twin saddle, ขายึด skirt หลัง', 'M2.5x6_csk_ISO10642': 'ฝา→ตัวยึดมุม 6, ฝา→lid bar 4, ขายึด skirt ข้าง 8 (จากด้านใน)',
       'M2.5_heatset_insert_L4_OD3.6': 'ตัวยึดมุม, twin saddle, lid bar, ขายึด skirt', 'M3x10_button_ISO7380': 'บล็อกอาวุธ → แผ่นตั้ง (แทน M3×20 ของ R3; มวลอยู่ในค่าเผื่อ R3)',
       'EPDM_pad_0.5mm': 'รองใน cup ของ twin saddle และ lid bar (กาวในตัว)'}
ctx = {'report': report, 'status': 'BLOCKED — รอผลวัด/ทดสอบ ห้ามผลิตชุดจริง', 'blockers': blockers, 'fastener_counts': counts,
       'fastener_g': FASTENER_G, 'fastener_use': use}
# Partial release recorded by the user (r3/PRODUCTION/RELEASE_DECISION.json): stamp each sheet with its own status.
decision_path = PR / 'r3' / 'PRODUCTION' / 'RELEASE_DECISION.json'
if decision_path.exists():
    decision = json.loads(decision_path.read_text())
    if not build_signature(config).startswith(decision['basis']['r3_build_sha256_prefix']):
        raise SystemExit('RELEASE_DECISION.json was made for another R3 build: re-decide or delete it')
    released_panels = set(decision['released']['cnc_pc_panels'])
    released_petg = decision['released'].get('pc_body_petg') == 'all'

    def status_of(kind, name):
        if (kind == 'panel' and name in released_panels) or (kind == 'petg' and released_petg):
            return 'RELEASED — อนุมัติผลิต %s (รับความเสี่ยงตาม RELEASE_DECISION.json)' % decision['date']
        if kind == 'petg' and name.startswith(('Motor_Cradle', 'Motor_Twin_Saddle', 'Motor_Lid_Bar')):
            return 'HOLD — รอวัดมอเตอร์ขับ และผล coupon insert'
        if kind == 'petg':
            return 'HOLD — รอผล coupon insert'
        return ctx['status']
    ctx['status_of'] = status_of
    if released_petg and released_panels == {pn.name for pn in panels.values()}:
        ctx['status'] = 'RELEASED — อนุมัติผลิต %s: ผู้สั่งรับความเสี่ยงตาม RELEASE_DECISION.json' % decision['date']
report['production'] = production_docs.make(OUT, panels, panel_manifest, groups, ctx, filleted)
report['release_blockers'] = blockers
report['petg_groups'] = [{k: v for k, v in gr.items() if k != 'shape'} for gr in groups]
(OUT / 'pc_body_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')
print(json.dumps({'interference': len(interference), 'screw_envelope_hits': len(envelope_hits),
                  'min_insert_gap_mm': min((g['gap_mm'] for g in insert_gaps if 'gap_mm' in g), default=None),
                  'min_insert_outer_wall_mm': min(g['min_outer_wall_mm'] for g in insert_gaps if 'min_outer_wall_mm' in g),
                  'body': report['body_breakdown_g'], 'allowance_g': report['body_allowance_g'],
                  'r3': balance['r3_tpu'], 'pc': balance['pc_body'], 'nest_sheet_mm': sheet}, indent=1))
