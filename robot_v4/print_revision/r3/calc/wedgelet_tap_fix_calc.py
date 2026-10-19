"""R3 RFQ defects: wedgelet plate thickness options and upright edge-tap / side-tap clearance.

Wedgelet: every option is modelled in CadQuery as a real flat plate (thickness perpendicular to its faces,
underside = carrier top, rear end square as laser-cut, front end ground vertical as now), its mass and
centroid replace Wedgelet_L/R in build_report.json, and the rear wheel load is recomputed with the build's own
formula (front_y - CG_y) / (front_y - rear_y). Nothing in output_R3 or the CAD inputs is modified.
The baseline is the pre-fix R3 build 319857d1 (archived 1.627 mm plate), read from git commit 665be50, so a rerun
after the fix still compares against the same plate.
Run: .venv-v4-print/bin/python robot_v4/print_revision/r3/calc/wedgelet_tap_fix_calc.py
"""
import json
import math
import subprocess
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
R3 = HERE.parent
BASELINE_COMMIT, BASELINE_BUILD = '665be50', '319857d1'
REPORT = json.loads(subprocess.run(['git', 'show', BASELINE_COMMIT + ':robot_v4/print_revision/r3/output_R3/build_report.json'],
                                   cwd=HERE, capture_output=True, text=True, check=True).stdout)
if not REPORT['build_sha256'].startswith(BASELINE_BUILD):
    raise RuntimeError('baseline build_report is not the pre-fix R3 build ' + BASELINE_BUILD)
STEEL = 0.00785                      # g/mm3, same as checks.py
AL = 0.0027
FRONT_Y, REAR_Y = 18.0, -70.0
REAR_MIN = 0.10
MASS_LIMIT = 2000.0
X0, X1 = 27.0, 88.0                  # right plate span (left is the mirror)
HOLES_X = (49.0, 75.0)
R3_LIFTER_N = 111.36                 # options_calc.json, servo 4 N*m on 35.9 mm arm
YIELD = {'mild_SS400': 275.0, 'spring_65Mn_SK5_hardened': 1000.0}   # MPa, ASSUMED handbook (as options_calc)

# 90 deg countersunk heads (mm). dk_theo = theoretical sharp-edge max (worst case), dk_act = actual head max.
# The countersink must be cut to the head actually bought, measured; both bases are reported.
HEADS = {
    'M3 DIN 7991 hex socket': dict(hole=3.4, dk_theo=6.0, dk_act=6.0, k=1.7),
    'M3 ISO 10642 hex socket': dict(hole=3.4, dk_theo=6.72, dk_act=6.72, k=1.86),       # actual max not tabulated
    'M3 ISO 14581 / ISO 7046 / JIS B1111 flat': dict(hole=3.4, dk_theo=6.3, dk_act=5.5, k=1.65),
    'M2.5 ISO 14581 / ISO 7046': dict(hole=2.9, dk_theo=5.5, dk_act=4.7, k=1.5),
}


def plate(angle_deg, t, rear=(100.0, 20.5), tip_z=0.5, holes=None, csk=None):
    """Right wedgelet. Underside runs rear -> tip; thickness t along the face normal."""
    a = math.radians(angle_deg)
    u = (math.cos(a), -math.sin(a))                  # down-slope direction (y, z)
    n = (math.sin(a), math.cos(a))                   # face normal, up/forward
    length_under = (rear[1] - tip_z) / math.sin(a)
    tip = (rear[0] + u[0] * length_under, tip_z)
    top_rear = (rear[0] + n[0] * t, rear[1] + n[1] * t)
    top_front = (tip[0], tip[1] + t / math.cos(a))   # vertical ground front edge, as R3 / RFQ
    poly = [rear, tip, top_front, top_rear]
    wp = cq.Workplane(cq.Plane(origin=(X0, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0)))
    shape = wp.polyline(poly).close().extrude(X1 - X0).val()
    if holes:
        hole_d, dk = holes, csk
        # Holes on the carrier-insert axis: carriers' insert points sit on the underside at y 109.05.
        s = (109.05 - rear[0]) / u[0]
        base = (rear[0] + u[0] * s, rear[1] + u[1] * s)
        for x in HOLES_X:
            o = cq.Vector(x, base[0] - n[0], base[1] - n[1])
            d = cq.Vector(0, n[0], n[1])
            shape = shape.cut(cq.Solid.makeCylinder(hole_d / 2, t + 2, o, d))
            if dk:
                h = (dk - hole_d) / 2                       # 90 deg cone depth
                top = cq.Vector(x, base[0] + n[0] * t, base[1] + n[1] * t)
                shape = shape.cut(cq.Solid.makeCone(hole_d / 2, dk / 2 + 0.5, h + 0.5, top - d * h, d))
    return shape, poly, length_under


def mass_row(shape):
    return shape.Volume() * STEEL, shape.Center()


base_parts = REPORT['mass']['parts']
base_total = sum(p['mass_g'] for p in base_parts)
base_my = sum(p['mass_g'] * p['centroid_mm'][1] for p in base_parts)
old = [p for p in base_parts if p['part'] in ('Wedgelet_L', 'Wedgelet_R')]
old_m = sum(p['mass_g'] for p in old)
old_my = sum(p['mass_g'] * p['centroid_mm'][1] for p in old)


def balance(new_pair_g, new_pair_my, extra=()):
    total = base_total - old_m + new_pair_g + sum(m for m, _ in extra)
    my = base_my - old_my + new_pair_my + sum(m * y for m, y in extra)
    cg = my / total
    return dict(total_g=round(total, 2), cg_y_mm=round(cg, 3), rear_load_pct=round(100 * (FRONT_Y - cg) / (FRONT_Y - REAR_Y), 3),
                rear_ok=(FRONT_Y - cg) / (FRONT_Y - REAR_Y) >= REAR_MIN, mass_ok=total <= MASS_LIMIT,
                margin_to_2000_g=round(MASS_LIMIT - total, 2))


out = {'baseline': {'build_sha256': REPORT['build_sha256'], 'total_g': round(base_total, 2),
                    'rear_load_pct': round(100 * REPORT['mass']['rear_static_load_fraction'], 3)}}

# Check: the archived 2 mm vertical offset on the 35.54 deg slope is 2*cos(35.54) perpendicular.
slope = math.degrees(math.atan2(20, 28))
out['as_built_R3'] = {'slope_deg': round(slope, 3), 'vertical_offset_mm': 2.0, 'perpendicular_t_mm': round(2 * math.cos(math.radians(slope)), 4),
                      'cad_mass_each_g': old[0]['mass_g']}

options = {}
for key, ang, t, head, rear in [
        ('A_1.6_M3_DIN7991', slope, 1.6, 'M3 DIN 7991 hex socket', (100.0, 20.5)),
        ('A_1.6_M3_ISO10642', slope, 1.6, 'M3 ISO 10642 hex socket', (100.0, 20.5)),
        ('A_1.6_M3_ISO14581', slope, 1.6, 'M3 ISO 14581 / ISO 7046 / JIS B1111 flat', (100.0, 20.5)),
        ('A_1.6_M2.5_ISO14581', slope, 1.6, 'M2.5 ISO 14581 / ISO 7046', (100.0, 20.5)),
        ('B_2.0_M3_ISO14581', slope, 2.0, 'M3 ISO 14581 / ISO 7046 / JIS B1111 flat', (100.0, 20.5)),
        ('B_2.0_M3_DIN7991', slope, 2.0, 'M3 DIN 7991 hex socket', (100.0, 20.5)),
        ('C_25deg_1.5_M3_DIN7991', 25.0, 1.5, 'M3 DIN 7991 hex socket', (100.0, 20.5))]:
    hd = HEADS[head]
    shape, poly, length = plate(ang, t, rear, holes=hd['hole'], csk=hd['dk_act'])
    m, c = mass_row(shape)
    pair_g, pair_my = 2 * m, 2 * m * c.y
    width = X1 - X0
    z_sec = width * t ** 2 / 6
    row = dict(angle_deg=round(ang, 2), t_mm=t, fastener=head, plate_length_mm=round(length, 2),
               tip_y_mm=round(poly[1][0], 2), mass_each_g=round(m, 2), centroid_y_mm=round(c.y, 2),
               delta_pair_g=round(pair_g - old_m, 2), delta_moment_pair_gmm=round(pair_my - old_my, 1),
               land_mm_theoretical_head=round(t - (hd['dk_theo'] - hd['hole']) / 2, 3),
               land_mm_actual_head=round(t - (hd['dk_act'] - hd['hole']) / 2, 3),
               land_fraction_actual=round((t - (hd['dk_act'] - hd['hole']) / 2) / t, 3),
               new_inserts_and_carrier_holes=hd['hole'] < 3.4,
               head_height_k_mm=hd['k'],
               bending_stiffness_vs_R3=round((t / 1.6275) ** 3, 2),
               pry_yield_tip_N={k: round(s * z_sec / length, 1) for k, s in YIELD.items()},
               pry_margin_vs_R3_lifter={k: round(s * z_sec / length / R3_LIFTER_N, 2) for k, s in YIELD.items()},
               **balance(pair_g, pair_my))
    options[key] = row

# Options that fail 10 %: rear ballast on the inside of the rear wall (y -105) needed for 10.00 % and for R3's 10.48 %.
def offset(key):
    o = options[key]
    total = o['total_g']
    my = o['cg_y_mm'] * total
    need = {}
    for target in (0.10, 0.1048):
        cg_t = FRONT_Y - target * (FRONT_Y - REAR_Y)
        b = max(0.0, (my - cg_t * total) / (cg_t + 105.0))
        need['%.2f%%' % (100 * target)] = {'rear_ballast_at_y-105_g': round(b, 2), 'total_g': round(total + b, 2),
                                           'margin_to_2000_g': round(MASS_LIMIT - total - b, 2)}
    o['rear_ballast_needed'] = need


for key in options:
    if not options[key]['rear_ok']:
        offset(key)
out['wedgelet_options'] = options

# 25 deg / 1.5 mm: options_calc.py scaled from the 26.58 g as-built plate as if it were 2.0 mm thick.
oc = json.loads((HERE / 'options_calc.json').read_text())['wedgelet_plates']['w25_long_t1.5']
out['options_calc_correction'] = dict(options_calc_mass_each_g=round(oc['plate_g_each'], 2), cad_mass_each_g=options['C_25deg_1.5_M3_DIN7991']['mass_each_g'],
                                      options_calc_delta_pair_g=round(oc['delta_mass_g_pair'], 2), cad_delta_pair_g=options['C_25deg_1.5_M3_DIN7991']['delta_pair_g'],
                                      cause='options_calc assumed the R3 plate was 2.0 mm; it is 1.627 mm, so every scaled mass is 2.0/1.627 = 1.229x low',
                                      carriers_note='25 deg also needs new carrier/insert geometry; not modelled here')

# ---- Upright edge taps (x +-30, y 69 from brace holes) vs the M3 side taps through the 6 mm upright ----
# Solids: drill D2.5 to full depth + 118 deg point; thread = M3 major D3.0 to thread depth. Distances by OCC.
TAP_D, MAJOR = 2.5, 3.0
POINT = TAP_D / 2 / math.tan(math.radians(59))


def edge_hole(y, z_edge, sgn, depth, d, point=True):
    o = cq.Vector(30, y, z_edge - sgn * 0.5)
    v = cq.Vector(0, 0, sgn)
    solid = cq.Solid.makeCylinder(d / 2, depth + 0.5, o, v)
    if point:
        solid = solid.fuse(cq.Solid.makeCone(d / 2, 0, POINT, o + v * (depth + 0.5), v))
    return solid


def side_hole(y, z, d):
    return cq.Solid.makeCylinder(d / 2, 6, cq.Vector(27, y, z), cq.Vector(1, 0, 0))


def walls(side_screws, depth, thread_depth):
    rows = []
    for y, z in side_screws:
        for edge, z_edge, sgn in (('top', 57.0, -1), ('bottom', 7.0, 1)):
            drill = edge_hole(69, z_edge, sgn, depth, TAP_D)
            thread = edge_hole(69, z_edge, sgn, thread_depth, MAJOR, point=False)
            overlap = drill.intersect(side_hole(y, z, TAP_D)).Volume()
            rows.append(dict(side_tap_yz=[y, z], edge=edge,
                             drill_wall_mm=round(-1.0 if overlap > 1e-6 else drill.distance(side_hole(y, z, TAP_D)), 2),
                             drill_overlap_mm3=round(overlap, 3),
                             thread_wall_mm=round(thread.distance(side_hole(y, z, MAJOR)), 2)))
    return dict(min_drill_wall_mm=min(r['drill_wall_mm'] for r in rows), rows=rows)


R3_SIDE = [(70, 14), (70, 50), (90, 14), (90, 46)]
NEW_SIDE = [(75, 14), (75, 50), (90, 14), (90, 46)]
taps = {'R3_side_y70_drill8_thread6': walls(R3_SIDE, 8, 6), 'R3_side_y70_drill5_thread4': walls(R3_SIDE, 5, 4),
        'new_side_y75_drill8_thread6': walls(NEW_SIDE, 8, 6)}
mr = 35.0 / 2 + 1                                  # weapon-motor relief in the left upright, R3 motor D35
relief = cq.Solid.makeCylinder(mr, 6, cq.Vector(27, 47.5, 42.5), cq.Vector(1, 0, 0))
axle = cq.Solid.makeCylinder(4.1, 6, cq.Vector(27, 95, 32), cq.Vector(1, 0, 0))
top = edge_hole(69, 57.0, -1, 8, TAP_D)
taps['new_layout_other_walls'] = {
    'edge_tap_to_rear_edge_y64_mm': round(69 - TAP_D / 2 - 64.0, 2),
    'edge_tap_to_plate_faces_x_mm': round((6 - TAP_D) / 2, 2),
    'edge_tap_to_left_motor_relief_mm': round(top.distance(relief), 2),
    'side_y75_to_axle_bore_mm': round(side_hole(75, 50, TAP_D).distance(axle), 2),
    'side_y75_to_left_motor_relief_mm': round(side_hole(75, 50, TAP_D).distance(relief), 2),
    'side_y75_to_side_y90_drill_wall_mm': round(15 - TAP_D, 2),
    'side_y75_to_rear_edge_mm': round(75 - TAP_D / 2 - 64.0, 2),
    'side_y75_axis_to_axle_mm': round(math.hypot(95 - 75, 50 - 32), 2),
    'side_y70_axis_to_axle_mm': round(math.hypot(95 - 70, 50 - 32), 2),
    'tooth_tip_radius_mm': 29.5,
    'side_screw_max_length_mm': 22,
    'side_screw_length_note': 'head on the TPU block face x +-50; upright inner face x +-27 is 23 mm away; disc face at x +-26. '
                              'y 75 is inside the tooth circle like y 90, so the screw tip must stay inside the upright (<= M3x22; R3 uses M3x20).',
    'brace_hole_y69_edge_distance_mm': {'to_y64': round(69 - 64 - 1.7, 2), 'top_brace_to_y77': round(77 - 69 - 1.7, 2), 'bottom_brace_to_y74': round(74 - 69 - 1.7, 2)},
    'why_not_move_braces': 'top-brace front edge y 77 is 1.3 mm from the tooth circle at z 57 (rotor y95 z32 r29.5); a brace hole '
                           'far enough from the y70 side tap (>= 5 mm) needs y <= 65 (1.3 mm to the y64 edge) or y >= 75 (into the rotor sweep).',
}
out['upright_taps'] = taps

# Every hole pair in the fixed upright (drill sizes), plus each hole to the plate outline. Requirement: >= 1.5 mm.
from itertools import combinations
OUTLINE = [(64, 7), (64, 57), (104, 57), (112, 40), (112, 16.5), (100, 25), (98, 25), (98, 7)]
plate = cq.Workplane(cq.Plane(origin=(27, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))).polyline(OUTLINE).close().extrude(6).val()
holes = {'axle D8.2': cq.Solid.makeCylinder(4.1, 6, cq.Vector(27, 95, 32), cq.Vector(1, 0, 0)),
         'motor relief (L only)': relief,
         'edge tap top': edge_hole(69, 57.0, -1, 8, TAP_D), 'edge tap bottom': edge_hole(69, 7.0, 1, 8, TAP_D)}
holes.update({'side tap y%d z%d' % yz: side_hole(yz[0], yz[1], TAP_D) for yz in NEW_SIDE})
pairs = {}
for (na, a_), (nb, b_) in combinations(holes.items(), 2):
    pairs[na + ' | ' + nb] = round(-1.0 if a_.intersect(b_).Volume() > 1e-6 else a_.distance(b_), 2)
edge_walls = {}
for n, h in holes.items():
    if n.startswith('motor'):
        continue
    # distance from the hole to the outline = min over outline side faces (x faces excluded: they are the sheet faces)
    faces = [f for f in plate.Faces() if abs(f.normalAt().x) < 0.5]
    edge_walls[n] = round(min(h.distance(f) if not n.startswith('edge') else (h.distance(f) if h.intersect(f).Area() < 1e-6 else 99) for f in faces), 2)
out['upright_hole_walls'] = {'required_mm': 1.5, 'hole_to_hole_mm': pairs, 'min_hole_to_hole_mm': min(pairs.values()),
                             'hole_to_outline_mm': edge_walls, 'edge_tap_to_sheet_faces_mm': {'drill': round((6 - TAP_D) / 2, 2), 'thread_major': round((6 - MAJOR) / 2, 2)}}

# ---- Fastener length / thread engagement ----
TAU_AL = 0.6 * 310.0          # 6061-T6 shear ~0.6 Rm, MPa (ASSUMED handbook)
AS_M3 = 5.03                  # mm2 tensile stress area
def strip_kN(le):             # internal-thread shear area ~ pi*d*Le*0.75 (simplified)
    return math.pi * 3.0 * le * 0.75 * TAU_AL / 1000
le_full = {g: round(AS_M3 * rm / (math.pi * 3.0 * 0.75 * TAU_AL), 2) for g, rm in (('8.8', 800), ('A2-70', 700))}
out['fasteners'] = {
    'Le_needed_to_strip_after_bolt_breaks_mm': le_full,
    'brace_to_upright_edge': {'screw': 'M3x8 (button or socket)', 'clamped_mm': 3.0, 'engagement_mm': 5.0, 'thread_depth_mm': 6.0,
                              'thread_left_below_tip_mm': 1.0, 'drill_left_below_tip_mm': 3.0, 'strip_capacity_kN': round(strip_kN(5.0), 2),
                              'bolt_8.8_break_kN': round(AS_M3 * 0.8, 2), 'note': 'M3x10 would reach 7 mm, past the 6 mm thread: do not use'},
    'side_block_to_upright': {'screw': 'M3x20 + 0.5 washer (R3 TPU tub)', 'stack_mm': '0.5 washer + 16 TPU block + 1 gap', 'engagement_mm': 2.5,
                              'strip_capacity_kN': round(strip_kN(2.5), 2), 'tip_x_mm': 29.5, 'upright_inner_face_x_mm': 27.0,
                              'option_M3x22': {'engagement_mm': 4.5, 'tip_x_mm': 27.5, 'strip_capacity_kN': round(strip_kN(4.5), 2)},
                              'max_length_mm': 22, 'note': 'y75 is inside the 29.5 mm tooth circle; the tip must stay inside the upright (disc face x 26)'},
    'wedgelet_to_carrier': {'screw': 'M3x5 flat head ISO 7046 / JIS B1111', 'plate_mm': 1.6, 'tip_below_underside_mm': 3.4, 'insert_length_mm': 4.0},
}

# ---- Same options on the PC-body variant (r3/pc_body owns its report; read-only here) ----
# The PC-body report is built on the fixed R3 (1.6 mm plate, option A), so each option is applied as its
# difference from A. Wedgelets are identical in both bodies.
PC = R3 / 'pc_body' / 'output' / 'pc_body_report.json'
if PC.exists():
    pc = json.loads(PC.read_text())
    A = options['A_1.6_M3_ISO14581']
    cases = {'TPU tub R3 (built)': pc['balance']['r3_tpu'], 'PC body nominal': pc['balance']['pc_body']}
    cases.update({'PC ' + k: v for k, v in pc['balance']['sensitivity'].items()})
    table = {}
    for key, o in options.items():
        dm = o['delta_pair_g'] - A['delta_pair_g']
        dmy = o['delta_moment_pair_gmm'] - A['delta_moment_pair_gmm']
        row = {}
        for name, c in cases.items():
            total = c['total_g'] + dm
            cg = (c['total_g'] * c['cg_y_mm'] + dmy) / total
            rl = (FRONT_Y - cg) / (FRONT_Y - REAR_Y)
            row[name] = {'total_g': round(total, 2), 'rear_load_pct': round(100 * rl, 3),
                         'passes_2000g_10pct': bool(total <= MASS_LIMIT and rl >= REAR_MIN)}
        table[key] = row
    out['both_bodies'] = {'pc_body_report_r3_total_g': pc['r3_check']['total_g'], 'delta_basis': 'option minus A_1.6_M3_ISO14581 (built)',
                          'options': table}

(HERE / 'wedgelet_tap_fix_calc.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
print(json.dumps(out, indent=1, ensure_ascii=False))
