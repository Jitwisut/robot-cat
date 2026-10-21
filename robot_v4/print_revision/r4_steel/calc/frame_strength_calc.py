"""R4-steel frame strength: weapon hit (2.6 kN design, 5.2 kN check) and 0.5 m drops, joint by joint.

Same load cases and method as the R3 PC body (r3/pc_body/build_pc_body.py, CALC_PC_BODY.md section 10):
  DT1  0.5 m drop upright, 2 mm stop -> 250 g      DT2  same, inverted
  DT3  tooth force 2.6 kN (1 ms half-sine), check 5.2 kN (0.5 ms); the weapon module's own inertia takes its share,
       the rest goes into the body through the two upright mounts: S = F * (1 - m_weapon / M) / 2 per side
Geometry and masses come from output/build_report.json (build_r4_steel.py). Steel grades are not certified, so the
capacities are ASSUMED handbook values (listed in 'capacities'). Required SF 1.5 on the 2.6 kN design and drops.
Run: python3 robot_v4/print_revision/r4_steel/calc/frame_strength_calc.py   (writes frame_strength_calc.json)
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R4 = os.path.dirname(HERE)
rep = json.load(open(os.path.join(R4, 'output', 'build_report.json')))
rows = rep['rows']
M = rep['mass']['total_g'] / 1000
G = 9.81
N_DROP = 500.0 / 2.0
F_DESIGN, F_CHECK, SF_REQ = 2600.0, 5200.0, 1.5
CAP = {'steel_fy_MPa': 235.0, 'steel_fu_MPa': 360.0, 'weld_MPa': 0.6 * 480.0,   # mild steel S235/STKR400, ER70S-6 wire
       'bolt_8.8_shear_N': 0.6 * 640.0 * 5.03, 'al6061_bearing_MPa': 386.0, 'pc_bearing_MPa': 62.0,
       'd_shaft_yield_MPa': (355.0, 640.0)}
FILLET_THROAT = {'tube': 0.7 * 1.0, 'tab': 0.7 * 1.5}   # fillet leg = thinner part, throat = 0.7 leg
AXLE = (95.0, 32.0)          # weapon axle (y, z)
RAIL_Z = 11.0                # rail centroid z (6..16)


def mass(prefixes):
    return sum(r['mass_g'] for r in rows if r['part'].startswith(prefixes)) / 1000


m_weapon = mass(('Upright_', 'Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Nose_Skid', 'Beater_', 'Bearing_608', 'Spacer_',
                 'Dead_Shaft', 'Weapon_Axle_', 'Belt_', 'rotor screws'))
body_share = (M - m_weapon) / M


def box_tube(side, wall):
    """Square tube: area, elastic and plastic section moduli, torsion (thin-wall Bredt)."""
    b, i = side, side - 2 * wall
    A = b * b - i * i
    I = (b ** 4 - i ** 4) / 12
    Zel, Zpl = I / (b / 2), (b ** 3 - i ** 3) / 4
    Am = (b - wall) ** 2
    return {'A': A, 'I': I, 'Zel': Zel, 'Zpl': Zpl, 'Am': Am, 't': wall}


def weld_group(lines, throat, Fy, Fz, Mx, ref):
    """Max resultant stress (MPa) in a fillet weld group in the y-z plane (polar method, lines as dense points).
    lines: [((y0, z0), (y1, z1))]; loads Fy, Fz (N) at ref (y, z); Mx about +x (N mm)."""
    pts, w = [], []
    for (a, b) in lines:
        n = 200
        L = math.dist(a, b)
        for k in range(n):
            t = (k + 0.5) / n
            pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            w.append(L / n * throat)
    pts, w = np.array(pts), np.array(w)
    A = w.sum()
    c = (pts * w[:, None]).sum(0) / A
    r = pts - c
    J = (w * (r ** 2).sum(1)).sum()
    M_c = Mx + Fz * (ref[0] - c[0]) - Fy * (ref[1] - c[1])
    ty = Fy / A - M_c * r[:, 1] / J
    tz = Fz / A + M_c * r[:, 0] / J
    return float(np.max(np.hypot(ty, tz))), {'area_mm2': round(A, 1), 'centroid_yz': [round(v, 2) for v in c], 'J': round(J, 0),
                                             'M_about_centroid_Nmm': round(M_c, 0)}


rail_wall = next(c['wall_mm'] for c in rep['tube_cut_list_mm'] if c['part'].startswith('Frame_Rail'))
out = {'inputs': {'M_kg': M, 'm_weapon_kg': round(m_weapon, 4), 'body_share': round(body_share, 4), 'rail_wall_mm': rail_wall,
                  'N_drop_g': N_DROP, 'capacities': CAP}, 'paths': []}


def path(name, case, load, cap, unit='N', note=''):
    sf = cap / load if load > 0 else None
    out['paths'].append({'path': name, 'case': case, 'load': round(load, 1), 'capacity': round(cap, 1), 'unit': unit,
                         'sf': None if sf is None else round(sf, 2), 'pass': sf is None or sf >= (1.0 if 'check' in case else SF_REQ), 'note': note})


# ---------------------------------------------------------------- DT3 weapon hit, per side
# upright web bolts: the 4 R3 upright taps; bracket = web (x 33..34.5) + floor flange (z 6..7.5, y 62..88, to the rail)
# + rungs (y 68.5 and 81, z 7.5..28) welded to the rail's inner face (x 48)
taps = rep['upright_taps_yz']
tap_c = np.mean(taps, axis=0)
for F, case in ((F_DESIGN, 'DT3 2.6 kN'), (F_CHECK, 'DT3 5.2 kN check')):
    S = F * body_share / 2
    for direction, (Fy, Fz) in (('fore-aft (-y)', (-S, 0.0)), ('up (+z)', (0.0, S)), ('down (-z)', (0.0, -S))):
        tag = '%s %s' % (case, direction)
        # 1. bolts: direct + eccentric share (polar, 4 bolts)
        r = np.array(taps) - tap_c
        Mb = Fz * (AXLE[0] - tap_c[0]) - Fy * (AXLE[1] - tap_c[1])
        Jb = (r ** 2).sum()
        per = max(math.hypot(Fy / 4 - Mb * ri[1] / Jb, Fz / 4 + Mb * ri[0] / Jb) for ri in r)
        path('Upright M3 8.8 bolts x4, shear (max bolt)', tag, per, CAP['bolt_8.8_shear_N'])
        path('Bolt bearing in the 1.5 mm steel web', tag, per, 2.4 * 3.0 * 1.5 * CAP['steel_fu_MPa'])
        path('Bolt bearing in the 6 mm 6061 upright', tag, per, 3.0 * 6.0 * CAP['al6061_bearing_MPa'])
        # 2. web -> flange (fillets both sides, y 66..88) + web -> rungs (vertical fillets, z 7.5..28)
        web_lines = [((66.0, 7.5), (88.0, 7.5))] * 2 + [((69.25, 7.5), (69.25, 28.0)), ((81.75, 7.5), (81.75, 28.0))]
        st, _ = weld_group(web_lines, FILLET_THROAT['tab'], Fy, Fz, 0.0, AXLE)
        path('Web fillets: to flange both sides + to 2 rungs', tag, st, CAP['weld_MPa'], 'MPa')
        # 3. bracket -> rail weld group: flange edge (y 62..88, z 6.75) + 2 rung edges (z 7.5..28)
        lines = [((62.0, 6.75), (88.0, 6.75)), ((69.25, 7.5), (69.25, 28.0)), ((81.75, 7.5), (81.75, 28.0))]
        st, info = weld_group(lines, FILLET_THROAT['tab'], Fy, Fz, 0.0, AXLE)
        path('Bracket-to-rail fillets (flange edge + 2 rungs)', tag, st, CAP['weld_MPa'], 'MPa')
        # 4. rail: axial + bending at the bracket's rear end (y 62), plastic interaction N/Np + M/Mp.
        # Down: the nose skid under the uprights is 1 mm over the floor, so the floor takes the push (upright ->
        # bottom brace -> skid), not the rails. Up: needs the downward-moving rear half of the rotor to hit, which sits
        # between the uprights inside the frame (front teeth move up: MuJoCo rotor sign) - reported, not designed for.
        if direction.startswith('down'):
            path('Rail at y 62 (down push grounded by the nose skid after 1 mm)', tag, 0.0, 1.0, 'ratio', 'floor reaction')
            continue
        tb = box_tube(10.0, rail_wall)
        Mr = abs(Fz * (AXLE[0] - 62.0) - Fy * (AXLE[1] - RAIL_Z))
        util = abs(Fy) / (tb['A'] * CAP['steel_fy_MPa']) + Mr / (tb['Zpl'] * CAP['steel_fy_MPa'])
        path('Rail 10x10x%.1f at y 62: N/Np + M/Mp (as load/capacity = util/1)' % rail_wall, tag, util, 1.0, 'ratio',
             'M %.0f N mm' % Mr + ('; non-credible direction for this rotor, accepted' if direction.startswith('up') else ''))
        if direction.startswith('up'):
            out['paths'][-1]['accepted_non_credible'] = True

# ---------------------------------------------------------------- drives: shaft collapse is the weak link by design
d, flat = 4.0, 0.5


def d_shaft_Zpl(d=4.0, flat=0.5, n=400):
    rr = d / 2
    ys = np.linspace(-rr, rr, n); dy = ys[1] - ys[0]
    zs = [z for y in ys for z in ys if y * y + z * z <= rr * rr and z <= rr - flat]
    zs = np.array(zs); zm = np.median(zs)
    return float(np.abs(zs - zm).sum() * dy * dy)


Zpl_shaft = d_shaft_Zpl()
lever = 67.5 - 56.0                       # wheel centre (|x| 59.5..75.5) to the gearbox face
F_cap = CAP['d_shaft_yield_MPa'][1] * Zpl_shaft / lever
m_motor = 0.110
x_face, x_clamp = 56.0, 10.0
couple = x_face - x_clamp
R_clamp = (F_cap * lever + m_motor * G * N_DROP * (x_face - 31.0)) / couple
R_face = F_cap + m_motor * G * N_DROP + R_clamp
for case in ('DT1 drop upright', 'DT2 drop inverted'):
    path('Motor face plate, 2 x M3 shear (shaft collapse + motor inertia)', case, R_face / 2, CAP['bolt_8.8_shear_N'])
    path('Motor face plate weld on the rail top (32 mm, one fillet)', case, R_face / (32 * FILLET_THROAT['tab']), CAP['weld_MPa'], 'MPa')
    tb = box_tube(10.0, 1.0)
    bar = 'Motor cross 10x10x1.0 (span 96, two pads at |x| 10)' if case.startswith('DT1') else 'Clamp bar 10x10x1.0 (cantilever 10 from the post)'
    Mbar = R_clamp * (48.0 - 10.0) if case.startswith('DT1') else R_clamp * 10.0
    path(bar + ', bending', case, Mbar / tb['Zpl'], CAP['steel_fy_MPa'], 'MPa')
    if case.startswith('DT2'):
        path('Clamp post 1.5 x 10 plate, tension (both motors)', case, 2 * R_clamp / (1.5 * 10.0), CAP['steel_fy_MPa'], 'MPa')
        path('Clamp post fillets (2 x 10 mm)', case, 2 * R_clamp / (2 * 10.0 * FILLET_THROAT['tab']), CAP['weld_MPa'], 'MPa')
out['drive'] = {'D_shaft_Zpl_mm3': round(Zpl_shaft, 2), 'lever_mm': lever, 'F_shaft_collapse_N': round(F_cap, 1),
                'R_clamp_N': round(R_clamp, 1), 'R_face_N': round(R_face, 1)}

# ---------------------------------------------------------------- whole frame in the drops: rails as beams on 2 supports
# the frame lands on the drive wheels (y 18) and the rear skid (y -102): everything behind the axle hangs on the rails
# over that 120 mm span (uniform, simply supported, both rails, PC floor ignored)
m_rear = sum(r['mass_g'] for r in rows if r['centroid_mm'][1] < 18.0) / 1000
tb = box_tube(10.0, rail_wall)
L_span = 18.0 + 102.0
M_rail = (m_rear * G * N_DROP / 2) * L_span / 8
path('Rail bending, mass behind the axle on the 120 mm wheel-to-skid span', 'DT1/DT2', M_rail / tb['Zpl'], CAP['steel_fy_MPa'], 'MPa',
     'm_rear %.3f kg; PC floor ignored' % m_rear)

design = [p for p in out['paths'] if 'check' not in p['case'] and not p.get('accepted_non_credible')]
out['summary'] = {'min_design_sf': min(p['sf'] for p in design if p['sf'] is not None),
                  'design_fails': [(p['path'], p['case'], p['sf']) for p in design if not p['pass']],
                  'accepted_non_credible': [(p['path'], p['case'], p['sf']) for p in out['paths'] if p.get('accepted_non_credible')],
                  'check_fails': [(p['path'], p['case'], p['sf']) for p in out['paths'] if 'check' in p['case'] and not p['pass']]}
# which stock wall would the rails need (fore-aft and up, design)
need = {}
for wall in (1.0, 1.2, 1.5, 2.0):
    tb = box_tube(10.0, wall)
    S = F_DESIGN * body_share / 2
    fa = S / (tb['A'] * 235) + S * (AXLE[1] - RAIL_Z) / (tb['Zpl'] * 235)
    up = S * (AXLE[0] - 62.0) / (tb['Zpl'] * 235)
    need['%.1f' % wall] = {'sf_fore_aft': round(1 / fa, 2), 'sf_up': round(1 / up, 2), 'g_per_rail_186mm': round(tb['A'] * 186 * 0.00785, 1)}
out['rail_wall_options'] = need
json.dump(out, open(os.path.join(HERE, 'frame_strength_calc.json'), 'w'), ensure_ascii=False, indent=1)
print('M %.3f kg, weapon module %.3f kg, body share %.3f, S(2.6 kN) per side %.0f N, rail wall %.1f' % (
    M, m_weapon, body_share, F_DESIGN * body_share / 2, rail_wall))
for p in out['paths']:
    print('%-5s %-70s %-30s load %8.1f cap %8.1f %-5s SF %s' % ('ok' if p['pass'] else 'FAIL', p['path'][:70], p['case'][:30], p['load'], p['capacity'], p['unit'], p['sf']))
print('rail wall options', json.dumps(need))
print('summary', json.dumps(out['summary'], ensure_ascii=False))
