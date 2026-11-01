"""R4-DIY strength: weapon hit (2.6 kN design, 5.2 kN check), 0.5 m drops (250 g), side hit, wedgelet, joint by joint.

Same load cases and method as r4_steel/calc/frame_strength_calc.py (W1), rebuilt for the DIY frame:
  * 12x12x1.2 tube everywhere (read from output/build_report.json, not hard-coded)
  * the weapon uprights are 3.2 mm steel welded to the frame (no M3 bolts, webs or flanges): upright -> base plate,
    upright foot -> bulkhead cross face, two rungs -> rail inner faces
  * welds are STICK (ARC) STITCHES, not continuous fillets: every joint uses the stitch lengths of WELD_PLAN, which the
    drawing package prints; fillet leg = the thinner part joined (1.6 sheet), throat 0.7 x leg
  * weapon motor mount 2.3 sheet on a foot strip + the bulkhead cross (the 250 g drop check that set its thickness)
  * wedgelet: bent ears on an M3 bolt pin
DT1/DT2 0.5 m drop, 2 mm stop -> 250 g. DT3 tooth force: S = F * (1 - m_weapon/M) / 2 per side, m_weapon = rotor stack +
uprights + braces + base plate + rungs (everything ahead of the welds checked). Required SF 1.5 design, 1.0 check.
Steel grades uncertified: capacities are ASSUMED handbook values for SS400 / E6013 (listed in 'capacities').
Run: python3 robot_v4/print_revision/r4_diy/calc/frame_strength_diy.py   (writes frame_strength_diy.json)
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DIY = os.path.dirname(HERE)
rep = json.load(open(os.path.join(DIY, 'output', 'build_report.json')))
rows = rep['rows']
M = rep['mass']['total_g'] / 1000
G = 9.81
N_DROP = 500.0 / 2.0
F_DESIGN, F_CHECK, SF_REQ = 2600.0, 5200.0, 1.5
F_SIDE = 0.2 * F_DESIGN          # glancing hit: 20 % of the tooth force sideways on one disc, ASSUMED (no W1 case)
CAP = {'steel_fy_MPa': 235.0, 'steel_fu_MPa': 400.0,                 # SS400 / STKR400 black tube
       'weld_MPa': 0.6 * 420.0,                                     # E6013 (Fu 420): 0.6 Fu on the throat
       'bolt_M3_4.8_shear_N': 0.6 * 400.0 * 5.03,                   # hardware-store M3 are class 4.8, not 8.8
       'bolt_M3_8.8_shear_N': 0.6 * 800.0 * 5.03, 'd_shaft_yield_MPa': (355.0, 640.0), 'al6061_shear_MPa': 159.0}
THROAT = 0.7 * 1.6               # every fillet here joins a 1.6 (or thicker) part to 1.2 tube or 1.6 sheet: leg 1.6
THROAT_TUBE = 0.7 * 1.2          # leg limited by the 1.2 tube wall
TUBE = 12.0
WALL = rep['tube_cut_list_mm'][0]['wall_mm']
AXLE = (95.0, 32.0)
RAIL_Z = 12.0                    # rail centroid (z 6..18)
UPR_T = 3.2
UPR_OUT = 30.2
RAIL_IN = 46.0


def g(prefixes):
    return sum(r['mass_g'] for r in rows if r['part'].startswith(prefixes)) / 1000


m_weapon = g(('Upright_Steel', 'Weapon_Top_Brace', 'Weapon_Base_Plate', 'Rung_', 'Nose_Skid', 'Beater_', 'Bearing_608', 'Spacer_',
              'Dead_Shaft', 'Weapon_Axle_', 'Belt_', 'rotor screws'))
body_share = (M - m_weapon) / M


def box_tube(side, wall):
    b, i = side, side - 2 * wall
    I = (b ** 4 - i ** 4) / 12
    return {'A': b * b - i * i, 'I': I, 'Zel': I / (b / 2), 'Zpl': (b ** 3 - i ** 3) / 4, 'Am': (b - wall) ** 2, 't': wall}


# Stitch plan (also printed on the drawing): (name, list of weld segments in the y-z plane as ((y0,z0),(y1,z1)) or
# ('pt', (y, z), length along x), throat). One upright side; both sides of a plate = two entries.
STITCH = 10.0                    # mm per stitch, ARC 2.0 rod
WELD_PLAN = {
    'upright_to_base': {'what': 'upright 3.2 bottom edge -> base plate, both faces, 3 stitches x 10 each face',
                        'segments': [((64.0, 7.6), (74.0, 7.6)), ((76.0, 7.6), (86.0, 7.6)), ((88.0, 7.6), (98.0, 7.6))] * 2},
    'foot_to_cross': {'what': 'upright foot (y 62) -> bulkhead cross front face, both faces, full 10.4',
                      'segments': [((62.0, 7.6), (62.0, 18.0))] * 2},
    'upright_to_rungs': {'what': 'each rung -> upright outer face, both faces of the rung, full 20.4',
                         'segments': [((69.3, 7.6), (69.3, 28.0))] * 2 + [((81.8, 7.6), (81.8, 28.0))] * 2},
    'base_to_cross': {'what': 'base plate front of cross, top fillet x 27..46 (one side) and x -27..27 shared',
                      'segments': [('pt', (62.0, 7.6), 19.0)]},
    'base_to_rail': {'what': 'base plate side arm -> rail inner face, top fillet, 2 stitches x 10 (y 63..73, 76..86)',
                     'segments': [((63.0, 7.6), (73.0, 7.6)), ((76.0, 7.6), (86.0, 7.6))]},
    'rungs_to_rail': {'what': 'each rung -> rail inner face, both faces of the rung, z 7.6..18',
                      'segments': [((69.3, 7.6), (69.3, 18.0))] * 2 + [((81.8, 7.6), (81.8, 18.0))] * 2},
}


def weld_group(segments, throat, Fy, Fz, Mx, ref):
    pts, w = [], []
    for seg in segments:
        if seg[0] == 'pt':
            pts.append(seg[1]); w.append(seg[2] * throat)
            continue
        a, b = seg
        L = math.dist(a, b)
        for k in range(100):
            t = (k + 0.5) / 100
            pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)); w.append(L / 100 * throat)
    pts, w = np.array(pts), np.array(w)
    A = w.sum()
    c = (pts * w[:, None]).sum(0) / A
    r = pts - c
    J = (w * (r ** 2).sum(1)).sum()
    M_c = Mx + Fz * (ref[0] - c[0]) - Fy * (ref[1] - c[1])
    ty = Fy / A - M_c * r[:, 1] / J
    tz = Fz / A + M_c * r[:, 0] / J
    return float(np.max(np.hypot(ty, tz))), {'throat_area_mm2': round(A, 1), 'centroid_yz': [round(v, 2) for v in c]}


out = {'inputs': {'M_kg': M, 'm_weapon_kg': round(m_weapon, 4), 'body_share': round(body_share, 4), 'tube': '12x12x%.1f' % WALL,
                  'N_drop_g': N_DROP, 'F_side_N': F_SIDE, 'capacities': CAP, 'weld_plan': {k: v['what'] for k, v in WELD_PLAN.items()}},
       'paths': []}


def path(name, case, load, cap, unit='N', note=''):
    sf = cap / load if load > 0 else None
    out['paths'].append({'path': name, 'case': case, 'load': round(load, 1), 'capacity': round(cap, 1), 'unit': unit,
                         'sf': None if sf is None else round(sf, 2), 'pass': sf is None or sf >= (1.0 if 'check' in case else SF_REQ), 'note': note})


tb = box_tube(TUBE, WALL)
# ---------------------------------------------------------------- DT3 weapon hit, per side
for F, case in ((F_DESIGN, 'DT3 2.6 kN'), (F_CHECK, 'DT3 5.2 kN check')):
    S = F * body_share / 2
    for direction, (Fy, Fz) in (('fore-aft (-y)', (-S, 0.0)), ('up (+z)', (0.0, S)), ('down (-z)', (0.0, -S))):
        tag = '%s %s' % (case, direction)
        path('Upright 3.2 bearing at the D8 axle hole', tag, S, 2.4 * 8.0 * UPR_T * CAP['steel_fu_MPa'])
        # upright section along its base (y 62..98 at z 7.6): N + M about x
        L = 98.0 - 62.0
        Mb = abs(Fy * (AXLE[1] - 7.6)) + abs(Fz * (AXLE[0] - (62.0 + 98.0) / 2))
        path('Upright 3.2 in-plane bending at its base (36 mm section)', tag, Mb / (UPR_T * L ** 2 / 6) + abs(Fz) / (UPR_T * L),
             CAP['steel_fy_MPa'], 'MPa')
        # welds holding the upright: to base plate + foot to cross + rungs (all on the upright)
        segs = WELD_PLAN['upright_to_base']['segments'] + WELD_PLAN['foot_to_cross']['segments'] + WELD_PLAN['upright_to_rungs']['segments']
        st, info = weld_group(segs, THROAT, Fy, Fz, 0.0, AXLE)
        path('Welds on the upright (base 6x10 stitches, foot to cross, rungs)', tag, st, CAP['weld_MPa'], 'MPa', json.dumps(info))
        # welds from the bracket (upright + base + rungs) into the tubes; the tube wall is the thinner part
        segs = WELD_PLAN['foot_to_cross']['segments'] + WELD_PLAN['base_to_cross']['segments'] + WELD_PLAN['base_to_rail']['segments'] \
            + WELD_PLAN['rungs_to_rail']['segments']
        st, info = weld_group(segs, THROAT_TUBE, Fy, Fz, 0.0, AXLE)
        path('Welds into the tubes (foot+base to cross, base+rungs to rail)', tag, st, CAP['weld_MPa'], 'MPa', json.dumps(info))
        if direction.startswith('down'):
            path('Rail at y 62 (down push grounded by the 5 mm nose skid under the base plate)', tag, 0.0, 1.0, 'ratio', 'floor reaction')
            continue
        Mr = abs(Fz * (AXLE[0] - 62.0) - Fy * (AXLE[1] - RAIL_Z))
        util = abs(Fy) / (tb['A'] * CAP['steel_fy_MPa']) + Mr / (tb['Zpl'] * CAP['steel_fy_MPa'])
        path('Rail 12x12x%.1f at y 62: N/Np + M/Mp (all of S through one rail)' % WALL, tag, util, 1.0, 'ratio',
             'M %.0f N mm' % Mr + ('; non-credible direction for this rotor, accepted' if direction.startswith('up') else ''))
        if direction.startswith('up'):
            out['paths'][-1]['accepted_non_credible'] = True
        if direction.startswith('fore'):
            # bulkhead cross torsion: the foot pushes its face 20 mm above the tube axis; the near rail end takes the
            # larger share (fixed-fixed torsion, upright |x| 28.6 between rails at |x| 46): rungs ignored
            share = (RAIL_IN + 28.6) / (2 * RAIL_IN)
            T = share * S * (AXLE[1] - RAIL_Z)
            path('Bulkhead cross torsion (all of S into the cross, rungs ignored)', tag, T / (2 * tb['Am'] * WALL), CAP['steel_fy_MPa'] / math.sqrt(3), 'MPa')
# side hit: 20 % of the tooth force sideways on one disc -> the spacer stack pushes one upright out of plane. The two
# rungs are in-plane cantilevers (depth 15.8 along x) from the base plate; the upright plate's own weak axis is ignored.
Ms = F_SIDE * (AXLE[1] - 7.6)
Zr = 2 * 1.6 * (RAIL_IN - UPR_OUT) ** 2 / 6
path('Side hit: 2 rungs as in-plane cantilevers (F_side at axle height)', 'side 0.2 x 2.6 kN', Ms / Zr, CAP['steel_fy_MPa'], 'MPa')
path('Side hit: rung fillets to the base plate (2 rungs x both faces x 15.8)', 'side 0.2 x 2.6 kN',
     Ms / (2 * 15.8 * THROAT * (1.6 + THROAT)) / (RAIL_IN - UPR_OUT) * 2, CAP['weld_MPa'], 'MPa', 'couple across each rung: arm t + throat')

# ---------------------------------------------------------------- weapon motor mount, drops
m_motor = next(r['mass_g'] for r in rows if r['part'].startswith('D3536')) / 1000 + 0.0136   # motor + pulley
offset = 42.0 - 23.5                                # mount face x -42 to the can centroid x -23.5
Mm = m_motor * G * N_DROP * offset
L_root, t_m = 40.0, 2.3                              # plate section y 22..62 at the shelf level (z ~19)
Z_gusset = 1.6 * 10.0 ** 2 / 6                       # 1.6 triangle, 10 mm tall at the plate, in-plane about y
for t_try, gz, note in ((1.6, 0.0, 'rejected'), (2.3, 0.0, 'rejected: SF just under 1.5 without the gusset'), (2.3, Z_gusset, 'selected')):
    path('Motor mount %.1f plate%s, bending at the shelf level (overhang %.1f mm)' % (t_try, ' + gusset' if gz else '', offset), 'DT1/DT2 drop',
         Mm / (L_root * t_try ** 2 / 6 + gz), CAP['steel_fy_MPa'], 'MPa', note + (' thickness/option, reported for the record' if note != 'selected' else ''))
# Only welds a stick electrode reaches: the mount's outer face is 1.7 mm from the rail, so the plate + foot + gusset are
# welded on the bench (both faces), the foot goes to the rail from underneath (frame turned over, before the floor),
# the shelf is welded from above. The moment is a couple between the shelf (z 18.8) and the foot (z 6.8).
F_couple = Mm / (18.8 - 6.8)
path('Mount: shelf -> plate outer face, one fillet from above (27 mm)', 'DT1/DT2 drop', F_couple / (27.0 * THROAT), CAP['weld_MPa'], 'MPa')
path('Mount: shelf -> rail top, fillet along the outer edge (27 mm)', 'DT1/DT2 drop', F_couple / (27.0 * THROAT_TUBE), CAP['weld_MPa'], 'MPa')
path('Mount: foot -> rail inner face, one fillet from underneath (28 mm)', 'DT1/DT2 drop', F_couple / (28.0 * THROAT_TUBE), CAP['weld_MPa'], 'MPa')
path('Mount: plate -> foot, bench fillets both faces (28 mm)', 'DT1/DT2 drop', F_couple / (2 * 28.0 * THROAT), CAP['weld_MPa'], 'MPa')

# ---------------------------------------------------------------- side wall under a skirt push (not in W1's calc; W1 has the same span)
# PC 2 mm wall (h 55) between its two wall tabs (y -102 .. 56); the skirt brackets bolt through it; the top edge only
# rests under the lid (angles at the corners). Load: an opponent pushing the skirt inward with about our own push.
E_PC, L_wall, h_wall, t_wall = 2400.0, 158.0, 55.0, 2.0
P_side = 8.0
I_wall = h_wall * t_wall ** 3 / 12
defl = P_side * L_wall ** 3 / (48 * E_PC * I_wall)
path('Side wall PC 2 mm, inward deflection at mid-span under a %.0f N skirt push' % P_side, 'skirt push check', defl, 3.0, 'mm',
     'span 158 mm between wall tabs, simply supported; 3 mm keeps the skirt on the floor: OPEN ISSUE (W1 has the same span)')
path('Side wall PC 2 mm, bending stress, same push', 'skirt push check', P_side * L_wall / 4 / (h_wall * t_wall ** 2 / 6), 60.0, 'MPa')

# ---------------------------------------------------------------- drives: shaft collapse is the weak link by design
def d_shaft_Zpl(d=4.0, flat=0.5, n=400):
    rr = d / 2
    ys = np.linspace(-rr, rr, n); dy = ys[1] - ys[0]
    zs = np.array([z for y in ys for z in ys if y * y + z * z <= rr * rr and z <= rr - flat]); zm = np.median(zs)
    return float(np.abs(zs - zm).sum() * dy * dy)


Zpl_shaft = d_shaft_Zpl()
lever = 67.5 - 56.0
F_cap = CAP['d_shaft_yield_MPa'][1] * Zpl_shaft / lever
m_dm = 0.110
x_face, x_clamp = 56.0, 10.0
R_clamp = (F_cap * lever + m_dm * G * N_DROP * (x_face - 31.0)) / (x_face - x_clamp)
R_face = F_cap + m_dm * G * N_DROP + R_clamp
for case in ('DT1 drop upright', 'DT2 drop inverted'):
    path('Motor face plate, 2 x M3 4.8 shear', case, R_face / 2, CAP['bolt_M3_4.8_shear_N'])
    path('Motor face plate on the rail top: 2 stitches x 10 + ends (32 mm)', case, R_face / (32 * THROAT_TUBE), CAP['weld_MPa'], 'MPa')
    Mbar = R_clamp * (RAIL_IN - 10.0) if case.startswith('DT1') else R_clamp * 10.0
    bar = 'Motor cross 12x12 (span 92, pads at |x| 10)' if case.startswith('DT1') else 'Clamp bar 12x12 (cantilever 10 from the post)'
    path(bar + ', bending', case, Mbar / tb['Zpl'], CAP['steel_fy_MPa'], 'MPa')
    if case.startswith('DT2'):
        path('Clamp post 1.6 x 12 plate, tension (both motors)', case, 2 * R_clamp / (1.6 * 12.0), CAP['steel_fy_MPa'], 'MPa')
        path('Clamp post fillets (2 x 12 mm, into tube walls)', case, 2 * R_clamp / (2 * 12.0 * THROAT_TUBE), CAP['weld_MPa'], 'MPa')
out['drive'] = {'F_shaft_collapse_N': round(F_cap, 1), 'R_clamp_N': round(R_clamp, 1), 'R_face_N': round(R_face, 1)}

# ---------------------------------------------------------------- wedgelet hinge (ears + M3 bolt), R3 lifter load and a ram
R3_LIFTER_N = 111.36                                 # r3/calc/wedgelet_tap_fix_calc.py
for Fw, case in ((R3_LIFTER_N, 'wedgelet: R3 lifter at the tip'), (500.0, 'wedgelet: 500 N ram on the tip, check')):
    per_ear = Fw / 2
    path('Ear 1.6 bearing at the D3.2 hole', case, per_ear, 2.4 * 3.0 * 1.6 * CAP['steel_fu_MPa'])
    path('Ear tear-out behind the hole (edge 1.8 mm, 2 planes)', case, per_ear, 2 * 1.8 * 1.6 * 0.6 * CAP['steel_fu_MPa'])
    path('M3 4.8 pin, shear (4 shear planes at the 2 ears)', case, Fw / 4, CAP['bolt_M3_4.8_shear_N'])
    path('Hinge tab 1.6 bearing (3 tabs)', case, Fw / 3, 2.4 * 3.0 * 1.6 * CAP['steel_fu_MPa'])

# ---------------------------------------------------------------- whole frame in the drops: rails over the wheel-to-skid span
m_rear = sum(r['mass_g'] for r in rows if r['centroid_mm'][1] < 18.0) / 1000
L_span = 18.0 + 102.0
M_rail = (m_rear * G * N_DROP / 2) * L_span / 8
path('Rail bending, mass behind the axle on the 120 mm wheel-to-skid span', 'DT1/DT2', M_rail / tb['Zpl'], CAP['steel_fy_MPa'], 'MPa',
     'm_rear %.3f kg; PC floor ignored' % m_rear)

design = [p for p in out['paths'] if 'check' not in p['case'] and not p.get('accepted_non_credible') and 'rejected' not in p['note']]
out['summary'] = {'min_design_sf': min(p['sf'] for p in design if p['sf'] is not None),
                  'design_fails': [(p['path'], p['case'], p['sf']) for p in design if not p['pass']],
                  'accepted_non_credible': [(p['path'], p['case'], p['sf']) for p in out['paths'] if p.get('accepted_non_credible')],
                  'check_fails': [(p['path'], p['case'], p['sf']) for p in out['paths'] if 'check' in p['case'] and not p['pass']]}
json.dump(out, open(os.path.join(HERE, 'frame_strength_diy.json'), 'w'), ensure_ascii=False, indent=1)
print('M %.3f kg, weapon side %.3f kg, body share %.3f, S(2.6 kN) per side %.0f N' % (M, m_weapon, body_share, F_DESIGN * body_share / 2))
for p in out['paths']:
    print('%-5s %-72s %-34s load %8.1f cap %8.1f %-5s SF %s' % ('ok' if p['pass'] else 'FAIL', p['path'][:72], p['case'][:34], p['load'], p['capacity'], p['unit'], p['sf']))
print('summary', json.dumps(out['summary'], ensure_ascii=False))
