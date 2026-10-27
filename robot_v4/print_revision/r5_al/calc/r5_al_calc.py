"""V4 R5-AL checks: what changed when steel became aluminium and lathe parts became prints/bought parts.

Same load cases as r4_diy/calc/frame_strength_diy.py (DT3 weapon hit 2.6 kN design / 5.2 kN check, 0.5 m drop 250 g,
SF 1.5 design, 1.0 check), applied to what R5-AL changed:
  * frame rail: 12.7 x 12.7 x 2.0 6063-T5 aluminium instead of 12 x 12 x 1.2 SS400 (R4-DIY's N and M per case,
    rescaled to R5's body share)
  * every R4-DIY weld on the weapon path is now a bolted joint: bolt-group check (M3 8.8 shear, bearing in the 2 mm
    aluminium wall, thread in the tapped 6 mm 6061 upright)
  * rotor: 3/4" steel pipe welded to the discs (torsion + weld), energy with the pipe instead of the 6061 hub
  * printed PETG parts that replace turned 6061: spacers, motor pulley, wheel hubs
  * PC side wall with the mid-wall support (R4-DIY open issue)
  * material table: steel / stainless / aluminium per part, with the governing number
Aluminium: 6063-T5 (Thai architectural stock) MINIMUM fy 110, fu 150 MPa; 6061-T6 fy 240, fu 260; 5052-H32 fu 210.
Run: python3 robot_v4/print_revision/r5_al/calc/r5_al_calc.py   (writes r5_al_calc.json)
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R5 = os.path.dirname(HERE)
PR = os.path.dirname(R5)
rep = json.load(open(os.path.join(R5, 'output', 'build_report.json')))
r4s = json.load(open(os.path.join(PR, 'r4_diy', 'calc', 'frame_strength_diy.json')))
r4b = json.load(open(os.path.join(PR, 'r4_diy', 'output', 'build_report.json')))
v4 = json.load(open(os.path.join(PR, '..', 'v4_design.json')))
r3 = json.load(open(os.path.join(PR, 'r3', 'output_R3', 'build_report.json')))
rows = rep['rows']
M = rep['mass']['total_g'] / 1000
SF_REQ = 1.5
FY_AL, FU_AL = 110.0, 150.0          # 6063-T5 minimum (EN 755-2 T5: Rp0.2 110, Rm 150)
GRADES = {'6063-T5': (110.0, 150.0), '6063-T6': (170.0, 215.0), '6061-T6': (240.0, 260.0)}   # EN 755-2 minimum Rp0.2, Rm
TUBE_GRADE = '6063-T6'              # chosen below: the lowest grade that passes the 5.2 kN check fore-aft
FU_6061, FU_5052, TAU_6061 = 260.0, 210.0, 0.6 * 260.0
BOLT_M3_88 = 0.6 * 800 * 5.03        # 2414 N single shear on the thread root
BOLT_M3_A2 = 0.6 * 700 * 5.03        # 2113 N (A2-70)
BOLT_M4_88 = 0.6 * 800 * 8.78        # 4214 N
out = {'inputs': {'M_kg': M, 'aluminium_6063_T5_min_MPa': [FY_AL, FU_AL]}, 'checks': [], 'materials': []}


def chk(name, case, load, cap, unit, note=''):
    sf = cap / load if load else None
    out['checks'].append({'check': name, 'case': case, 'load': round(load, 2), 'capacity': round(cap, 2), 'unit': unit,
                          'sf': None if sf is None else round(sf, 2),
                          'pass': sf is None or sf >= (1.0 if 'check' in case else SF_REQ), 'note': note})
    return sf


def g(prefixes):
    return sum(r['mass_g'] for r in rows if r['part'].startswith(prefixes)) / 1000


# ---------------------------------------------------------------- tooth load share per side (same rule as R4-DIY)
m_weapon = g(('Upright_6061', 'Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Nose_Skid', 'Beater_', 'Bearing_608', 'Spacer_',
              'Dead_Shaft', 'Weapon_Axle_', 'Belt_', 'Rotor_Pipe', 'rotor screws', 'Rung_Al', 'Upright_Foot'))
share = (M - m_weapon) / M
out['inputs'].update({'m_weapon_kg': round(m_weapon, 4), 'body_share': round(share, 4)})


# ---------------------------------------------------------------- rail: aluminium vs R4-DIY steel, same N + M per case
def tube_props(b, t):
    i = b - 2 * t
    return {'A': b * b - i * i, 'Zp': (b ** 3 - i ** 3) / 4}


al = tube_props(12.7, 2.0)
st = tube_props(12.0, 1.2)
s_r4 = r4s['inputs']['body_share']
for p in r4s['paths']:
    if not p['path'].startswith('Rail 12x12x1.2 at y 62'):
        continue
    F = 2600.0 if '2.6' in p['case'] else 5200.0
    S = F * share / 2
    Mr = float(p['note'].split('M ')[1].split(' N mm')[0]) * share / s_r4
    credible = 'up' not in p['case']
    for gname, (fy, _fu) in GRADES.items():
        ratio = S / (al['A'] * fy) + Mr / (al['Zp'] * fy)
        tag = p['case'] + ('' if credible else ' [non-credible direction]')
        if gname != TUBE_GRADE:
            out.setdefault('rail_grade_sweep', []).append({'grade': gname, 'case': tag, 'sf': round(1 / ratio, 2)})
            continue
        chk('Rail 12.7x12.7x2.0 %s at the cross: N/Np + M/Mp' % gname, tag, ratio, 1.0, 'ratio',
            'N and M BORROWED from R4-DIY (weld at y 62), rescaled to R5 body share; R5 loads the rail at the rung (y 72-85): approximate. R4-DIY steel same case SF %.2f' % p['sf'])
out['rail_section'] = {'al_12.7x2.0': al, 'steel_12x1.2': st, 'Mp_al_Nmm': al['Zp'] * FY_AL, 'Mp_steel_Nmm': st['Zp'] * 235.0,
                       'mass_g_per_mm_al': al['A'] * 2.7e-3, 'mass_g_per_mm_steel': st['A'] * 7.85e-3}

# ---------------------------------------------------------------- weapon joints (were welds): bolt group in the y-z plane
# Bolt positions come from the CAD (build_report weapon_bolts_yz): per side 3 x M4 8.8 along x into the tapped 6 mm
# upright: through the solid 12.7 rung bar (mid-height), the riser angle's vertical leg and the foot angle's leg B.
# The rung bar is solid, so the rung bolt is supported and sees shear at the upright face (no 13 mm free span).
WB = [(y, z) for side, _n, y, z in rep['weapon_bolts_yz'] if side == 'R']
ANG_T = rep['weapon_angle_t_mm']
FU_T = GRADES[TUBE_GRADE][1]
d4 = 4.0


def tearout(e, t, fu):
    """Block tear-out of one bolt toward a free edge at distance e (to the hole centre): 2 shear planes."""
    return 2 * max(e - d4 / 2, 0) * t * 0.6 * fu


# edge distances (mm) per bolt and direction, from the CAD: upright bottom z 7, rear edge y 64; angles' edges
UPR_BOTTOM, UPR_REAR = 7.0, 64.0
leg_b = rep['foot_leg_b_y']
per_bolt_cap = []
for (y, z) in WB:
    caps = {'bolt shear M4 8.8': BOLT_M4_88,
            'thread in 6 mm 6061': 0.5 * math.pi * d4 * 6 * TAU_6061,
            'upright tear-out down to z 7 (up load)': tearout(z - UPR_BOTTOM, 6.0, FU_6061),
            'upright tear-out to rear edge y 64 (+y recoil)': tearout(y - UPR_REAR, 6.0, FU_6061)}
    if abs(y - sum(leg_b) / 2) < 0.5:                       # foot angle leg B (3 mm 6061)
        caps['foot leg B tear-out (3 mm 6061)'] = tearout(min(y - leg_b[0], leg_b[1] - y), ANG_T, FU_6061)
        caps['foot leg B bearing'] = 2.4 * d4 * ANG_T * FU_6061
    elif z > 25:                                            # riser vertical leg (3 mm 6061), top at z 36
        caps['riser leg tear-out to its top (z 36)'] = tearout(36.0 - z, ANG_T, FU_6061)
        caps['riser leg bearing'] = 2.4 * d4 * ANG_T * FU_6061
    else:                                                   # rung: rail walls 2.0 (double) + solid bar
        caps['rail walls bearing (2 walls)'] = 2 * 2.4 * d4 * 2.0 * FU_T
        caps['rail wall tear-out to its corner (z 6 / 18.7), 2 walls'] = 2 * tearout(z - 6.0, 2.0, FU_T)
    gov = min(caps, key=caps.get)
    per_bolt_cap.append((caps[gov], gov, caps))
nb = len(WB)
cy = sum(b[0] for b in WB) / nb
cz = sum(b[1] for b in WB) / nb
J = sum((b[0] - cy) ** 2 + (b[1] - cz) ** 2 for b in WB)
for F, case in ((2600.0, 'DT3 2.6 kN'), (5200.0, 'DT3 5.2 kN check')):
    S = F * share / 2
    for d, (Fy, Fz) in (('fore-aft (-y)', (-S, 0.0)), ('up (+z)', (0.0, S)), ('down (-z)', (0.0, -S))):
        Mx = Fz * (95.0 - cy) - Fy * (32.0 - cz)
        worst = None
        for (by, bz), (cap, gov, _c) in zip(WB, per_bolt_cap):
            fy = Fy / nb - Mx * (bz - cz) / J
            fz = Fz / nb + Mx * (by - cy) / J
            r = math.hypot(fy, fz) / cap
            if worst is None or r > worst[0]:
                worst = (r, math.hypot(fy, fz), cap, gov, (by, bz))
        chk('Upright -> frame bolt group (3 x M4 8.8 per side, worst bolt)', '%s %s' % (case, d), worst[1], worst[2], 'N',
            'bolt at y %.1f z %.1f; governed by %s' % (worst[4][0], worst[4][1], worst[3]))
out['weapon_bolt_group'] = {'bolts_yz': WB, 'centroid_yz': [round(cy, 2), round(cz, 2)], 'J_mm2': round(J, 1),
                            'per_bolt_capacity_N': [{k: round(v) for k, v in c[2].items()} for c in per_bolt_cap]}
# foot leg A: 2 x M3 rivnuts in the cross front face carry the foot's share; take the worst bolt force of the group
for F, case in ((2600.0, 'DT3 2.6 kN'), (5200.0, 'DT3 5.2 kN check')):
    S = F * share / 2
    chk('Foot leg A -> cross, 2 x M3 rivnut (takes up to S/2, worst case)', case, S / 2 / 2,
        min(BOLT_M3_88, 2.4 * 3 * 2.0 * FU_T, 1500.0), 'N', 'M3 steel rivnut pull/shear in a 2 mm wall ~1.5 kN, ASSUMED')
# drive-motor tab: 250 g drop of the JB361 (~45 g ASSUMED) + wheel (hub ~13 g PETG + O-rings 12 g) on 2 x M3 legs
chk('Drive-motor tab legs, 2 x M3 rivnut, 0.5 m drop 250 g', 'DT1 drop', (0.045 + 0.025) * 9.81 * 250 / 2, 1500.0, 'N')
# wedgelet hinge tabs: a wedgelet hit (0.2 x 2.6 kN, ASSUMED) shared by 3 tabs per side; leg M3 at e 2.25 in 2 mm 5052
F_tab = 0.2 * 2600.0 / 3
chk('Hinge tab leg, 1 x M3 rivnut (e 2.25 mm in 2.0 5052), wedgelet hit / 3', 'DT3 wedgelet', F_tab,
    min(2 * (2.25 - 1.5) * 2.0 * 0.6 * FU_5052 + 2.4 * 3 * 2.0 * FU_5052 * 0, 1500.0) if False else 2 * (2.25 - 1.5) * 2.0 * 0.6 * FU_5052 * 2,
    'N', 'leg tear-out both ways around the hole; the tab also bears on the beam face in compression')
chk('Hinge tab pin bearing (2.0 5052, M3 pin)', 'DT3 wedgelet', F_tab, 2.4 * 3 * 2.0 * FU_5052, 'N')
# frame corner joints: rail -> bulkhead cross carries S into the body; 2 cleats x 2 M3 per corner, bearing in the 2.0 wall
for F, case in ((2600.0, 'DT3 2.6 kN'), (5200.0, 'DT3 5.2 kN check')):
    S = F * share / 2
    chk('Frame corner rail->cross (2 cleats x 2 M3, worst bolt = S/2)', case, S / 2, 2.4 * 3 * 2.0 * GRADES[TUBE_GRADE][1], 'N')
# motor mount M06 (6061 3 mm, R3 part) on two Al angles: 250 g drop of motor + mount, 4 bolts
m_mot = (102 + 13) / 1000
chk('M06 angles: 0.5 m drop, 250 g, 4 x M3 (worst bolt = 1/2 with a 2x moment factor)', 'DT1 drop', m_mot * 9.81 * 250 / 2,
    2.4 * 3 * 2.0 * FU_AL, 'N')

# ---------------------------------------------------------------- PC side wall with the mid-wall support
R4_DEFL, R4_SPAN = 7.5, 158.0
spans = [round(rep['midwall_y'] - (-110.0 + 2.0 + 12.7), 1), round(50.0 - (rep['midwall_y'] + 12.7), 1)]
for sp in spans:
    chk('PC 2 mm side wall deflection, 8 N skirt push (span %.1f mm)' % sp, 'skirt push', R4_DEFL * (sp / R4_SPAN) ** 3, 3.0, 'mm',
        'R4-DIY span 158 mm gave 7.5 mm (SF 0.4); deflection ~ span^3')

# ---------------------------------------------------------------- rotor: pipe + welds + energy
T_hit = v4['shaft_check']['peak_force_N_assumed_1ms'] * 29.5e-3 / 2       # half the tooth torque crosses to the other disc
ro, ri = 26.7 / 2, 22.1 / 2
Jp = math.pi / 2 * (ro ** 4 - ri ** 4)
chk('Rotor pipe 3/4" (OD 26.7 x 2.3) torsion, half the tooth torque', 'DT3 1 ms model', T_hit * 1e3 * ro / Jp, 0.58 * 235, 'MPa')
a_throat = 0.7 * 2.3
chk('Pipe -> disc fillet weld (leg 2.3) around OD 26.7', 'DT3 1 ms model', T_hit * 1e3 / (2 * math.pi * ro ** 2 * a_throat), 0.6 * 420, 'MPa')
# energy: R3 rotor inertia with the 6061 hub -> with the steel pipe; belt root D26.7 vs R3 groove root D27 (1:1 pulley D30)
m_hub = 11949 * 2.7e-6
m_pipe = math.pi / 4 * (26.7 ** 2 - 22.1 ** 2) * 40 * 7.85e-6
I_hub = m_hub / 2 * ((15e-3) ** 2 + (11e-3) ** 2)
I_pipe = m_pipe / 2 * ((ro * 1e-3) ** 2 + (ri * 1e-3) ** 2)
I3 = r3['mass']['rotor_inertia_kg_m2']
I5 = I3 - I_hub + I_pipe
ratio_speed = 27.0 / 26.7
E4 = v4['chosen']['energy_J']
out['rotor'] = {'T_half_hit_Nm': round(T_hit, 1), 'pipe_mass_g': round(m_pipe * 1000, 1), 'hub_6061_mass_g': round(m_hub * 1000, 1),
                'I_R3_kgm2': I3, 'I_R5_kgm2': I5, 'rotor_speed_ratio_vs_R3': round(ratio_speed, 4),
                'energy_J_operating': round(E4 * I5 / I3 * ratio_speed ** 2, 1), 'energy_J_R4_DIY': E4,
                'why_not_printed_hub': 'a PETG core between the discs would carry ~%.0f N m: torsion of a D30/D11 PETG core %.0f MPa vs ~30 MPa, '
                                       'and 4 x M4 bolts across a 40 mm gap would bend (M ~ %.0f N m vs ~3.6 N m) -> buy 3/4" pipe and weld instead'
                                       % (T_hit, T_hit * 1e3 * 15 / (math.pi / 32 * (30 ** 4 - 11 ** 4)), T_hit / 4 / 0.013 * 0.020)}

# ---------------------------------------------------------------- printed PETG parts that replace turned 6061
A_in = math.pi / 4 * (11 ** 2 - 8.3 ** 2)
chk('PETG inner shaft spacer, side hit 0.2 x 2.6 kN axial', 'DT3 side', 0.2 * 2600 / A_in, 55.0, 'MPa', 'nylock snug only (<= 300 N preload: %.1f MPa)' % (300 / A_in))
Kt = 60 / (2 * math.pi * 1250)
chk('PETG motor pulley on D5 shaft: grub M3 on a filed flat', 'ESC 40 A peak', Kt * 40, 1.0, 'N m', 'belt slips first; PETG Tg ~80 C: use ASA/PETG-CF if the can runs > 60 C')
F_wheel = 3.99144375
chk('PETG wheel hub D-bore (4 mm, flat 0.5)', 'drive stall', F_wheel * 0.032 / 1.5e-3 / (3.0 * 18.0), 40.0, 'MPa')

# ---------------------------------------------------------------- material table
MAT = [
    ('Frame tubes', 'SS400 12x12x1.2 (welded)', 'Al %s 12.7x12.7x2.0 (bolted); 6063-T5 fails the 5.2 kN check (see rail_grade_sweep)' % TUBE_GRADE,
     'rail check SF in checks; %.0f%% lighter per mm' % (100 * (1 - al['A'] * 2.7 / (st['A'] * 7.85)))),
    ('Tabs / angles', 'SS400 1.6 (welded)', 'Al 5052/6061 2.0 (bolted)', 'bearing per M3 %.0f N in 2.0 Al' % (2.4 * 3 * 2 * FU_AL)),
    ('Weapon uprights / braces / motor mount', 'SS400 3.2 / 1.6 / 2.3 (welded)', '6061 6 / 3 / 3 mm (R3 parts, bolted)',
     'R3 design already checked; no welding of aluminium needed'),
    ('Weapon discs', 'SS400 6 mm', 'STEEL kept (SS400 or Hardox 450)', '7.85 g/cm3 is what stores the energy; 304 stainless is softer (HB ~200 vs Hardox 450) and wears the teeth'),
    ('Rotor hub', '6061 turned (lathe)', 'STEEL 3/4" pipe welded to the discs', 'no lathe; aluminium cannot be welded to the steel discs; see rotor checks'),
    ('Dead shaft M8', '12.9', 'STEEL 12.9 kept', 'bending %.0f MPa: 12.9 (1080) SF %.2f; stainless A4-80 (600) SF %.2f -> stainless worse'
     % (v4['shaft_check']['bending_MPa'], 1080 / v4['shaft_check']['bending_MPa'], 600 / v4['shaft_check']['bending_MPa'])),
    ('Wedgelets', 'SS400 1.6', 'STEEL SS400 1.6 kept (304 1.5 if available)', 'thin hard edge; 5052 1.6 (fy ~160) dents and rolls; 304 is equal strength + no rust'),
    ('Wedgelet hinge pins', 'M3 8.8', 'STAINLESS A2-70 M3x55', 'shear %.0f N vs 8.8 %.0f N, load < 0.5 kN: stainless is better here (no rust in the pivot)' % (BOLT_M3_A2, BOLT_M3_88)),
    ('Skin / skirt screws', 'zinc steel', 'STAINLESS A2', 'low load; no rust'),
    ('Shaft spacers, motor pulley, wheel hubs', '6061 turned (lathe)', 'PETG printed 100 % (pulley ASA if hot)', 'see printed-part checks'),
    ('Skins', 'PC 2 mm', 'PC 2 mm (unchanged)', 'mid-wall support fixes the deflection'),
]
out['materials'] = [{'part': a, 'R4_DIY': b, 'R5_AL': c, 'governing_number': d} for a, b, c, d in MAT]
out['mass'] = {'R5_AL_g': rep['mass']['total_g'], 'R4_DIY_g': r4b['mass']['total_g'],
               'delta_g': round(rep['mass']['total_g'] - r4b['mass']['total_g'], 1), 'groups_g': rep['mass']['groups_g'],
               'margin_to_2000_g': rep['mass']['margin_to_2000_g']}
out['summary'] = {'design_fails': [(c['check'], c['case'], c['sf']) for c in out['checks'] if not c['pass'] and 'check' not in c['case']],
                  'check_fails': [(c['check'], c['case'], c['sf']) for c in out['checks'] if not c['pass'] and 'check' in c['case']],
                  'min_design_sf': min(c['sf'] for c in out['checks'] if c['sf'] and 'check' not in c['case'] and 'non-credible' not in c['case'])}
json.dump(out, open(os.path.join(HERE, 'r5_al_calc.json'), 'w'), indent=1, ensure_ascii=False)
for c in out['checks']:
    print('%-6s SF %-6s %-70s %s' % ('ok' if c['pass'] else 'FAIL', c['sf'], c['check'][:70], c['case']))
print('rotor', out['rotor'])
print('mass', out['mass'])
print('summary', out['summary'])
