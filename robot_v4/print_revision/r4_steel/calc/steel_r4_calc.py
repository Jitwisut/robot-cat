"""V4 R4-steel concept: welded steel frame, no 3D-printed parts. What has to be cut to stay under 2000 g?

Method (mass budget from fixed mass, not from the PC-body variant):
  1. Every R3 part (output_R3/build_report.json) is FIXED unless it is printed (name has _TPU/_PETG) or is the TPU body.
  2. Printed parts get bought / CNC replacements (wheels, PC electronics plate, nylon standoffs, ...).
  3. A welded steel frame (square tube) + steel tabs + PC skins replaces the body.
  4. Cut options are applied one at a time and in combinations; each reports mass, CG y, axle loads, push and weapon energy.
Weapon energy uses the R3 rotor numbers (r3/calc/r3_calc.json): each 6 mm disc holds disc_share_of_rotor_I of the inertia.
Push uses the R3 drive numbers: JGA25-370 400 rpm motor-limited at motor_limit_per_wheel_N at 12.6 V.
Coordinates: +Z up, nose +Y, mm (AGENTS.md). Nothing here changes R3 files.
Run: python3 robot_v4/print_revision/r4_steel/calc/steel_r4_calc.py   (writes steel_r4_calc.json)
"""
import itertools
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R3 = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'r3')
report = json.load(open(os.path.join(R3, 'output_R3', 'build_report.json')))
r3calc = json.load(open(os.path.join(R3, 'calc', 'r3_calc.json')))
pc = json.load(open(os.path.join(R3, 'pc_body', 'output', 'pc_body_report.json')))

LIMIT = 2000.0
TARGET = 1950.0                     # keep >= 50 g for weld beads, paint, wire length and scale error until parts are weighed
FRONT_Y, REAR_Y = 18.0, -70.0       # R3 axles
REAR_SKID_Y = -110.0                # 2WD option: UHMW skid under the rear wall
NOSE_SKID_Y = 69.0                  # R3 Nose_Skid_UHMW_3mm (z 2.5): 2WD front-heavy case rests on wheels + this skid
RHO_STEEL, RHO_PC = 7.85e-3, 1.20e-3   # g/mm^3
MU = 0.6                            # tyre on plastic/tile, ASSUMED middle of the r3_calc 0.4-0.8 range
G = 9.81
SRC = {}


def src(name, value, tag, note=''):
    SRC[name] = {'value': value, 'tag': tag, 'note': note}
    return value


# ------------------------------------------------------------------ 1. fixed R3 parts
parts = report['mass']['parts']
BODY = ('Tub_TPU', 'Lid_TPU_2mm')
printed = [p for p in parts if ('_TPU' in p['part'] or '_PETG' in p['part']) and p['part'] not in BODY]
fixed = [p for p in parts if p not in printed and p['part'] not in BODY]
items = [(p['part'], p['mass_g'], p['centroid_mm'][1]) for p in fixed]

# ------------------------------------------------------------------ 2. replacements for printed parts
WHEEL_G = src('bought_wheel_g', 45.0, 'ASSUMED', '65 mm rubber wheel + 4 mm D hex coupler for JGA25; no Thai listing with mass found; sensitivity +-15 g')
WHEEL_DIA = src('bought_wheel_dia_mm', 65.0, 'ASSUMED', 'R3 wheel is 64 mm; same push model')
repl = [
    ('wheel_bought_Front_L', WHEEL_G, FRONT_Y), ('wheel_bought_Front_R', WHEEL_G, FRONT_Y),
    ('wheel_bought_Rear_L', WHEEL_G, REAR_Y), ('wheel_bought_Rear_R', WHEEL_G, REAR_Y),
    ('electronics_plate_PC2mm_CNC 122x54.5', 122 * 54.5 * 2 * RHO_PC, -29.5),
    ('nylon_standoffs_M3x20 x4', 4 * 0.6, -30.0),
    ('hall_bracket_al_1.5mm', 2.0, 58.0),
    ('switch_panel_mount_bought', 3.0, -30.0),
    ('rear_skid_UHMW_3mm (4WD: none)', 0.0, REAR_SKID_Y),
]
src('replacements', [r[0] for r in repl], 'ASSUMED', 'battery tray, wedgelet blocks/carriers and weapon mount feet become frame tabs')

# ------------------------------------------------------------------ 3. welded steel frame + skins
TUBE = src('tube_10x10_wall_mm', 1.0, 'ASSUMED', 'minimum wall for MIG on thin tube; confirm with the welding shop (0.8 only with TIG)')


def tube_g_per_mm(wall, side=10.0):
    return (side ** 2 - (side - 2 * wall) ** 2) * RHO_STEEL


# members on the PC-body box (x +-83, y -110..97, z 4..62); no ring member in front of the bulkhead: the discs sweep z 2.5 at y 95
MEMBERS = {
    'side_rails 2x207 (y -110..97)': (2 * 207, -6.5),
    'rear_cross 160': (160, -105.0),
    'bulkhead_cross 160 (y 62)': (160, 62.0),
    'posts 4x48 (rear corners + bulkhead)': (4 * 48, (-105 - 105 + 62 + 62) / 4),
    'top_rear_cross + top_bulkhead_cross (lid clamp)': (2 * 160, (-105 + 62) / 2),
}
TAB_T = src('tab_thickness_mm', 1.5, 'ASSUMED', 'mild steel plate, laser cut, welded to the tubes')
TABS = {   # name: (count, area mm^2 each, y) sized from the interfaces they carry
    'drive_motor_face_plates 32x32 - D8.5 (JGA25 gearbox face)': (4, 32 * 32 - 57, None),
    'upright_plates 40x45 (bolt the 6061 uprights)': (2, 40 * 45, 80.0),
    'weapon_motor_mount_tabs 20x25 (M06 bolts on)': (2, 20 * 25, 43.0),
    'wedgelet_hinge_tabs 15x20': (4, 15 * 20, 102.0),
    'battery/electronics/skin tabs 12x15': (10, 12 * 15, -30.0),
}
RIVNUTS = src('rivnut_M3_plus_screw_g', (24, 1.4), 'ASSUMED', '24 M3 steel rivnuts (0.9 g) + screws (0.5 g) hold the PC skins; tube too thin to tap')
WELD = src('weld_bead_paint_fraction', 0.04, 'ASSUMED', 'of frame + tabs')
PANELS = {p['file'][:-4]: (p['mass_g'], p['thickness_mm']) for p in pc['panels']}
PANEL_Y = {'PC3_Floor': -6.5, 'PC3_Side_Wall_L': -6.5, 'PC3_Side_Wall_R': -6.5, 'PC3_Rear_Wall': -108.0,
           'PC2_Bulkhead_L': 63.0, 'PC2_Bulkhead_R': 63.0, 'PC3_Lid': -23.0}


def frame_items(wall=TUBE, skin_scale=1.0, drive_tabs=4):
    out = []
    for name, (length, y) in MEMBERS.items():
        out.append(('frame ' + name, length * tube_g_per_mm(wall), y))
    for name, (n, area, y) in TABS.items():
        if name.startswith('drive_motor'):
            out += [('tab ' + name + ' F', 2 * area * TAB_T * RHO_STEEL, FRONT_Y)]
            out += [('tab ' + name + ' R', (drive_tabs - 2) * area * TAB_T * RHO_STEEL, REAR_Y)]
        else:
            out.append(('tab ' + name, n * area * TAB_T * RHO_STEEL, y))
    steel = sum(g for _, g, _ in out)
    out.append(('weld beads + paint', WELD * steel, -6.5))
    out.append(('rivnuts + screws', RIVNUTS[0] * RIVNUTS[1], -6.5))
    for name, (g, t) in PANELS.items():
        scale = 1.0 if name.startswith('PC2') else skin_scale     # bulkheads are already 2 mm
        out.append(('skin ' + name, g * scale, PANEL_Y[name]))
    return out


# ------------------------------------------------------------------ 4. weapon and drive models from R3
rot = r3calc['1_rotor']
E0 = rot['12.6V_full']['energy_J_no_load']
DISC_SHARE = rot['disc_share_of_rotor_I']          # one 6 mm disc
DISC_G = rot['disc_mass_g_cad']
F_MOTOR = r3calc['9_drive']['12.6V|mu0.6']['motor_limit_per_wheel_N']


def weapon(disc_t, n_disc=2):
    k = 1 - n_disc * DISC_SHARE * (1 - disc_t / 6.0) - (2 - n_disc) * DISC_SHARE
    return {'disc_thickness_mm': disc_t, 'discs': n_disc, 'inertia_vs_R3': k, 'energy_J_12.6V_no_load': E0 * k,
            'spinup_time_vs_R3': k, 'bite_vs_R3': 1.0}


def evaluate(its, two_wd):
    m = sum(g for _, g, _ in its)
    cg = sum(g * y for _, g, y in its) / m
    w = m / 1000 * G
    if two_wd:
        # two drive wheels on the front axle; the robot rocks onto whichever skid is on the CG side
        other = REAR_SKID_Y if cg < FRONT_Y else NOSE_SKID_Y
        front = w * (cg - other) / (FRONT_Y - other)
        rear = w - front
        push = 2 * min(F_MOTOR, MU * front / 2)
        push_net = push - 0.2 * rear           # skid drag, UHMW on tile mu 0.2 ASSUMED
        rests_on = 'wheels + rear skid' if cg < FRONT_Y else 'wheels + nose skid (y %.0f)' % NOSE_SKID_Y
    else:
        front = w * (cg - REAR_Y) / (FRONT_Y - REAR_Y)
        rear = w - front
        push = 2 * min(F_MOTOR, MU * front / 2) + 2 * min(F_MOTOR, MU * rear / 2)
        push_net = push
        rests_on = '4 wheels'
    return {'total_g': round(m, 1), 'margin_to_2000_g': round(LIMIT - m, 1), 'margin_to_target_g': round(TARGET - m, 1),
            'cg_y_mm': round(cg, 2), 'cg_behind_front_axle_mm': round(FRONT_Y - cg, 2),
            'front_axle_load_pct': round(100 * front / w, 1), 'other_support_load_pct': round(100 * rear / w, 1), 'rests_on': rests_on,
            'push_N_mu0.6': round(push_net, 2), 'fits_2000': m <= LIMIT, 'fits_target': m <= TARGET,
            'stable': (FRONT_Y - cg >= 5.0) if not two_wd else front / w >= 0.85}


# ------------------------------------------------------------------ 5. options
OPTIONS = {
    'D5': ('discs 6 -> 5 mm', {'disc_t': 5}),
    'D4': ('discs 6 -> 4 mm', {'disc_t': 4}),
    'D1': ('one 6 mm disc instead of two (layout change: centre disc)', {'n_disc': 1}),
    '2WD': ('2WD front drive, rear motors + wheels off, rear UHMW skid', {'two_wd': True}),
    'B650': ('battery 3S 850 -> 650 mAh (58 g ASSUMED)', {'battery_g': 58.0}),
    'S2': ('PC skins 3 -> 2 mm', {'skin_scale': 2 / 3}),
    'NoSkirt': ('remove the 3 aluminium skirts', {'no_skirt': True}),
    'NoCage': ('frame without posts/top crosses (lid screws to the PC walls with bought angle brackets)', {'no_cage': True}),
    'T08': ('tube wall 1.0 -> 0.8 mm (TIG only)', {'wall': 0.8}),
}
src('battery_650_g', 58.0, 'ASSUMED', '3S 650 mAh 75-80C pack; 40 A firmware cap needs >= 62C on 650 mAh')


def build(opts, wheel_g=WHEEL_G):
    o = {}
    for key in opts:
        o.update(OPTIONS[key][1])
    two_wd = o.get('two_wd', False)
    disc_t, n_disc = o.get('disc_t', 6), o.get('n_disc', 2)
    its = []
    for name, g, y in items:
        if name.startswith('Beater_Disc'):
            if name.endswith('_R') and n_disc == 1:
                continue
            g = g * disc_t / 6.0
        if two_wd and name.startswith('JGA25') and '_Rear_' in name:
            continue
        if name == '3S_850mAh_LiPo_ENVELOPE_75x35x25' and 'battery_g' in o:
            g = o['battery_g']
        if o.get('no_skirt') and name.startswith('Skirt_Plate'):
            continue
        its.append((name, g, y))
    for name, g, y in repl:
        if name.startswith('wheel_bought'):
            if two_wd and '_Rear_' in name:
                continue
            g = wheel_g
        if name.startswith('rear_skid') and two_wd:
            g = 8.0
        its.append((name, g, y))
    fr = frame_items(wall=o.get('wall', TUBE), skin_scale=o.get('skin_scale', 1.0), drive_tabs=2 if two_wd else 4)
    if o.get('no_cage'):
        fr = [f for f in fr if not f[0].startswith(('frame posts', 'frame top_'))] + [('bought Al angle brackets 8x1.5 g (lid)', 12.0, -23.0)]
    its += fr
    res = evaluate(its, two_wd)
    res.update({'options': list(opts), 'weapon': weapon(disc_t, n_disc)})
    return res, its


out = {'sources': SRC, 'limit_g': LIMIT, 'target_g': TARGET}
fixed_g = sum(g for _, g, _ in items)
out['fixed_r3_g'] = round(fixed_g, 1)
out['printed_removed_g'] = round(sum(p['mass_g'] for p in printed), 1)
out['printed_removed'] = sorted(p['part'] for p in printed)
out['replacements_g'] = round(sum(g for _, g, _ in repl), 1)
base, base_items = build([])
fr = frame_items()
out['frame_breakdown_g'] = {n: round(g, 1) for n, g, _ in fr}
out['frame_steel_g'] = round(sum(g for n, g, _ in fr if n.startswith(('frame', 'tab', 'weld'))), 1)
out['skins_g'] = round(sum(g for n, g, _ in fr if n.startswith('skin')), 1)
out['allowed_body_cross_check'] = {'r3_body_material_calc_allowed_body_g': 464.7,
                                   'here_body_budget_g': round(LIMIT - fixed_g - out['replacements_g'], 1)}
out['baseline_no_cuts'] = base
out['single_options'] = {k: build([k])[0] for k in OPTIONS}
combos = {}
keys = list(OPTIONS)
for n in (1, 2, 3, 4, 5):
    for c in itertools.combinations(keys, n):
        if ('D5' in c) + ('D4' in c) + ('D1' in c) > 1:
            continue
        r = build(list(c))[0]
        if r['fits_target'] and r['stable']:
            combos['+'.join(c)] = r
out['combos_fitting_target'] = dict(sorted(combos.items(), key=lambda kv: (-kv[1]['weapon']['energy_J_12.6V_no_load'], -kv[1]['push_N_mu0.6'], len(kv[0]))))
REC = {'A_keep_weapon_2WD': ['2WD', 'B650', 'S2'], 'B_keep_4WD_best': ['D1', 'B650', 'S2', 'NoCage', 'T08', 'NoSkirt'],
       'C_2WD_more_margin': ['2WD', 'B650', 'S2', 'NoCage']}
out['recommended_sets'] = {}
for name, c in REC.items():
    r = build(c)[0]
    r['wheel_sensitivity'] = {'%+d g/wheel' % d: build(c, WHEEL_G + d)[0]['total_g'] for d in (-15, 15)}
    out['recommended_sets'][name] = r
out['r3_reference'] = {'total_g': report['mass']['total_g'], 'energy_J': E0,
                       'push_N_mu0.6': r3calc['9_drive']['12.6V|mu0.6']['push_N_R3_loads']}
json.dump(out, open(os.path.join(HERE, 'steel_r4_calc.json'), 'w'), ensure_ascii=False, indent=1)

print('fixed R3 %.1f g | printed removed %.1f g | replacements %.1f g | frame steel %.1f g | skins %.1f g' % (
    fixed_g, out['printed_removed_g'], out['replacements_g'], out['frame_steel_g'], out['skins_g']))
print('body budget here %.1f g (R3 body_material_calc: 464.7)' % out['allowed_body_cross_check']['here_body_budget_g'])


def line(tag, r):
    w = r['weapon']
    print('%-28s total %7.1f  m2000 %+7.1f  cgY %6.2f  frontLoad %5.1f%%  push %5.2f N  E %5.1f J  %s%s' % (
        tag, r['total_g'], r['margin_to_2000_g'], r['cg_y_mm'], r['front_axle_load_pct'], r['push_N_mu0.6'],
        w['energy_J_12.6V_no_load'], 'FITS' if r['fits_target'] else ('<2kg' if r['fits_2000'] else 'OVER'),
        '' if r['stable'] else ' UNSTABLE'))


line('R3 reference', {'total_g': report['mass']['total_g'], 'margin_to_2000_g': 2000 - report['mass']['total_g'], 'cg_y_mm': report['mass']['centroid_mm'][1],
                      'front_axle_load_pct': 100 - 100 * report['mass']['rear_static_load_fraction'], 'push_N_mu0.6': out['r3_reference']['push_N_mu0.6'],
                      'weapon': {'energy_J_12.6V_no_load': E0}, 'fits_target': True, 'fits_2000': True, 'stable': True})
line('steel, no cuts', base)
for k, r in out['single_options'].items():
    line('  +' + k, r)
for k, r in out['recommended_sets'].items():
    line('SET ' + k, r)
    print('     wheel sensitivity', r['wheel_sensitivity'])
four = {k: r for k, r in combos.items() if '2WD' not in k}
print('%d combinations fit the %.0f g target (%d of them 4WD); top 6 by weapon energy:' % (len(combos), TARGET, len(four)))
for k, r in list(out['combos_fitting_target'].items())[:6]:
    line('  ' + k, r)
print('lightest 4WD combination:')
best4 = min((build(list(c))[0] | {'k': '+'.join(c)} for n in (3, 4, 5, 6) for c in itertools.combinations([k for k in keys if k != '2WD'], n)
             if ('D5' in c) + ('D4' in c) + ('D1' in c) <= 1), key=lambda r: r['total_g'])
line('  ' + best4['k'], best4)
out['lightest_4WD'] = best4
