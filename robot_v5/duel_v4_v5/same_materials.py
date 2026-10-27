"""V4 vs V5 rerun with the user's material spec for BOTH robots (2026-10-27):
frame = stainless steel (AISI 304), armour/skins = 3D-printed TPU.

Step 1 re-weighs both robots part by part from their CAD reports and sizes what has to change to stay
<= 2.0 kg. Step 2 maps the material change onto the duel inputs of duel_v4_v5.py and reruns it with the
same sweep. Material data: 304 annealed rho 7.93 g/cm3, yield ~215 MPa (SS400: 7.85, 245); TPU 95A
printed rho 1.21 x 0.85 infill; PC 1.20. Behaviour factors without test data are ASSUMED and swept.
Run: python3 robot_v5/duel_v4_v5/same_materials.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)
import duel_v4_v5 as d  # noqa: E402

SS304, SS400 = 7.93e-3, 7.85e-3
TPU = 1.21e-3 * 0.85
PC = 1.20e-3
Y304, Y400 = 215.0, 245.0
LIMIT = 2000.0

# ------------------------------------------------ V4 (R4-DIY D1) re-weighed
r4 = json.load(open(os.path.join(ROOT, 'robot_v4', 'print_revision', 'r4_diy', 'output', 'build_report.json')))
g4 = {}
for row in r4['rows']:
    m = row.get('material')
    g4[m] = g4.get(m, 0.0) + row['mass_g']
v4_steel = g4['steel_tube'] + g4['steel_tab'] + g4['steel_wedgelet']
v4_pc = g4['PC']
pc_area = v4_pc / (PC * 2.0)                               # mm^2 of 2 mm PC skin
v4 = {}
for t in (2.0, 2.5, 3.0):
    total = r4['mass']['total_g'] + v4_steel * (SS304 / SS400 - 1) - v4_pc + pc_area * t * TPU
    v4[f'TPU_{t}mm'] = round(total, 1)
V4_T = max(t for t in (2.0, 2.5, 3.0) if v4[f'TPU_{t}mm'] <= LIMIT - 20)   # keep >= 20 g margin

# ------------------------------------------------ V5 with a stainless tube frame + TPU skins
c5 = json.load(open(os.path.join(ROOT, 'robot_v5', 'cad', 'output', 'build_report.json')))
rows5 = c5['rows_park']
tub_lid = sum(r['mass_g'] for r in rows5 if 'TPU' in r['part'] and ('Tub' in r['part'] or 'Lid' in r['part']))
steel5 = sum(r['mass_g'] for r in rows5 if r.get('material') == 'steel')
base5 = c5['mass']['total_g'] - tub_lid + steel5 * (SS304 / SS400 - 1)
# perimeter frame inside the old tub footprint (x +-77, y -95..45): 2 rails + 3 crosses (rear, mid, front)
FRAME_MM = 2 * 140 + 3 * 149
# skins: floor, 2 sides, front, rear, lid minus the arm-module opening (46 x 60)
SKIN_MM2 = 154 * 140 + 2 * 140 * 48 + 2 * 154 * 48 + (154 * 140 - 46 * 60)
TUBES = {'12x12x1.2': (12, 1.2), '10x10x1.0': (10, 1.0)}
v5 = {}
for tn, (s, w) in TUBES.items():
    frame_g = FRAME_MM * (s ** 2 - (s - 2 * w) ** 2) * SS304 + 4 * 8.0          # + 4 motor tabs
    for t in (1.5, 2.0, 3.0):
        skins = SKIN_MM2 * t * TPU
        total = base5 + frame_g * 1.08 + skins                                  # 8 % weld beads (r4_diy)
        v5[f'{tn}_TPU_{t}mm'] = {'frame_g': round(frame_g, 1), 'skins_g': round(skins, 1), 'total_g': round(total, 1)}
fits = {k: v for k, v in v5.items() if v['total_g'] <= LIMIT}
V5_PICK = max(fits, key=lambda k: (k.startswith('12x12'), float(k.split('_')[-1][:-2]))) if fits else min(v5, key=lambda k: v5[k]['total_g'])
M5 = v5[V5_PICK]['total_g'] / 1000

# ------------------------------------------------ strength change of the V5 arm in 304
arm_sf_304 = c5['load_path']['safety_factors']['arm_plates_yield'] * Y304 / Y400
survive_304 = c5['load_path']['survivable_hit_energy_J (all SF >= 1)'] * (Y304 / Y400) ** 2

# ------------------------------------------------ duel inputs that the materials change (ASSUMED factors)
P2 = dict(d.P)
P2['E_petg_block_J'] = (1e9, 1e9)          # fork hinges now welded to the stainless frame: no PETG blocks
P2['p_tub_internal_v5'] = (0.02, 0.06)     # V4's hits land on the steel frame front cross, not a 3 mm TPU wall
# V5's down-strike now meets TPU skins on V4 instead of 2 mm PC: TPU deforms instead of cracking and a
# 1.6 mm tooth bite cuts it poorly -> ASSUMED 0.6-0.9 of the PC chance (swept as its own input)
TPU_VS_PC = (0.6, 0.9)
P2['p_strike_disables_v4'] = (d.P['p_strike_disables_v4'][0] * TPU_VS_PC[0], d.P['p_strike_disables_v4'][1] * TPU_VS_PC[1])


def run():
    saved = (d.M4, d.M5_RANGE)
    d.M4 = v4[f'TPU_{V4_T}mm'] / 1000
    d.M5_RANGE = (M5, M5)
    out = {}
    try:
        on, ex, res = d.grid(P2)
        out['base'] = {'V4_win': d.band(res, 'A_win'), 'V5_win': d.band(res, 'B_win'), 'draw': d.band(res, 'draw'),
                       'V4_by_KO': d.band(res, 'A_by_immobilise'), 'V5_by_KO': d.band(res, 'B_by_immobilise'),
                       'headon': on['head_on'], 'extras': ex}
        sweep = {}
        for k, (lo, hi) in P2.items():
            if lo == hi:
                continue
            for tag, val in (('low', (lo, lo)), ('high', (hi, hi))):
                _, _, rr = d.grid(dict(P2, **{k: val}), n_matches=1500)
                sweep[f'{k}={tag}'] = {'V4_win_median': d.band(rr, 'A_win')[1], 'V5_win_median': d.band(rr, 'B_win')[1]}
        out['sweep'] = sweep
        out['flips'] = [k for k, v in sweep.items() if v['V5_win_median'] > v['V4_win_median']]
        best = dict(P2, p_inv_v5=(0.4, 0.4), p_side_stuck_v5=(0.05, 0.05), p_selfright_v5=(0.95, 0.95),
                    p_tub_internal_v5=(0.02, 0.02), p_v5_ready=(0.9, 0.9), p_strike_lands_front=(0.55, 0.55),
                    p_strike_lands_side=(0.8, 0.8), p_strike_disables_v4=(P2['p_strike_disables_v4'][1],) * 2)
        _, _, rb = d.grid(best)
        out['V5_best_case'] = {'V4_win': d.band(rb, 'A_win'), 'V5_win': d.band(rb, 'B_win')}
    finally:
        d.M4, d.M5_RANGE = saved
    return out


def main():
    res = run()
    out = {'spec': 'frame AISI 304 stainless, armour 3D-printed TPU, both robots',
           'V4_mass_g_by_TPU_skin_thickness': v4, 'V4_skin_mm_used': V4_T,
           'V5_mass_options': v5, 'V5_option_used': V5_PICK, 'V5_mass_g_used': round(M5 * 1000, 1),
           'V5_over_2kg_with_12x12_frame': {k: v['total_g'] for k, v in v5.items() if k.startswith('12x12')},
           'V5_arm_plate_SF_304': round(arm_sf_304, 2), 'V5_arm_survives_J_304': round(survive_304, 1),
           'duel_inputs_changed': {'E_petg_block_J': P2['E_petg_block_J'], 'p_tub_internal_v5': P2['p_tub_internal_v5'],
                                   'p_strike_disables_v4': P2['p_strike_disables_v4'], 'TPU_vs_PC_factor': TPU_VS_PC},
           'result': res}
    json.dump(out, open(os.path.join(HERE, 'results_same_materials.json'), 'w'), indent=1, ensure_ascii=False)
    print('V4 mass by TPU skin', v4, '-> use', V4_T, 'mm')
    for k, v in v5.items():
        print('V5', k, v)
    print('V5 use', V5_PICK, M5, '| arm SF 304', round(arm_sf_304, 2), 'survives', round(survive_304, 1), 'J')
    b = res['base']
    print('BASE V4 win', b['V4_win'], 'V5 win', b['V5_win'], 'draw', b['draw'], '| KO V4', b['V4_by_KO'], 'V5', b['V5_by_KO'])
    print('headon', {k: round(v, 3) for k, v in b['headon'].items()})
    print('extras', {k: round(v, 3) for k, v in b['extras'].items()})
    print('V5 best case', res['V5_best_case'])
    print('flips', res['flips'])


if __name__ == '__main__':
    main()
