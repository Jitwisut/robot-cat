"""V5 cost cuts so V5 can be built IN ADDITION to V4.

User decision 2026-10-27: build V5 as well as V4, find ways to cut the V5 budget.
Starts from the recommended V5 in design_v5.py (steel disc B + DS3218 + 4WD) and
prices every cut together with what it costs in mass, energy, spin-up or strength.
DIY routes follow robot_v4/print_revision/r4_diy (user has a stick welder and uses
ordinary steel shops). Prices not in the repo catalog are ASSUMED.

Run: python3 budget_cuts_v5.py   (writes budget_cuts_v5.json)
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design_v5 as d  # noqa: E402

SAW = d.saw['B_steel_disc_85mm_3mm_3T']
I_DISC = SAW['I_kgm2']
SERVO = 'DS3218 19kg-cm @5V'
BASE_COST = d.cost(SERVO, '4WD')                      # includes ESP32 + charger to buy
BASE_MASS = d.mass_budget('B_steel_disc_85mm_3mm_3T', SERVO, '4WD')

# ------------------------------------------------ saw motor options
# kv, winding ohm, ESC current limit A, no-load A, mass g, price THB, belt ratio (disc/motor)
MOTORS = {
    'D3536 1250kV + Skywalker 40A (baseline)': (1250, 0.075, 40, 1.5, 102, 560 + 490, 1.0),
    # Prices 2026-10-27 (PRICES below): motor ABC 2212 320 THB (KV to confirm with the shop; Lazada A2212 1400KV ~200),
    # SkyWalker 30A-V2-UBEC 400 THB with a 5 V / 5 A switching BEC. Winding R / no-load A are ASSUMED.
    'A2212 1400kV + SkyWalker 30A V2, belt 0.9': (1400, 0.10, 30, 0.8, 54, 320 + 400, 0.9),
    'A2212 1000kV + SkyWalker 30A V2, belt 1.0': (1000, 0.13, 30, 0.6, 54, 320 + 400, 1.0),
}
ESC_G = {'D3536 1250kV + Skywalker 40A (baseline)': 35}
SKYWALKER30_G = 39                                    # seller spec, 68 x 25 x 8 mm
PRICES = {   # found 2026-10-27, Thai shops unless noted; not yet confirmed by phone/cart
    'A2212 class motor': {'thb': 320, 'source': 'https://www.abcflying.com/product/4408/', 'note': 'Motor ABC 2212, KV list not shown; Lazada A2212 1400KV listing ~200 THB'},
    'ESC 30A with BEC': {'thb': 400, 'source': 'https://www.abcflying.com/product/10413/hobbywing-skywalker-30a-v2-ubec-3s-4s', 'note': 'BEC 5 V / 5 A switching, 39 g; listing text also says 40A/60A'},
    'DS3218 servo': {'thb': 735, 'source': 'https://www.mikroelec.com/product/3424/ds3218mg-pro-waterproof-digital-servo-20kg-360-degree-position-control', 'note': 'Pro 360-deg position version; Lazada RC surpass hobby 801 THB'},
    'UBEC (separate)': {'thb': 550, 'source': 'abcflying.com Hobbywing UBEC-5A-V2-Air', 'note': 'not needed with the SkyWalker BEC; a 25 THB 5 V/3 A buck (allnewstep A2052) is 2 A continuous, too weak for a stalled servo'},
    'flat bar 1/2in x 4 mm, 6 m': {'thb': 198, 'source': 'search result, Thai steel price page (undated)', 'note': '~2.2 kg; forks need ~0.3 m'},
    'black sheet 2.3 mm 4x8 ft': {'thb': 2087, 'source': 'http://bks-steel.co.th (1,950 + 7 % VAT, 53.2 kg, early-2025 page)', 'note': '39 THB/kg; buy an offcut, the arm needs ~0.12 kg'},
}


def motor_case(kv, r, i_lim, i0, belt):
    kt = 60 / (2 * math.pi * kv)
    w_motor = 0.9 * d.V_BATT / kt
    w = w_motor * belt
    j = I_DISC                                          # disc + hub at the disc; the motor rotor is ignored
    wm, t, dt = 0.0, 0.0, 1e-4
    while wm * belt < 0.9 * w and t < 10:
        i = min(i_lim, (d.V_BATT - kt * wm) / r)
        wm += kt * max(i - i0, 0) / (j * belt ** 2) * dt
        t += dt
    return {'disc_rpm': round(w * 60 / (2 * math.pi)), 'energy_J': round(0.5 * j * w ** 2, 1),
            'energy_vs_V4': round(0.5 * j * w ** 2 / d.V4_ENERGY_J, 2), 't_spin_90pct_s': round(t, 2),
            'peak_input_W': round(d.V_BATT * i_lim)}


motors = {}
for k, (kv, r, il, i0, g, thb, belt) in MOTORS.items():
    motors[k] = dict(motor_case(kv, r, il, i0, belt), mass_g=g + ESC_G.get(k, SKYWALKER30_G), price_thb=thb)

# ------------------------------------------------ DIY arm / towers / forks / disc
ST = 7.85e-3                                          # g/mm^3
arm_al_g = d.arm_plates_g
ARM_ST_T = 2.3                                        # CAD load path: 1.6 left SF 1.01 at the stop, 2.3 gives 1.45
arm_st_g = 2 * ARM_ST_T * 27 * (d.ARM_L_MM + 20) * ST   # black sheet offcut (R4-DIY already buys 2.3), CAD outline
towers_st_g = 2 * 40 * 60 * 1.6 * ST                  # 1.6 mm sheet, welded to the frame (was 6061 4 mm, 40 g)
# In-plane stiffness/strength of the 2-plate arm, plates on edge: EI ~ E*t, M_yield ~ Sy*t
arm_ratio = {'stiffness_steel2.3_vs_6061_3mm': round(200 * ARM_ST_T / (69 * 3.0), 2),
             'yield_moment_steel2.3_vs_6061_3mm': round(245 * ARM_ST_T / (276 * 3.0), 2)}
# Hit load path from the CAD (stop posts under the arbor): cad/output/v5_cad_props.json
LOAD = d.CAD['load_path'] if d.CAD and 'load_path' in d.CAD else None
arm_sigma_mpa = LOAD['arm_plate_stress_at_stop_MPa'] if LOAD else float('nan')

FORK_T_DIY = 4.0                                      # Thai Watsadu's thinnest flat bar (r4_diy sources)
fork_diy_g = d.FORK_N * d.FORK_W_MM * FORK_T_DIY * d.FORK_L_MM * ST
f_drop = sum(BASE_MASS.values()) / 1000 * d.G * 5       # own robot dropped on one tip x5 g
# CAD (cad/build_v5.py) stands the 12 x 4 bar ON EDGE: 12 tall for vertical loads, 4 wide sideways
z_vert = FORK_T_DIY * d.FORK_W_MM ** 2 / 6            # 96 mm^3
z_side = d.FORK_W_MM * FORK_T_DIY ** 2 / 6            # 32 mm^3
fork_diy = {'orientation': 'on edge (12 tall x 4 wide)',
            'stress_drop_on_tip_MPa': round(f_drop * d.FORK_L_MM / z_vert), 'SF_S275': round(275 / (f_drop * d.FORK_L_MM / z_vert), 2),
            'SF_opponent_on_tip': round(275 / (2 * d.G * 3 * d.FORK_L_MM / z_vert), 2),
            'SF_side_push_30N_at_tip': round(275 / (30 * d.FORK_L_MM / z_side), 2), 'mass_g': round(fork_diy_g, 1)}

# ------------------------------------------------ the cuts
best_motor = 'A2212 1400kV + SkyWalker 30A V2, belt 0.9'
CUTS = [
    # (name, saving THB, mass delta g, what it costs)
    ('saw motor + ESC: D3536/Skywalker40A -> ' + best_motor,
     MOTORS['D3536 1250kV + Skywalker 40A (baseline)'][5] - MOTORS[best_motor][5],
     motors[best_motor]['mass_g'] - motors['D3536 1250kV + Skywalker 40A (baseline)']['mass_g'],
     'energy %s J (was %s), spin-up %s s (was %s); A2212 is rated ~15-20 A, the 30 A spin-up overloads it briefly'
     % (motors[best_motor]['energy_J'], motors['D3536 1250kV + Skywalker 40A (baseline)']['energy_J'],
        motors[best_motor]['t_spin_90pct_s'], motors['D3536 1250kV + Skywalker 40A (baseline)']['t_spin_90pct_s'])),
    ('arm (2.3) + towers/tongue (1.6) + stop posts (3.2 flat): CNC/laser 6061 -> steel, hand-cut + welded; + UHMW nose skid',
     600 - 100 - 40, round(arm_st_g + towers_st_g - arm_al_g - 40, 1),
     'arm stiffness x%s vs 3 mm 6061; under a %s J hit the plates see %.0f MPa at the stop (SF %s), arm survives ~%s J; '
     'tongue/posts/skid mass is in the CAD total, not in this row'
     % (arm_ratio['stiffness_steel2.3_vs_6061_3mm'], LOAD and LOAD['energy_J'], arm_sigma_mpa,
        LOAD and LOAD['safety_factors']['arm_plates_yield'], LOAD and LOAD['survivable_hit_energy_J (all SF >= 1)'])),
    ('forks: laser-cut 65Mn 12x3 -> mild flat bar 12x4 on edge (hinged), one 6 m bar', 400 - PRICES['flat bar 1/2in x 4 mm, 6 m']['thb'], round(fork_diy_g - d.fork_g, 1),
     'SF dropped-on-tip %s (65Mn 3 mm flat was 2.17); sideways 30 N push at the tip SF %s; straighten between fights'
     % (fork_diy['SF_S275'], fork_diy['SF_side_push_30N_at_tip'])),
    ('saw disc: laser-cut -> 3 mm SS400 offcut ground to a 1:1 template (as R4-DIY disc)', 250 - 50, 0,
     'same outline = same energy; softer teeth wear, regrind'),
    ('UBEC 8 A dropped: the SkyWalker 30A V2 BEC (5 V / 5 A) runs servo + ESP32', 150, -15,
     'servo at 5 V: 1.86 N m and 0.16 s/60 deg instead of 2.1 N m / 0.14 s at 6.8 V; still above the 1.3 N m clutch'),
    ('fasteners/inserts/XT30: share bulk packs with V4 (ASSUMED)', 100, 0, 'none'),
]
SHARED = [
    ('charger: share V4\'s iMAX B6AC', 640, 'charge one pack at a time'),
    ('ESP32: user owns two (2026-10-27), one is for V5', 180, 'none'),
    ('PS4-clone pad: one pad, re-pair to whichever robot fights', 350, 'firmware must accept the pad MAC on both robots; no spare pad'),
]
REJECTED = [
    ('2WD instead of 4WD', 370, 'push 6.2 N vs 10.9 N at mu 0.6 (design_v5.py) -> loses to wedges/pushers'),
    ('MG996R servo instead of DS3218', 735 - 150, 'about 0.95 N m at 5 V (spec 9.4 kg cm at 4.8 V) -> below the 1.3 N m clutch '
     'setting, self-right margin %.2f < 1.5' % (0.95 / d.self_right(sum(BASE_MASS.values()) / 1000, 1.86)['torque_needed_Nm'])),
    ('share one LiPo between V4 and V5', 490, 'cannot fight back-to-back; one bad pack takes out both robots'),
    ('TCT shop blade instead of steel disc', 0, 'not cheaper than a DIY disc and stores %.1f J instead of %.0f J' % (d.saw['A_tct_blade_85mm_24T']['energy_J'], SAW['energy_J'])),
]


def main():
    base_total = sum(BASE_COST.values())
    cut_thb = sum(c[1] for c in CUTS)
    cut_g = sum(c[2] for c in CUTS)
    base_g = sum(BASE_MASS.values())
    # charger is a tool, not a robot part; the user owns a second ESP32 for V5 (2026-10-27), and owned parts
    # stay out of the BOM as in V4 (memory: owned-parts)
    robot = base_total - 640 - 180 - cut_thb
    out = {
        'baseline_cost_thb_incl_esp32_charger': base_total,
        'prices_found_2026_10_27': PRICES,
        'motor_options': motors,
        'diy_forks_12x4_mild': fork_diy,
        'arm_steel_vs_6061': arm_ratio,
        'cuts': [{'cut': c[0], 'saves_thb': c[1], 'mass_delta_g': c[2], 'cost_of_cut': c[3]} for c in CUTS],
        'shared_with_v4': [{'item': s[0], 'saves_thb': s[1], 'catch': s[2]} for s in SHARED],
        'rejected': [{'idea': r[0], 'would_save_thb': r[1], 'why_not': r[2]} for r in REJECTED],
        'result': {
            'v5_robot_parts_thb (pad included; owned ESP32 and charger excluded)': robot,
            'margin_to_6000_cap_thb': 6000 - robot,
            'extra_cash_for_v5_when_v4_exists_thb': robot - 350,
            'mass_g': round(base_g + cut_g), 'margin_to_2kg_g': round(2000 - base_g - cut_g),
            'saw_energy_J': motors[best_motor]['energy_J'], 't_spin_90pct_s': motors[best_motor]['t_spin_90pct_s'],
        },
    }
    json.dump(out, open(os.path.join(HERE, 'budget_cuts_v5.json'), 'w'), indent=2, ensure_ascii=False)
    print('MOTORS'); [print(' ', k, v) for k, v in motors.items()]
    print('FORK DIY', fork_diy, 'ARM', arm_ratio, 'arm sigma', round(arm_sigma_mpa, 1))
    print('CUTS'); [print('  -%4d THB %+6.1f g  %s | %s' % (c[1], c[2], c[0], c[3])) for c in CUTS]
    print('REJECTED'); [print(' ', r) for r in REJECTED]
    print('RESULT', out['result'], 'baseline', base_total, 'base mass', round(base_g))


if __name__ == '__main__':
    main()
