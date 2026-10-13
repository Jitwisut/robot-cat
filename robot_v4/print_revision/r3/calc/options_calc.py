"""Hand calculations for the low-wedge options simulated in mujoco_sim/wedge_options_r3.py.

Wedgelet plates: mass, balance moment, reach past the teeth, and the tip force that yields the plate when an
opponent lifter pries it up against its up-stop (cantilever from the carrier). Big/slow rotor: disc thickness at
equal mass, inertia, energy, gyro, bite, tooth strength and the unbalance of a 1-tooth disc.
Run: .venv/bin/python robot_v4/print_revision/r3/calc/options_calc.py   (writes options_calc.json)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
V4 = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, V4)
sys.path.insert(0, os.path.join(V4, 'mujoco_sim'))
import design_v4  # noqa: E402

calc = json.load(open(os.path.join(HERE, 'r3_calc.json')))
G = 9.81
STEEL = 7850.0
WIDTH = 61.0                                  # wedgelet plate span x 27..88 mm (CAD)
TOOTH_REACH_Y = 124.5
I_ROT = calc['sources']['rotor_I_kgm2']['value']
W_NL = calc['1_rotor']['12.6V_full']['rpm_no_load'] * 2 * math.pi / 60
M_ROBOT = calc['sources']['robot_mass_kg']['value']
X_CG = abs(calc['sources']['cg_mm']['value'][0])
ALLOW_ADD_FRONT_G = calc['10_mass_balance']['target_10%']['max_add_at_y110_g']
SIGMA = {'mild_S275': 275, 'spring_or_AR_1000': 1000}
# R3 lifter: servo 4 N*m (sim.py forcerange) on a 35.9 mm arm -> tip force
R3_LIFTER_TIP_N = 4.0 / math.hypot(0.0345, 0.010)
src = {'plate_width_mm': [WIDTH, 'CAD'], 'R3_lifter_tip_N': [R3_LIFTER_TIP_N, 'SIM model servo 4 N*m'],
       'yield_MPa': [SIGMA, 'ASSUMED handbook'], 'pry_lever': ['full plate length from the carrier', 'ASSUMED conservative']}

PLATES = {'now': (35.54, 2.0, (100.0, 20.5), (128.0, 0.5)),
          'w25_short': (25.0, 2.0, (100.0, 13.56), (128.0, 0.5)),
          'w25_long_t1.5': (25.0, 1.5, (100.0, 20.5), (100.0 + 20.0 / math.tan(math.radians(25)), 0.5)),
          'w20_long_t1.2': (20.0, 1.2, (100.0, 20.5), (100.0 + 20.0 / math.tan(math.radians(20)), 0.5))}
now_len = math.hypot(28.0, 20.0)
now_g = 26.5836
plates = {}
for name, (ang, t, a, b) in PLATES.items():
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    g = now_g * length * t / (now_len * 2.0)
    cy = (a[0] + b[0]) / 2
    Z = WIDTH * t ** 2 / 6
    plates[name] = {'angle_deg': ang, 't_mm': t, 'length_mm': length, 'tip_y_mm': b[0],
                    'tip_ahead_of_teeth_mm': b[0] - TOOTH_REACH_Y, 'plate_g_each': g,
                    'delta_mass_g_pair': 2 * (g - now_g),
                    'delta_moment_gmm_pair': 2 * (g * cy - now_g * (100 + 128) / 2),
                    'within_front_allowance': 2 * (g - now_g) <= ALLOW_ADD_FRONT_G,
                    'pry_yield_tip_N': {k: s * Z / length for k, s in SIGMA.items()},
                    'R3_lifter_tip_N': R3_LIFTER_TIP_N}
    plates[name]['survives_R3_lifter'] = {k: v > R3_LIFTER_TIP_N for k, v in plates[name]['pry_yield_tip_N'].items()}

# big/slow rotor: r40 single-tooth disc at equal mass
m_now, i_now = design_v4.disc_props(6.0)
design_v4.TOOTH_R_MM, design_v4.N_TEETH = 40.0, 1
m6, _ = design_v4.disc_props(6.0)
t_eq = 6.0 * m_now / m6
m_big, i_big = design_v4.disc_props(t_eq)
I_big = I_ROT - 2 * i_now + 2 * i_big


def unbalance(t_mm):
    """First moment of mass (kg*m) of one disc about its axis, by the same polar integration as design_v4."""
    r = np.linspace(design_v4.BORE_R_MM, design_v4.TOOTH_R_MM, 600) / 1000
    th = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    R, TH = np.meshgrid(r, th)
    d = np.angle(np.exp(1j * TH))
    inside = (R <= design_v4.BASE_R_MM / 1000) | (np.abs(d) < math.radians(design_v4.TOOTH_SPAN_DEG / 2))
    X, Y = R * np.cos(TH), R * np.sin(TH)
    for rc, ac, rh in design_v4.DISC_HOLES:
        cx, cy = rc / 1000 * math.cos(math.radians(ac)), rc / 1000 * math.sin(math.radians(ac))
        inside &= (X - cx) ** 2 + (Y - cy) ** 2 > (rh / 1000) ** 2
    w = inside * (r[1] - r[0]) * (th[1] - th[0]) * R * STEEL * t_mm / 1000
    return float(math.hypot((w * X).sum(), (w * Y).sum()))


me = unbalance(t_eq)
big = {}
for ratio in (1.0, 1.5):
    w = W_NL / ratio
    L = I_big * w
    big['ratio_%.1f' % ratio] = {
        'rotor_rpm': w * 60 / (2 * math.pi), 'energy_J': 0.5 * I_big * w ** 2, 'tip_speed_m_s': w * 0.040,
        'gyro_lift_yaw_rad_s': M_ROBOT * G * (0.070 - X_CG / 1000) / L,
        'bite_mm_at_2m_s': 2.0 * 2 * math.pi / (1 * 0.9 * w) * 1000,
        'unbalance_force_per_disc_N': me * w ** 2,
        'bearing_load_from_couple_N_if_discs_180deg': me * w ** 2 * 46.0 / 33.0}
big['disc_t_equal_mass_mm'] = t_eq
big['disc_mass_g'] = m_big * 1000
big['rotor_I_kgm2'] = I_big
big['I_ratio_vs_now'] = I_big / I_ROT
big['tooth_height_mm'] = 40.0 - design_v4.BASE_R_MM
big['tooth_root_bending_cap_N'] = {k: s * t_eq * 23.0 ** 2 / (6 * (40.0 - 23.0)) for k, s in SIGMA.items()}
big['tooth_root_bending_cap_N_now'] = {k: s * 6.0 * 23.0 ** 2 / (6 * 6.5) for k, s in SIGMA.items()}
big['disc_unbalance_kg_m'] = me
big['note'] = ('axle z 42.5 mm (tip 2.5 mm off the floor): taller uprights, new belt/motor position, rotor top '
               'z 82.5 mm above the 63 mm lid; not mass-checked in CAD')

out = {'sources': src, 'wedgelet_plates': plates, 'big_slow_rotor': big,
       'front_mass_allowance_g_at_y110': ALLOW_ADD_FRONT_G}
json.dump(out, open(os.path.join(HERE, 'options_calc.json'), 'w'), indent=1, default=float)
for k, v in plates.items():
    print(k, {a: (round(b, 2) if isinstance(b, float) else b) for a, b in v.items()})
print(json.dumps(big, indent=1, default=float))
