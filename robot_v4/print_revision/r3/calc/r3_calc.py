"""V4_PRINT_R3 engineering calculation sheet: weapon, strength, electrical, drive, mass/balance.

Reduced-order hand calculations, not FEA. Every input carries a source tag:
  CAD      r3/output_R3/build_report.json (mass, centroid, rotor inertia) or legacy_geometry.py dimensions
  SIM      r3/output_R3/simulation.json (MuJoCo, same build)
  CATALOG  r3/inputs_R3.json electrical specs / hardware catalog values (source_id in that file)
  NOMINAL  inputs_R3.json nominal values (not confirmed for the bought part)
  ASSUMED  engineering assumption made here; replace with a measurement
Strength results are CAPACITIES: the tooth-tip force at which each part reaches its limit, so they do not depend
on the (unknown) impact force. Demand is shown separately as a range.
Does not touch design_v4.py / v4_design.json (sim.py loads them) or output_R3/ (release-gated).
Run: python3 robot_v4/print_revision/r3/calc/r3_calc.py   (writes r3_calc.json next to this file)
Units SI unless the name ends in _mm, _g, _mAh or _rpm.
"""
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R3 = os.path.dirname(HERE)
PR = os.path.dirname(R3)
V4 = os.path.dirname(PR)
sys.path.insert(0, PR)
sys.path.insert(0, V4)
from inputs import geometry_payload  # noqa: E402
from design_v4 import disc_props  # noqa: E402  (pure function; same tooth profile as legacy_geometry)

G = 9.81
SRC = {}


def src(name, value, tag, note=''):
    SRC[name] = {'value': value, 'source': tag, **({'note': note} if note else {})}
    return value


# ------------------------------------------------------------------ inputs
report = json.load(open(os.path.join(R3, 'output_R3', 'build_report.json')))
config = json.load(open(os.path.join(R3, 'inputs_R3.json')))
simres = json.load(open(os.path.join(R3, 'output_R3', 'simulation.json')))
if hashlib.sha256(json.dumps(geometry_payload(config), sort_keys=True).encode()).hexdigest() != report['input_sha256']:
    raise RuntimeError('R3 inputs changed after the CAD build; rebuild before calculating')
if simres['build_sha256'] != report['build_sha256']:
    raise RuntimeError('simulation.json is from another build')
mass = report['mass']
parts = {p['part']: p['mass_g'] for p in mass['parts']}
spec = config['electrical']['specs']
nom = config['electrical']['nominal']

M = src('robot_mass_kg', mass['total_g'] / 1000, 'CAD')
CG = src('cg_mm', mass['centroid_mm'], 'CAD')
I_ROT = src('rotor_I_kgm2', mass['rotor_inertia_kg_m2'], 'CAD')
KV = src('weapon_kv', spec['weapon_kv'], 'CATALOG', 'SURPASS C3536-V2 manual, R3-08')
KE = 60 / (2 * math.pi * KV)
R_M = src('weapon_R_ohm', spec['weapon_resistance_ohm'], 'CATALOG', 'R3-08')
I0 = src('weapon_I0_A', spec['weapon_no_load_current_a'], 'CATALOG', 'R3-08')
ESC_A = src('esc_continuous_A', spec['esc_continuous_current_a'], 'CATALOG', 'Skywalker 40A, TH03; burst rating not found')
MOTOR_MAX_A = src('weapon_motor_max_A', 50, 'CATALOG', 'SURPASS table "50A ESC" (sources.json), not a measured limit')
CELLS = spec['battery_cells']
CAP_MAH = src('battery_mAh', spec['battery_capacity_mah'], 'CATALOG', 'GNB8503S80A, R3-02')
C_RATING = src('battery_C', 80, 'CATALOG', 'label value; real continuous capability is usually lower')
V_FULL, V_NOM = CELLS * 4.2, CELLS * 3.7
R_BATT = src('battery_IR_ohm_cases', [0.0, 0.015, 0.030], 'ASSUMED', 'pack internal resistance, measure with charger')
ETA_BELT = src('belt_efficiency', 0.92, 'ASSUMED', 'same as sim.py')
RAMP = src('firmware_ramp_us_per_s', 1600.0, 'CAD', 'v4_controller.ino WEAPON_RAMP_US_PER_S')
YAW_LIMIT = src('firmware_yaw_limit_rad_s', 690.0 * math.pi / 180, 'CAD', 'v4_controller.ino YAW_LIMIT_DPS')
DRIVE_TQ12 = src('drive_stall_Nm_12V', nom['drive_stall_torque_nm'], 'NOMINAL', 'JGA25-370 400 rpm, interpolated')
DRIVE_RPM12 = src('drive_rpm_12V', nom['drive_rated_rpm'], 'NOMINAL')
DRIVE_STALL_A = src('drive_stall_A_each', 1.3, 'NOMINAL', 'design_v4.py, PMD table')
DRV_PEAK_A = src('drv8871_peak_A', 3.6, 'CATALOG', 'datasheet peak; continuous is thermal-limited, not confirmed')
REST_LOADS = src('rest_wheel_loads_N', simres['rest_wheel_loads_N'], 'SIM', 'FL, FR, RL, RR')
TURN_PEAK = src('sim_turn_peak_yaw_rad_s', simres['turn']['peak_yaw_rad_s'], 'SIM', 'spin in place, full stick')

# geometry (legacy_geometry.py + revision.py), mm
AXLE_Z, ROTOR_Y, TIP_R, BASE_R, TOOTH_DEG = 32.0, 95.0, 29.5, 23.0, 60.0
DISC_X = (20.0, 26.0)            # disc faces, each side
BRG_X = 16.5                     # 608 centre (13..20)
UPR_X = 30.0                     # upright mid-plane (27..33)
SCREW_R = 13.0                   # 4x M3 disc-to-hub screws
FRONT_Y, REAR_Y = config['design']['front_axle_y_mm'], config['design']['rear_axle_y_mm']
HALF_TRACK = 70.0                # wheel centre x
WHEEL_R = 32.0
src('geometry_mm', {'axle_z': AXLE_Z, 'rotor_y': ROTOR_Y, 'tip_r': TIP_R, 'base_r': BASE_R, 'disc_x': DISC_X,
                    'bearing_x': BRG_X, 'upright_x': UPR_X, 'disc_screw_r': SCREW_R, 'axles_y': [FRONT_Y, REAR_Y],
                    'half_track': HALF_TRACK}, 'CAD', 'legacy_geometry.py / revision.py')

# materials (yield / limits, MPa) - none of the metal grades is confirmed by a certificate
MAT = src('materials_MPa', {
    'disc_steel_S275_yield': 275, 'disc_steel_hardened_AR400_yield': 1000,
    'al6061T6_yield': 276, 'al6061T6_bearing_yield': 386, 'al6061T6_shear_yield': 159,
    'bolt_8.8_yield': 640, 'bolt_10.9_yield': 940, 'bolt_12.9_yield': 1080, 'A2-70_yield': 450,
    'tpu95a_bearing_allow': (5, 10), 'bearing_608_C0_N': 1370}, 'ASSUMED',
    'typical handbook values; disc steel grade, bolt grades and 608 C0 not confirmed for the bought parts')

out = {'build_sha256': report['build_sha256'], 'sources': SRC}


# ------------------------------------------------------------------ 1. rotor and energy
def w_noload(v):
    return (v - I0 * R_M) / KE


m_disc, i_disc = disc_props(6.0)
rot = {'rotor_mass_g': sum(parts[n] for n in parts if n.startswith(('Beater_', 'Bearing_')) or n == 'rotor screws'),
       'disc_mass_g_calc': m_disc * 1000, 'disc_mass_g_cad': parts['Beater_Disc_L'],
       'disc_I_kgm2': i_disc, 'disc_share_of_rotor_I': i_disc / I_ROT,
       'tip_effective_mass_kg': I_ROT / (TIP_R / 1000) ** 2}
for label, v in (('12.6V_full', V_FULL), ('11.1V_nominal', V_NOM)):
    w = w_noload(v)
    rot[label] = {'rpm_no_load': w * 60 / (2 * math.pi), 'energy_J_no_load': 0.5 * I_ROT * w ** 2,
                  'energy_J_at_90pct': 0.5 * I_ROT * (0.9 * w) ** 2, 'tip_speed_m_s': w * TIP_R / 1000,
                  'angular_momentum_Nms': I_ROT * w}
out['1_rotor'] = rot


# ------------------------------------------------------------------ 2. spin-up and recovery
def spin(ramp=RAMP, r_batt=0.0, cap=None, v_oc=V_FULL, w_start=0.0, thr_start=0.0, t_end=3.0, dt=2e-5):
    """Rotor speed ODE. Motor current I_m = (thr*V_oc - KE*w)/(R_m + thr^2*R_b) (battery current = thr*I_m)."""
    w, thr, t = w_start, thr_start, 0.0
    w_nl = w_noload(v_oc)
    peak_m = peak_b = tq_peak = e_batt = 0.0
    t90 = None
    while t < t_end:
        thr = 1.0 if ramp is None else min(1.0, thr + ramp / 1000 * dt)
        im = max((thr * v_oc - KE * w) / (R_M + thr * thr * r_batt), 0.0)
        if cap is not None:
            im = min(im, cap)
        ib = thr * im
        tq = KE * max(im - I0, 0.0) * ETA_BELT
        w += tq / I_ROT * dt
        e_batt += (v_oc - r_batt * ib) * ib * dt
        peak_m, peak_b, tq_peak = max(peak_m, im), max(peak_b, ib), max(tq_peak, tq)
        t += dt
        if t90 is None and w >= 0.9 * w_nl:
            t90 = t
    return {'t90_s': t90, 'rpm_end': w * 60 / (2 * math.pi), 'peak_motor_A': peak_m, 'peak_battery_A': peak_b,
            'peak_rotor_torque_Nm': tq_peak, 'battery_energy_J': e_batt, 'battery_mAh': e_batt / v_oc / 3.6,
            'rotor_energy_J': 0.5 * I_ROT * w ** 2}


su = {'check_vs_sim': {'calc': spin(t_end=2.5), 'sim_rpm': simres['spinup']['rotor_rpm'],
                       'sim_peak_A': simres['spinup']['predicted_peak_current_a']}}
for rb in R_BATT:
    for name, kw in (('ramp1600', {}), ('ramp800', {'ramp': 800.0}), ('ramp3200', {'ramp': 3200.0}),
                     ('no_ramp', {'ramp': None}), ('no_ramp_cap40', {'ramp': None, 'cap': 40.0})):
        su['Rb%.0fmOhm|%s' % (rb * 1000, name)] = spin(r_batt=rb, **kw)
# after a hit the throttle is already full (firmware ramps only upward from the current command)
for frac in (0.5, 0.0):
    for rb in R_BATT:
        su['after_hit_to_%d%%|Rb%.0fmOhm' % (frac * 100, rb * 1000)] = spin(
            ramp=None, r_batt=rb, w_start=frac * w_noload(V_FULL), thr_start=1.0, t_end=2.0)
# rpm-aware throttle cap using the existing hall sensor: thr <= (KE*w + I_lim*R_m)/V keeps motor current <= I_lim
for frac in (0.5, 0.0):
    su['after_hit_to_%d%%|cap40_rpm_limiter|Rb15mOhm' % (frac * 100)] = spin(
        ramp=None, r_batt=0.015, cap=40.0, w_start=frac * w_noload(V_FULL), thr_start=1.0, t_end=2.0)
out['2_spinup'] = su

# ------------------------------------------------------------------ 3. gyroscopic wheel lift and spin-up pitch
x_cg = abs(CG[0])
gy = {}
for label, v in (('12.6V', V_FULL), ('11.1V', V_NOM)):
    L = I_ROT * w_noload(v)
    gy[label] = {'yaw_rate_lifting_weak_side_wheel_rad_s': M * G * (HALF_TRACK - x_cg) / 1000 / L,
                 'yaw_rate_lifting_strong_side_wheel_rad_s': M * G * (HALF_TRACK + x_cg) / 1000 / L}
gy['firmware_limit_rad_s'] = YAW_LIMIT
gy['sim_spin_in_place_peak_rad_s'] = TURN_PEAK
out['3_gyro'] = gy

restoring = M * G * (FRONT_Y - CG[1]) / 1000            # nose-down tipping pivots on the front axle
rear_static_N = REST_LOADS[2] + REST_LOADS[3]
pitch = {'restoring_moment_about_front_axle_Nm': restoring, 'rear_static_load_N': rear_static_N,
         'wheelbase_m': (FRONT_Y - REAR_Y) / 1000}
for k in ('Rb0mOhm|ramp1600', 'Rb30mOhm|ramp1600', 'Rb0mOhm|no_ramp', 'Rb0mOhm|no_ramp_cap40'):
    tq = su[k]['peak_rotor_torque_Nm'] / ETA_BELT                  # chassis reacts the motor torque
    pitch[k] = {'reaction_Nm': tq, 'rear_wheels_unload_N': tq / pitch['wheelbase_m'],
                'nose_tips_onto_skid': tq > restoring}
lo, hi = 200.0, RAMP
for _ in range(30):                              # largest ramp whose reaction stays below the restoring moment
    mid = (lo + hi) / 2
    if spin(ramp=mid, r_batt=0.015, t_end=1.5)['peak_rotor_torque_Nm'] / ETA_BELT < restoring:
        lo = mid
    else:
        hi = mid
pitch['max_ramp_keeping_rear_wheels_down_us_per_s'] = lo
pitch['t90_at_that_ramp_s'] = spin(ramp=lo, r_batt=0.015)['t90_s']
out['3_spinup_pitch'] = pitch

# ------------------------------------------------------------------ 4. bite
bite = {}
for v in (V_FULL, V_NOM):
    w = 0.9 * w_noload(v)
    bite['%.1fV_90pct' % v] = {'%.1f_m_s' % vc: vc * 2 * math.pi / (2 * w) * 1000 for vc in (0.5, 1.0, 2.0, 3.0)}
bite['tooth_height_mm'] = TIP_R - BASE_R
bite['tip_floor_clearance_mm'] = AXLE_Z - TIP_R
bite['note'] = '2 teeth per disc, both discs in phase (same profile in CAD)'
out['4_bite_mm'] = bite


# ------------------------------------------------------------------ 5. hitting a sloped face (why wedges deflect)
def vertical_impulse_sign(alpha_deg, mu):
    """Unit normal impulse on a face sloping alpha up and away from V4, with friction dragging it up-slope.
    z component per unit normal impulse = -cos(a) + mu*sin(a): > 0 means the hit can lift the opponent."""
    a = math.radians(alpha_deg)
    return -math.cos(a) + mu * math.sin(a)


faces = {'R3_plow_wings': 25.3, 'R3_centre_lifter': math.degrees(math.atan2(10.0, 34.5)),
         'V2_spatula': math.degrees(math.atan2(25.7, 44.0)), 'V2_side_wedges': math.degrees(math.atan2(78.0, 50.0)),
         'vertical_wall': 90.0}
wedge = {'faces_deg': faces, 'min_face_angle_to_lift_deg': {}}
for mu in (0.2, 0.3, 0.45, 0.6):
    wedge['min_face_angle_to_lift_deg'][str(mu)] = 90 - math.degrees(math.atan(mu))
wedge['vertical_impulse_per_unit_normal'] = {f: {str(mu): vertical_impulse_sign(a, mu) for mu in (0.3, 0.6)}
                                            for f, a in faces.items()}
wedge['tooth_lowest_point_mm'] = AXLE_Z - TIP_R
wedge['tooth_front_reach_y_mm'] = ROTOR_Y + TIP_R
out['5_wedge_contact'] = wedge

# ------------------------------------------------------------------ 6. impact (upper-bound impulse model)
m_r = I_ROT / (TIP_R / 1000) ** 2
v_tip = 0.9 * w_noload(V_FULL) * TIP_R / 1000
imp = {'tip_speed_m_s_90pct_12.6V': v_tip, 'rotor_tip_effective_mass_kg': m_r}
for name, m_o in (('V2_1.087kg', 1.087), ('R3_1.237kg', 1.237), ('heavy_2.0kg', 2.0)):
    mu_red = m_r * m_o / (m_r + m_o)
    for e in (0.0, 0.3, 0.5):
        J = (1 + e) * mu_red * v_tip
        imp['%s|e%.1f' % (name, e)] = {'impulse_Ns': J, 'opp_dv_m_s': J / m_o,
                                       'opp_rise_if_all_vertical_mm': (J / m_o) ** 2 / (2 * G) * 1000,
                                       'opp_energy_J': J ** 2 / (2 * m_o),
                                       'share_of_rotor_energy_to_opp_motion': J ** 2 / (2 * m_o) / (0.5 * m_r * v_tip ** 2),
                                       'rotor_tip_speed_after_fraction': 1 - J / (m_r * v_tip),
                                       'v4_recoil_m_s': J / M}
# calibration against the MuJoCo head-on test: V2 charging, median opponent CoM peak (rest CoM ~30.5 mm)
wt = json.load(open(os.path.join(V4, 'mujoco_sim', 'wedge_test_r3.json')))
z_med = wt['results']['r3']['headon|now|V2|charging']['opp_max_com_z_mm_median']
rise = (z_med - 30.5) / 1000
J_sim = 1.087 * math.sqrt(2 * G * rise)
J_calc = imp['V2_1.087kg|e0.3']['impulse_Ns']
imp['sim_calibration'] = {'V2_median_com_peak_mm': z_med, 'rise_mm': rise * 1000,
                          'vertical_impulse_implied_Ns': J_sim, 'calc_impulse_e0.3_Ns': J_calc,
                          'sim_over_calc': J_sim / J_calc}
forces = {}
for tag, J in (('calc_e0.3_vs_V2', J_calc), ('sim_calibrated_vs_V2', J_sim)):
    forces[tag] = {'%.1fms' % (dt * 1000): {'avg_N': J / dt, 'peak_halfsine_N': math.pi / 2 * J / dt}
                   for dt in (0.3e-3, 0.5e-3, 1e-3, 2e-3)}
imp['tooth_force_demand'] = forces
out['6_impact'] = imp

# ------------------------------------------------------------------ 7. strength capacities (tooth-tip force F, N)
cap = {}
w_root = 2 * BASE_R * math.sin(math.radians(TOOTH_DEG / 2))          # chord of the tooth at the base circle
t = 6.0
lever = TIP_R - BASE_R
for g in ('disc_steel_S275_yield', 'disc_steel_hardened_AR400_yield'):
    sy = MAT[g]
    cap['tooth_root_bending|' + g] = sy * t * w_root ** 2 / (6 * lever)
    cap['tooth_root_shear|' + g] = 0.577 * sy * w_root * t
# disc-to-hub screws: struck disc passes F radially (no spigot: disc bore r7 vs spacer r5.5) and the torque of the
# rest of the rotor tangentially, through 4 M3 at r 13. Per-screw shear = |F/4 radial + F*r_tip*(1-share)/(4*r_s)|
share_rest = 1 - i_disc / I_ROT
per_screw_per_N = math.hypot(1 / 4, TIP_R * share_rest / (4 * SCREW_R))
A_m3 = 5.03
for g in ('bolt_8.8_yield', 'bolt_10.9_yield', 'A2-70_yield'):
    cap['disc_screws_shear|' + g] = 0.577 * MAT[g] * A_m3 / per_screw_per_N
preload = 0.75 * 0.9 * MAT['bolt_8.8_yield'] * A_m3 * 0.75      # 75 % of proof x 0.75 torque scatter (ASSUMED)
cap['disc_screws_friction_slip_mu0.15_8.8'] = 4 * 0.15 * preload / per_screw_per_N / 4
cap['disc_screws_note'] = 'per-screw shear factor %.3f x F; rest-of-rotor inertia share %.2f' % (per_screw_per_N, share_rest)
# 608 bearings: struck disc centre at x 23, bearings at +-16.5 -> near bearing carries F*(23+16.5)/33
x_d = sum(DISC_X) / 2
near = (x_d + BRG_X) / (2 * BRG_X)
far = (x_d - BRG_X) / (2 * BRG_X)
cap['bearing_608_static_C0'] = MAT['bearing_608_C0_N'] / near
cap['bearing_load_factor_near'] = near


def shaft_moment(P, a, L, fixed):
    """Max |M| (N*mm) for point loads P at a (from left support) on span L."""
    xs = np.linspace(0, L, 601)
    Mx = np.zeros_like(xs)
    for p, ai in zip(P, a):
        bi = L - ai
        if fixed:
            MA, MB = -p * ai * bi ** 2 / L ** 2, -p * ai ** 2 * bi / L ** 2
            RA = p * bi ** 2 * (3 * ai + bi) / L ** 3
            Mx += MA + RA * xs - p * np.clip(xs - ai, 0, None)
        else:
            Mx += p * bi / L * xs - p * np.clip(xs - ai, 0, None)
    return float(np.max(np.abs(Mx)))


L = 2 * UPR_X
loads = [near, -far]
pos = [UPR_X - BRG_X, UPR_X + BRG_X]
Z8 = math.pi * 8 ** 3 / 32
for fixed in (False, True):
    m1 = shaft_moment(loads, pos, L, fixed)
    for g in ('bolt_8.8_yield', 'bolt_12.9_yield'):
        cap['dead_shaft_M8_%s|%s' % ('fixed' if fixed else 'simply_supported', g)] = MAT[g] * Z8 / m1
# upright: shaft hole bearing in 6 mm 6061; reaction at the near upright ~ (L/2 + ...) solve simply supported
RA = sum(p * (L - ai) / L for p, ai in zip(loads, pos))
cap['upright_hole_bearing_6061'] = MAT['al6061T6_bearing_yield'] * 8.0 * 6.0 / RA
# upright -> tub: 4x M3 through a 16 mm TPU block per side (bearing on the TPU hole wall); load ~ RA * F
for s in MAT['tpu95a_bearing_allow']:
    cap['upright_mount_TPU_block_bearing_%dMPa' % s] = 4 * 3.0 * 16.0 * s / RA
cap['upright_reaction_factor'] = RA
cap['hinge_pin_3mm_double_shear_A2-70_N'] = 2 * 0.577 * MAT['A2-70_yield'] * math.pi * 1.5 ** 2
cap['disc_screws_note'] += '; friction slip is not a failure - after slip the screws carry it in shear'
ranked = sorted(((k, v) for k, v in cap.items() if isinstance(v, float) and 'factor' not in k
                 and not k.startswith(('hinge', 'disc_screws_friction'))), key=lambda kv: kv[1])
out['7_capacity_N_at_tooth'] = cap
out['7_ranked_weakest_first'] = ranked[:12]

# ------------------------------------------------------------------ 8. electrical
el = {'esc_continuous_A': ESC_A, 'motor_table_max_A': MOTOR_MAX_A,
      'battery_label_continuous_A': CAP_MAH / 1000 * C_RATING}
for rb in R_BATT:
    el['sag_V_at_40A|Rb%.0fmOhm' % (rb * 1000)] = 40 * rb
    el['sag_V_at_100A|Rb%.0fmOhm' % (rb * 1000)] = 100 * rb
# match energy budget (parameters - match length is not in the known rules)
budget = {}
spin_mAh = su['Rb15mOhm|ramp1600']['battery_mAh']
recover_mAh = su['after_hit_to_50%|Rb15mOhm']['battery_mAh'] - 0.0
for minutes in (2, 3):
    for n_hits, drive_avg_A in ((10, 1.5), (30, 3.0)):
        weapon_idle = I0 * minutes * 60 / 3.6                     # rotor at speed, no-load current
        drive = drive_avg_A * minutes * 60 / 3.6
        logic = 0.15 * minutes * 60 / 3.6                         # ESP32/IMU via BEC, ~0.15 A from the pack (ASSUMED)
        total = spin_mAh + n_hits * recover_mAh * 0.5 + weapon_idle + drive + logic
        budget['%dmin|%dhits|drive%.1fA' % (minutes, n_hits, drive_avg_A)] = {
            'total_mAh': total, 'pct_of_80pct_usable': total / (0.8 * CAP_MAH) * 100,
            'parts_mAh': {'first_spinup': spin_mAh, 'respins': n_hits * recover_mAh * 0.5, 'weapon_idle': weapon_idle,
                          'drive': drive, 'logic': logic}}
el['match_budget'] = budget
el['match_budget_note'] = ('re-spin per hit = half of a 50%->full recovery (ASSUMED average); drive average current ASSUMED; '
                           'match length not in the known rules - confirm with the organiser')
el['drv8871_two_motors_stall_A'] = 2 * DRIVE_STALL_A
el['drv8871_peak_A'] = DRV_PEAK_A
out['8_electrical'] = el

# ------------------------------------------------------------------ 9. drive and push
dr = {}
loads_N = np.array(REST_LOADS)
for v in (V_FULL, V_NOM):
    tq = DRIVE_TQ12 * v / 12
    f_motor = tq / (WHEEL_R / 1000)
    for mu in (0.4, 0.6, 0.8):
        per = np.minimum(f_motor, mu * loads_N)
        even = np.minimum(f_motor, mu * M * G / 4) * 4
        d15 = M * G * np.array([0.425, 0.425, 0.075, 0.075])
        dr['%.1fV|mu%.1f' % (v, mu)] = {'push_N_R3_loads': float(per.sum()), 'push_N_even_25pct': float(even),
                                         'push_N_if_rear_15pct': float(np.minimum(f_motor, mu * d15).sum()),
                                         'per_wheel_N': per.round(2).tolist(), 'motor_limit_per_wheel_N': f_motor,
                                         'loss_vs_even_pct': (1 - per.sum() / even) * 100}
    dr['%.1fV|top_speed_no_load_m_s' % v] = DRIVE_RPM12 * v / 12 * 2 * math.pi / 60 * WHEEL_R / 1000
dr['sim_measured_wall_push_N'] = {'V4_old': 8.5, 'V2': 6.1, 'R3': 7.5, 'source': 'mujoco_sim/REPORT.md validation'}
out['9_drive'] = dr

# ------------------------------------------------------------------ 10. mass and balance levers
mb = {'mass_g': mass['total_g'], 'margin_to_2000_g': 2000 - mass['total_g'], 'cg_y_mm': CG[1],
      'rear_load_from_cg': (FRONT_Y - CG[1]) / (FRONT_Y - REAR_Y), 'rear_load_build': mass['rear_static_load_fraction']}
wb = FRONT_Y - REAR_Y
for target in (0.10, 0.15):
    y_need = FRONT_Y - target * wb
    dm = (CG[1] - y_need) * mass['total_g']                  # g*mm of nose-down moment to remove
    mb['target_%d%%' % (target * 100)] = {
        'cg_y_needed_mm': y_need, 'cg_shift_needed_mm': CG[1] - y_need, 'moment_to_remove_gmm': dm,
        'max_add_at_y110_g': max(0.0, -dm / (110 - y_need)), 'max_add_at_y95_g': max(0.0, -dm / (95 - y_need)),
        'remove_at_y100_g': max(0.0, dm / (100 - y_need)),
        'rear_ballast_at_y-100_g': max(0.0, dm / (y_need + 100)),
        'battery_move_back_mm': max(0.0, dm / 80.0)}
out['10_mass_balance'] = mb

json.dump(out, open(os.path.join(HERE, 'r3_calc.json'), 'w'), indent=1, default=float)


# ------------------------------------------------------------------ console summary
def p(*a):
    print(*a)


p('== rotor', {k: (round(v, 4) if isinstance(v, float) else v) for k, v in rot.items() if not isinstance(v, dict)})
for k in ('12.6V_full', '11.1V_nominal'):
    p('  ', k, {a: round(b, 3) for a, b in rot[k].items()})
p('== spin-up check vs sim', {k: round(v, 3) if isinstance(v, float) else v for k, v in su['check_vs_sim']['calc'].items()},
  'sim rpm', round(su['check_vs_sim']['sim_rpm']), 'sim A', round(su['check_vs_sim']['sim_peak_A'], 2))
for k, v in su.items():
    if k != 'check_vs_sim':
        p('  %-34s t90 %s  rpm %.0f  Imot %.0f  Ibat %.0f  tq %.3f  %.1f mAh  %.0f J' % (
            k, None if v['t90_s'] is None else round(v['t90_s'], 3), v['rpm_end'], v['peak_motor_A'],
            v['peak_battery_A'], v['peak_rotor_torque_Nm'], v['battery_mAh'], v['battery_energy_J']))
p('== gyro', json.dumps(gy, default=float))
p('== pitch', json.dumps(pitch, default=float))
p('== bite', json.dumps(bite, default=float))
p('== wedge', json.dumps(wedge, default=float))
p('== impact', json.dumps({k: v for k, v in imp.items() if k != 'tooth_force_demand'}, default=float))
p('   forces', json.dumps(forces, default=float))
p('== capacity ranked')
for k, v in ranked:
    p('   %-55s %8.0f N' % (k, v))
p('   factors', cap['disc_screws_note'], 'bearing near', round(near, 3), 'upright RA', round(RA, 3))
p('== electrical', json.dumps({k: v for k, v in el.items() if k != 'match_budget'}, default=float))
for k, v in budget.items():
    p('   %-28s %.0f mAh (%.0f%% of usable)  %s' % (k, v['total_mAh'], v['pct_of_80pct_usable'],
                                                 {a: round(b) for a, b in v['parts_mAh'].items()}))
p('== drive')
for k, v in dr.items():
    p('  ', k, v)
p('== mass', json.dumps(mb, default=float))
