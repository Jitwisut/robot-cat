"""V5 concept sizing: SawBlaze-style hammersaw (arm + saw) with forks, <=2 kg.

Reduced-order hand calculations, not FEA, for choosing the concept before CAD.
SawBlaze (BattleBots heavyweight, ~113 kg) is used for LAYOUT RATIOS ONLY:
real parts do not scale with the cube law, so every V5 part is sized from
1-2 kg-class components. Ratios were measured from the bounding boxes of the
5 meshes in SawBlazeExport.fbx (units unknown, so only ratios are used).
Every estimate is marked ASSUMED; prices come from robot_v4/V4_Plan.md and
robot_v4/print_revision/catalog/PROCUREMENT.md unless marked ASSUMED.

Axes follow the V4 convention: Z up, nose +Y, X lateral, mm in names ending _mm.
Run: python3 design_v5.py   (writes v5_design.json)
Units: SI unless the name ends in _mm, _g or _thb.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
STEEL = 7850.0
AL = 2700.0
RNG = np.random.default_rng(20261027)

# ------------------------------------------------ SawBlaze layout ratios (FBX)
# Viewer frame was Y-up with the forks toward +Z; mapped here to Z-up, nose +Y.
SB = {
    'overall_width': 132.6,           # wheel outer face to wheel outer face
    'body_width': 100.0,              # Box001 X span
    'wheel_d': 31.0,                  # Cylinder329/330 height span
    'axle_to_fork_tip': 90.3,         # wheel centre (0.5) to Box005 front (90.8)
    'axle_to_body_rear': 15.0,        # wheels sit at the very back
    'arm_pivot_height': 35.0,         # densest arm vertices behind the body, above the wheels
    'arm_pivot_to_blade_centre': 64.0,  # pivot (~5, 35) to blade centre (~45, 85)
    'blade_d': 46.0,
    'drive': '2 rear wheels, forks drag/skid at the front',
}
RATIO = {k: v / SB['overall_width'] for k, v in SB.items() if isinstance(v, float)}

# ------------------------------------------------ V5 envelope from the ratios
W_MM = 200.0                          # overall width, same class as V4 (206 mm)
geom = {k: round(r * W_MM, 1) for k, r in RATIO.items()}
WHEEL_D_MM = 64.0                     # real part: V4 PETG hub + TPU tyre (ratio would give 47 mm, too small to drive inverted)
ARM_L_MM = 100.0                      # pivot -> blade centre (ratio gives 97 mm)
PIVOT_H_MM = 90.0                     # pivot above floor (ratio 53 mm); CAD 2026-10-27: 90 so the arm clears the lid in the strike pose
AXLE_TO_TIP_MM = 136.0                # ratio x W
WHEELBASE_MM = 90.0                   # 4WD option: same as V4
TRACK_MM = 140.0

# ------------------------------------------------ saw options (on the arm tip)
V4_ENERGY_J = 57.39                   # robot_v4/v4_design.json chosen.energy_J
V4_BITE_MM = 4.84                     # at 2 m/s closing, 2 teeth
MOTOR_KV = 1250.0                     # D3536 1250kV (same motor V4 chose)
V_BATT = 11.1                         # 3S under load
R_MOTOR = 0.075                       # ohm, ASSUMED (3536-class outrunner)
I_LIMIT = 40.0                        # A, Skywalker 40A
KT = 60.0 / (2 * math.pi * MOTOR_KV)  # N m / A


def disc(d_mm, bore_mm, t_mm, extra_g=0.0):
    ro, ri, t = d_mm / 2000, bore_mm / 2000, t_mm / 1000
    m = STEEL * math.pi * (ro ** 2 - ri ** 2) * t + extra_g / 1000
    return m, 0.5 * m * (ro ** 2 + ri ** 2)


SAWS = {
    # ASSUMED: 85 mm x 10 mm bore TCT mini-circular-saw blade, 1.1 mm plate + 24 carbide tips (~0.3 g each).
    # Rated max rpm must be read off the blade; 10,000 rpm is ASSUMED until then.
    'A_tct_blade_85mm_24T': dict(d=85, bore=10, t=1.1, extra_g=24 * 0.3, teeth=24, rpm_cap=10000, belt=1.0),
    # 3 mm steel saw-disc, 3 teeth (bite), SawBlaze look. Mass and inertia come from the CAD solid when built.
    'B_steel_disc_85mm_3mm_3T': dict(d=85, bore=10, t=3.0, extra_g=0.0, teeth=3, rpm_cap=None, belt=1.0),
}
HUB_I = 3.0e-6                        # arbor + 2x 608 inner races + driven pulley, ASSUMED
HUB_G = 20.0
# Measured from robot_v5/cad/build_v5.py (disc outline with tooth gaps, real arm). The full-annulus estimate
# above over-states disc B's inertia because the tooth gaps remove outer rim, and I grows with r^4.
CAD_PATH = os.path.join(HERE, 'cad', 'output', 'v5_cad_props.json')
CAD = json.load(open(CAD_PATH)) if os.path.exists(CAD_PATH) else None


def spin_up(i_total, w_max):
    """Current-limited DC motor model; time to 90 % of loaded speed."""
    w, t, dt = 0.0, 0.0, 1e-4
    w_nl = KT ** -1 * V_BATT
    target = 0.9 * min(w_max, 0.9 * w_nl)
    while w < target and t < 10:
        i = min(I_LIMIT, (V_BATT - KT * w) / R_MOTOR)
        w += KT * max(i - 1.5, 0) / i_total * dt      # 1.5 A no-load current, ASSUMED
        t += dt
    return t


saw = {}
for name, s in SAWS.items():
    m, i_d = disc(s['d'], s['bore'], s['t'], s['extra_g'])
    w_motor_op = 0.9 * KT ** -1 * V_BATT          # loaded motor speed, rad/s
    w = w_motor_op * s['belt']
    if s['rpm_cap']:
        w = min(w, s['rpm_cap'] * 2 * math.pi / 60)
    i_tot = i_d + HUB_I
    if CAD and name.startswith('B_'):
        m = CAD['disc']['mass_g'] / 1000
        i_tot = CAD['disc']['spinning_I_kgm2 (disc + hub + half bearings)']
    e = 0.5 * i_tot * w ** 2
    rps = w / (2 * math.pi)
    saw[name] = {
        'mass_g': round(m * 1000, 1),
        'I_kgm2': i_tot,
        'rpm': round(w * 60 / (2 * math.pi)),
        'tip_speed_m_s': round(w * s['d'] / 2000, 1),
        'energy_J': round(e, 1),
        'energy_vs_V4': round(e / V4_ENERGY_J, 2),
        't_spin_90pct_s': round(spin_up(i_tot, w), 2),
        # Bite per tooth when the arm brings the saw down at v_close
        'bite_mm_at_arm_1m_s': round(1.0 / (s['teeth'] * rps) * 1000, 3),
        'bite_mm_at_drive_2m_s': round(2.0 / (s['teeth'] * rps) * 1000, 3),
        # Same disc with fewer teeth bites deeper (SawBlaze look = many teeth)
        'bite_mm_at_arm_1m_s_by_teeth': {n: round(1.0 / (n * rps) * 1000, 2) for n in (2, 3, 4, 6, 24)},
    }

# ------------------------------------------------ arm actuator (servo) options
# (stall N m, no-load rad/s, mass g, price THB) at the stated voltage, ASSUMED from typical listings
SERVOS = {
    # Runs at 5 V from the SkyWalker 30A V2 BEC (budget build). Seller spec (mikroelec 3424): 19 kg cm, 0.16 s/60 deg
    # at 5 V; 735 THB at mikroelec.com 2026-10-27 (the DS3218MG Pro listing; Lazada RC surpass hobby 801 THB)
    'DS3218 19kg-cm @5V':  (1.86, math.radians(60) / 0.16, 60, 735),
    'DS3235SG 35kg-cm @6.8V': (3.43, math.radians(60) / 0.11, 65, 650),
    'DS5160 60kg-cm @8.4V':  (5.88, math.radians(60) / 0.12, 150, 1100),
}
ARM_PLATE_T_MM, ARM_PLATE_W_MM = 3.0, 25.0
arm_plates_g = 2 * AL * (ARM_L_MM + 20) / 1000 * ARM_PLATE_W_MM / 1000 * ARM_PLATE_T_MM / 1000 * 1000
ARBOR_BEARINGS_G = 2 * 12.0           # 608ZZ: 688 would be 9.5x over C0 on a hit (delivery_checks)
PULLEY_BELT_ON_ARM_G = 8.0
SWING_DEG = CAD['swing_deg'] if CAD else 150.0   # park -> strike on the stop posts


def arm_inertia(saw_mass_kg):
    """(I about the pivot kg m^2, first moment kg m with the arm horizontal).
    From the CAD arm when built, shifted for a different saw mass at the arbor."""
    L = ARM_L_MM / 1000
    if CAD:
        dm = saw_mass_kg - CAD['disc']['mass_g'] / 1000
        return CAD['arm']['I_pivot_kgm2'] + dm * L ** 2, CAD['arm']['first_moment_kgm (horizontal)'] + dm * L
    tip = saw_mass_kg + (HUB_G + ARBOR_BEARINGS_G + PULLEY_BELT_ON_ARM_G) / 1000
    plates = arm_plates_g / 1000
    return tip * L ** 2 + plates * ((L + 0.02) ** 2) / 3, tip * L + plates * (L + 0.02) / 2


def strike(stall, w0, i_arm, m_r):
    """Integrate the strike: servo torque-speed line + gravity, from parked (SWING_DEG above the strike line)
    down to the forward horizontal (saw on the forks). Positive = striking direction."""
    L = ARM_L_MM / 1000
    th, w, t, dt, work = 0.0, 0.0, 0.0, 1e-4, 0.0
    while th < math.radians(SWING_DEG) and t < 2:
        tau_servo = max(stall * (1 - w / w0), 0)
        ang = math.radians(SWING_DEG) - th           # angle above the forward horizontal
        tau_g = m_r * G * math.cos(ang)              # opposes while the arm is behind vertical
        acc = (tau_servo + tau_g) / i_arm
        w += acc * dt
        th += w * dt
        work += tau_servo * w * dt
        t += dt
    return {'t_strike_s': round(t, 3), 'tip_speed_m_s': round(w * L, 2),
            'arm_KE_J': round(0.5 * i_arm * w ** 2, 2), 'servo_work_J': round(work, 2)}


# ------------------------------------------------ forks
FORK_N, FORK_W_MM, FORK_T_MM, FORK_L_MM = 4, 12.0, 3.0, 70.0
FORK_MATS = {'mild steel S275': 275.0, '65Mn spring steel, hardened': 785.0}
fork_g = FORK_N * STEEL * FORK_W_MM * FORK_T_MM * FORK_L_MM * 1e-9 * 1000


def fork_check(robot_mass):
    z = FORK_W_MM * FORK_T_MM ** 2 / 6                   # mm^3
    i_sec = FORK_W_MM * FORK_T_MM ** 3 / 12               # mm^4
    out = {}
    cases = {
        'opponent 2 kg on one tip x3 g': 2.0 * G * 3,
        'own robot dropped on one tip x5 g': robot_mass * G * 5,
    }
    for c, f in cases.items():
        sig = f * FORK_L_MM / z
        out[c] = {'F_N': round(f, 1), 'stress_MPa': round(sig, 1),
                  'SF': {k: round(v / sig, 2) for k, v in FORK_MATS.items()}}
    # Spinner hit: fork as a cantilever spring absorbing a share of the opponent's energy
    for e_hit in (2.0, 5.0):
        k = 3 * 200e3 * i_sec / FORK_L_MM ** 3              # N/mm
        f = math.sqrt(2 * e_hit * 1000 * k)                  # N (E in N mm)
        sig = f * FORK_L_MM / z
        out[f'{e_hit:.0f} J absorbed by one fork'] = {'F_N': round(f), 'stress_MPa': round(sig),
                                                      'SF': {kk: round(v / sig, 2) for kk, v in FORK_MATS.items()}}
    return out


# ------------------------------------------------ mass budget (concept, ASSUMED unless noted)
def mass_budget(saw_key, servo_key, drive):
    s = saw[saw_key]
    b = {
        f'saw: {saw_key}': s['mass_g'],
        'saw arbor/hub + 2x 608 + driven pulley': HUB_G + ARBOR_BEARINGS_G,
        'saw motor D3536 1250kV (seller, same as V4)': 102,
        'saw ESC Skywalker 40A (V4 assumption)': 35,
        'PU round belt + motor pulley (motor coaxial with arm pivot)': 15,
        f'arm: 2x 6061 plates {ARM_PLATE_T_MM:.0f}x{ARM_PLATE_W_MM:.0f}x{ARM_L_MM + 20:.0f} mm': round(arm_plates_g, 1),
        'arm pivot: M8 dead shaft + 2x 608 + spacers': 40,
        'arm pivot towers 6061 4 mm x2': 40,
        f'arm servo {servo_key}': SERVOS[servo_key][2],
        'servo horn/coupling + slip clutch (ASSUMED)': 20,
        'UBEC 6.8 V 8 A for servo': 15,
        f'forks: {FORK_N}x steel {FORK_W_MM:.0f}x{FORK_T_MM:.0f}x{FORK_L_MM:.0f} mm': round(fork_g, 1),
        'fork hinge blocks PETG + pins': 28,
        'ESP32 DevKit': 10,
        'DRV8871 x2': 10,
        '3S 850 mAh LiPo (V4 assumption)': 80,
        'sensors (IMU, hall, temp) + wiring + connectors + switch/fuse': 12 + 50 + 15,
        'fasteners + inserts': 40,
        'body: TPU tub + lid (V4 442 g, ~5 % smaller footprint)': 420,
    }
    if drive == '4WD':
        b['drive JGA25-370 x4 (110 g)'] = 440
        b['wheels x4 PETG hub + TPU tyre'] = 180
    else:
        b['drive JGA25-370 x2 (110 g)'] = 220
        b['wheels x2 PETG hub + TPU tyre'] = 90
        b['front skid UHMW under forks'] = 6
    return b


# ------------------------------------------------ drive / traction
MOTOR_PUSH_PER_MOTOR_N = 14.065 / 4   # V4 model: JGA25-370 400 rpm, 64 mm wheel
FORK_MU = 0.2                          # steel tip on plastic/tile, ASSUMED


def drive_check(mass_kg, drive, cg_ahead_of_axle_mm):
    w = mass_kg * G
    out = {}
    for mu in (0.4, 0.6, 0.8):
        if drive == '4WD':
            # hinged forks rest on the floor with ~half their own weight; everything else on 4 wheels
            n_fork = fork_g / 1000 / 2 * G
            n_wheels = w - n_fork
            motors = 4
        else:
            # rigid forks: statics between rear axle and fork tips
            frac_tip = cg_ahead_of_axle_mm / AXLE_TO_TIP_MM
            n_fork = w * frac_tip
            n_wheels = w - n_fork
            motors = 2
        traction = mu * n_wheels
        motor = motors * MOTOR_PUSH_PER_MOTOR_N
        push = min(traction, motor) - FORK_MU * n_fork
        out[f'mu_{mu}'] = {'wheel_load_frac': round(n_wheels / w, 2), 'traction_N': round(traction, 2),
                           'motor_N': round(motor, 2), 'fork_drag_N': round(FORK_MU * n_fork, 2),
                           'net_push_N': round(push, 2)}
    return out


# ------------------------------------------------ self-righting with the arm
def self_right(mass_kg, stall):
    # Inverted: bot pivots on its rear edge; arm reaches back over the pivot and pushes the floor.
    if CAD and CAD['self_right'].get('status') == 'ok':
        # CAD side-view statics at first floor contact (build_v5.py self_right), scaled to this mass
        need = CAD['self_right']['servo_torque_needed_Nm'] * mass_kg / (CAD['mass_total_g'] / 1000)
        return {'torque_needed_Nm': round(need, 2), 'servo_stall_Nm': stall, 'margin': round(stall / need, 2),
                'source': 'CAD', 'note': 'first-contact statics; need margin >= 1.5'}
    d_cg = 0.045                       # CG horizontal distance from the rear tipping edge, ASSUMED
    need = mass_kg * G * d_cg          # N m at the arm pivot (lever ratio ~1, pivot over the rear axle)
    return {'torque_needed_Nm': round(need, 2), 'servo_stall_Nm': stall,
            'margin': round(stall / need, 2), 'note': 'need margin >= 1.5 (servo torque falls with speed and battery sag)'}


# ------------------------------------------------ hit on the arm from a spinner (servo survival)
def servo_hit():
    # Same impulse model as V4's shaft check: V4-class 57 J hit, 1 ms contact, at the blade.
    f_peak = 8298.0                   # N, robot_v4/v4_design.json shaft_check.peak_force_N_assumed_1ms
    tau = f_peak * ARM_L_MM / 1000
    return {'peak_force_N': f_peak, 'torque_at_pivot_Nm': round(tau), 'servo_gear_limit_Nm_ASSUMED': 10,
            'conclusion': 'servo gears strip unless the arm is decoupled by a slip clutch / compliant link'}


# ------------------------------------------------ does the stored energy reach the opponent?
def delivery_checks(s, i_arm):
    """Arm backing mass and arbor/pivot strength, same 1 ms impulse model as V4's shaft_check."""
    d_contact = (ARM_L_MM + 42.5) / 1000            # pivot -> front of the disc
    m_eff = i_arm / d_contact ** 2
    v = math.sqrt(2 * 0.4 * s['energy_J'] / 1.5)    # 1.5 kg opponent, eta 0.4 (V4 model)
    f = 1.5 * v / 1e-3
    a = (3.0 / 2 + 5.0) / 1000                       # disc centre to arm plate, 5 mm gap ASSUMED
    m_bend = f / 2 * a
    d = 0.008
    sigma = 32 * m_bend / (math.pi * d ** 3) / 1e6
    return {
        'arm_effective_mass_at_disc_front_kg': round(m_eff, 3),
        'v4_backing_mass_kg (rotor on frame)': 1.89,
        'rule_hard_stop': 'in the strike pose the arm must rest on a frame hard stop, or each hit throws the arm up and slips the clutch',
        'rule_spin_direction': 'spin +X (right-hand, nose +Y, Z up): front teeth move up, bottom teeth move forward; '
                               'the reaction then drives the arm down onto the stop for both front and top contacts',
        'peak_force_N_1ms': round(f), 'v4_peak_force_N_1ms': 8298,
        'arbor_M8_bending_MPa': round(sigma), 'grade_8.8_yield_MPa': 640, 'grade_12.9_yield_MPa': 1080,
        'load_per_bearing_N': round(f / 2),
        'overload_vs_C0': {'688 (C0 ~590 N, ASSUMED)': round(f / 2 / 590, 1), '608 (C0 1370 N)': round(f / 2 / 1370, 1),
                           'V4 608 for comparison': round(4149 / 1370, 1)},
        'pivot_608_overload_if_no_hard_stop': round(f / 2 / 1370, 1),
    }


# ------------------------------------------------ cost (THB)
def cost(servo_key, drive, owns_esp32_and_charger=False):
    c = {
        'saw motor D3536 1250kV (V4_Plan typical)': 560,
        'Skywalker 40A (catalog TH03)': 490,
        '3S 850 mAh 150C (catalog TH07)': 490,
        'DRV8871 x2 (catalog TH05)': 150,
        f'JGA25-370 x{4 if drive == "4WD" else 2} @185 (V4_Plan)': 185 * (4 if drive == '4WD' else 2),
        f'servo {servo_key} (ASSUMED)': SERVOS[servo_key][3],
        'UBEC 8 A (ASSUMED)': 150,
        'saw: 85 mm TCT blade or laser-cut 3 mm disc (ASSUMED)': 250,
        'bearings 4x 608 (@18 catalog TH08)': 72,
        'belt + pulleys (ASSUMED)': 120,
        'arm plates + pivot towers 6061, laser/CNC (ASSUMED)': 600,
        'forks 4x steel, laser-cut (ASSUMED)': 400,
        'TPU body + PETG parts + wheels, self-print (V4 BOM)': 550,
        'fasteners, inserts, wiring, XT30 (ASSUMED)': 300,
        'sensors IMU/hall/temp (ASSUMED)': 100,
        'PS4-clone pad (owned-parts memory)': 350,
    }
    if not owns_esp32_and_charger:
        # STATUS.md 8 Oct + catalog TH04 say the board is not owned yet; memory of 3 Oct says it is
        c['ESP32 30pin (catalog TH04)'] = 180
        c['charger iMAX B6AC (V4_Plan, cybertice)'] = 640
    return c


# ------------------------------------------------ matchup (same opponent set as robot_v4/matchup_screen.py)
N = 50000
SIG = 0.7
OPP = {
    'V2 robot2 (wedge+lifter)':  (1.09, [(-31, 31, 2.07, 2.07), (31, 80, 0.0, 0.0), (-80, -31, 0.0, 0.0)], (8.5, 8.5), (0, 0)),
    'R3 (lifting wedge 4WD)':    (1.24, [(-84, 84, 0.5, 1.5)], (9.6, 9.6), (0, 0)),
    'Generic low wedge':         (1.8, [(-80, 80, 0.3, 3.0)], (6.0, 14.0), (0, 0)),
    'Horizontal bar spinner':    (1.8, [(-80, 80, 3.0, 8.0)], (4.0, 9.0), (40, 150)),
    'Vertical spinner':          (1.8, [(-30, 30, 1.0, 4.0), (30, 80, 0.5, 3.0), (-80, -30, 0.5, 3.0)], (5.0, 10.0), (30, 120)),
    'Control/pusher (no weapon)': (2.0, [(-80, 80, 1.0, 5.0)], (10.0, 16.0), (0, 0)),
}
V5_ZONES = [(-100, 100, 0.5, 1.5)]    # hinged forks resting on the floor: same edge range as V4's wedgelets
V4_PUSH = (6.2, 12.4)                 # robot_v4/matchup_screen.py; V5 4WD uses the same motors, wheels and mass


def u(lo, hi):
    return RNG.uniform(lo, hi, N)


def matchup(e_saw, push_range=V4_PUSH):
    res = {}
    v5_push = u(*push_range)
    v5_edge = u(V5_ZONES[0][2], V5_ZONES[0][3]) + RNG.normal(0, SIG, N)
    for name, (m, zones, push, energy) in OPP.items():
        off = u(-60, 60)
        under = np.zeros(N, bool)
        opp_under = np.zeros(N, bool)
        for (ox0, ox1, lo, hi) in zones:
            oe = u(lo, hi) + RNG.normal(0, SIG, N)
            overlap = (off + ox1 > -100) & (off + ox0 < 100)
            under |= overlap & (v5_edge < oe - 0.2)
            opp_under |= overlap & (oe < v5_edge - 0.2)
        r = {'p_v5_forks_under': round(float(under.mean()), 2),
             'p_opponent_under_v5': round(float(opp_under.mean()), 2),
             'p_v5_wins_push': round(float((v5_push > u(*push)).mean()), 2)}
        if energy[1] > 0:
            r['p_saw_energy_higher'] = round(float((e_saw > u(*energy)).mean()), 2)
        res[name] = r
    return res


def main():
    out = {'note': 'concept screening; SawBlaze used for ratios only', 'sawblaze_fbx_measured': SB,
           'ratios_over_overall_width': {k: round(v, 3) for k, v in RATIO.items()},
           'v5_from_ratios_at_W200_mm': geom,
           'v5_chosen_geometry_mm': {'overall_width': W_MM, 'wheel_d': WHEEL_D_MM, 'arm_pivot_to_saw': ARM_L_MM,
                                     'arm_pivot_height': PIVOT_H_MM, 'axle_to_fork_tip': AXLE_TO_TIP_MM},
           'saw': saw, 'v4_reference': {'energy_J': V4_ENERGY_J, 'bite_mm_2m_s': V4_BITE_MM}}

    configs = {}
    for saw_key in SAWS:
        for servo_key in SERVOS:
            for drive in ('4WD', '2WD'):
                mb = mass_budget(saw_key, servo_key, drive)
                m_g = sum(mb.values())
                m = m_g / 1000
                i_arm, m_r = arm_inertia(saw[saw_key]['mass_g'] / 1000)
                stall, w0 = SERVOS[servo_key][:2]
                hold = m_r * G
                cg = 35.0 if drive == '2WD' else None
                dr = drive_check(m, drive, cg or 0)
                cst = cost(servo_key, drive, owns_esp32_and_charger=True)
                configs[f'{saw_key} | {servo_key} | {drive}'] = {
                    'mass_g': round(m_g), 'margin_to_2kg_g': round(2000 - m_g),
                    'cost_thb_if_esp32_charger_owned': sum(cst.values()),
                    'cost_thb_if_buying_esp32_charger': sum(cst.values()) + 820,
                    'arm_hold_torque_Nm': round(hold, 3), 'arm_hold_margin': round(stall / hold, 1),
                    'arm_I_kgm2': i_arm, 'strike': strike(stall, w0, i_arm, m_r),
                    'self_right': self_right(m, stall), 'drive': dr,
                    'yaw_rate_limit_rad_s (gyro lifts a wheel)': round(m * G * TRACK_MM / 2000 / (saw[saw_key]['I_kgm2'] * saw[saw_key]['rpm'] * 2 * math.pi / 60), 1),
                }
    out['configs'] = configs

    # DS3218 over DS3235SG: self-right margin 2.3 (>1.5) and 350 THB more budget headroom;
    # the strike is speed-limited, so the stronger servo does not hit harder.
    rec = 'B_steel_disc_85mm_3mm_3T | DS3218 19kg-cm @5V | 4WD'
    out['recommended'] = rec
    out['recommended_mass_budget_g'] = mass_budget('B_steel_disc_85mm_3mm_3T', 'DS3218 19kg-cm @5V', '4WD')
    out['recommended_cost_thb'] = cost('DS3218 19kg-cm @5V', '4WD')
    out['recommended_cost_total_thb'] = {
        'ESP32 + charger to buy (STATUS.md 8 Oct, catalog TH04)': sum(cost('DS3218 19kg-cm @5V', '4WD').values()),
        'ESP32 + charger already owned (memory 3 Oct)': sum(cost('DS3218 19kg-cm @5V', '4WD', True).values()),
    }
    out['forks'] = fork_check(configs[rec]['mass_g'] / 1000)
    out['servo_hit'] = servo_hit()
    p = configs[rec]['drive']
    out['matchup_recommended'] = matchup(saw['B_steel_disc_85mm_3mm_3T']['energy_J'])
    out['delivery_checks'] = delivery_checks(saw['B_steel_disc_85mm_3mm_3T'], configs[rec]['arm_I_kgm2'])
    json.dump(out, open(os.path.join(HERE, 'v5_design.json'), 'w'), indent=2, ensure_ascii=False)

    print('SAW'); [print(' ', k, v) for k, v in saw.items()]
    print('CONFIGS')
    for k, v in configs.items():
        print(' ', k, '| mass', v['mass_g'], '| THB', v['cost_thb_if_buying_esp32_charger'], '| hold x', v['arm_hold_margin'],
              '| strike', v['strike'], '| right x', v['self_right']['margin'],
              '| push mu0.6', v['drive']['mu_0.6']['net_push_N'], '| yaw lim', v['yaw_rate_limit_rad_s (gyro lifts a wheel)'])
    print('FORKS', json.dumps(out['forks'], ensure_ascii=False))
    print('SERVO HIT', out['servo_hit'])
    print('DELIVERY', json.dumps(out['delivery_checks'], ensure_ascii=False))
    print('COST', out['recommended_cost_total_thb'])
    print('MATCHUP', json.dumps(out['matchup_recommended'], ensure_ascii=False))


if __name__ == '__main__':
    main()
