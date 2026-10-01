"""Reduced-order duel model: robot2 (V2 wedge + lifter) vs ROBOT_V3 R3 (lifting wedge).

This is not a 3D contact simulation. Each engagement is split into:
  1. geometry: whose leading edge is lower at the moment of contact
     (edge heights from the Fusion CAD, robot pitch stance, height noise);
  2. forces: traction-limited push contest with wedge weight transfer, and
     whether the lifter that got underneath can lift the other robot's wheels.
A Monte Carlo over the declared uncertain parameters gives probabilities.

Run:  python3 duel_sim.py      (writes results.json, mc_samples.csv, *.png)
Conventions: y forward from each robot's own origin, z up, heights in mm
above the floor unless named otherwise. Forces N, masses kg.
"""
import csv
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
N_MC = 20000
RNG = np.random.default_rng(20260925)

# --------------------------------------------------------------------------
# Mass properties. Both robots use the same maker masses for bought parts
# (servo 66 g, motor 61 g, ESC 9 g, UBEC 21 g, ER6 14.5 g), the same assumed
# 3S battery (80 g) and the same 30 g wiring allowance. R3 also gets the
# motor cradle + face mount + hub robot2 carries (27.9 g per motor) and the
# same 26.2 g wheels, so neither robot is lighter just because it is less
# finished in CAD.
# --------------------------------------------------------------------------
MAKER = {'servo': 66.0, 'motor': 61.0, 'esc': 9.0, 'ubec': 21.0, 'rx': 14.5,
         'battery': 80.0, 'wiring': 30.0}


def robot2_mass():
    rows = [l.split('|') for l in open(os.path.join(HERE, 'robot2_bodies.txt')).read().split('\n') if l]
    sub = {'Repeat_40kg_Servo_MaxEnvelope': MAKER['servo'], 'BBB_22mm_Gearmotor_L': MAKER['motor'],
           'BBB_22mm_Gearmotor_R': MAKER['motor'], 'BBB_Dual_Brushed_ESC_v2_Envelope': MAKER['esc'],
           'UBEC_5A': MAKER['ubec'], 'Rx_ER6': MAKER['rx'], 'Battery_and_Tray': MAKER['battery']}
    m = my = mz = 0.0
    for name, mass, _x, y, z in rows:
        mass = sub.get(name, float(mass))
        m += mass; my += mass * float(y); mz += mass * float(z)
    m += MAKER['wiring']; my += MAKER['wiring'] * -40; mz += MAKER['wiring'] * 20  # wiring sits by the battery
    return m / 1000.0, my / m, mz / m


def r3_mass():
    # From the R3 Fusion body list (conceptual coords, z above floor = z-2).
    rows = [l.split('|') for l in open(os.path.join(HERE, 'r3_bodies.txt')).read().split('\n') if l]
    m = my = mz = 0.0
    for name, mass, _x, y, z in rows:
        mass = float(mass)
        if 'Gearmotor' in name:
            mass = MAKER['motor'] + 27.9
        elif name.startswith('Wheel_'):
            mass = 26.2
        elif name.startswith('Repeat_40kg_Servo'):
            mass = MAKER['servo']
        elif name.startswith('3S_Battery'):
            mass = MAKER['battery']
        elif name.startswith('RadioMaster_ER6'):
            mass = MAKER['rx']
        elif name.startswith('BBB_Dual_ESC'):
            mass = MAKER['esc']
        elif name.startswith('Hobbywing_UBEC'):
            mass = MAKER['ubec']
        m += mass; my += mass * float(y); mz += mass * (float(z) - 2.0)
    m += MAKER['wiring']; my += MAKER['wiring'] * -10; mz += MAKER['wiring'] * 15
    return m / 1000.0, my / m, mz / m


M2, CG2_Y, CG2_Z = robot2_mass()   # robot2 axle at y=0
M3, CG3_Y, CG3_Z = r3_mass()       # R3 axles at y=+58 and y=-60

# --------------------------------------------------------------------------
# Drive: BBB 22 mm gearmotor v2 (maker: 12 V, stall 4.3 A, no-load 0.3 A,
# ~890 rpm no-load). Maker gives no stall torque, so the output-shaft motor
# constant is ESTIMATED from those numbers; gearbox load loss is a swept
# efficiency. Both robots: 42 mm wheels, 3S, BBB Dual ESC v2 (8 A/channel,
# current limited). R3 puts 2 motors on each channel, robot2 1.
# --------------------------------------------------------------------------
R_MOTOR = 12.0 / 4.3
KE = (12.0 - 0.3 * R_MOTOR) / (890 * 2 * math.pi / 60)   # V*s/rad at output
I0 = 0.3
R_WHEEL = 0.021


def motor_force(v, volts, eta, amps_limit_per_motor):
    """Tractive force of one motor at wheel speed v (m/s)."""
    omega = np.maximum(v, 0.0) / R_WHEEL
    amps = np.minimum((volts - KE * omega) / R_MOTOR, amps_limit_per_motor)
    return np.maximum(0.0, eta * KE * (amps - I0) / R_WHEEL)


# --------------------------------------------------------------------------
# Geometry from Fusion (heights above floor, mm).
# robot2 rests on its two centre wheels and tips onto a skid: the base
# plate's front edge (y=+90) or rear edge (y=-90), both 3 mm up when level.
# Its centre of mass is only ~2 mm ahead of the axle, so its stance flips
# with throttle: braking/impact -> nose down, accelerating/pushing -> nose up.
# --------------------------------------------------------------------------
PITCH = math.atan(3.0 / 90.0)            # 1.91 deg either way
R2_SPATULA_TIP = (112.0, 5.8)            # level CAD
R2_PIVOT = (68.0, 31.5)


def r2_height(y, z_level, stance):
    """Height of a robot2 point after pitching about the axle (stance +1 nose-down, -1 nose-up)."""
    return z_level - stance * y * math.tan(PITCH)


R2 = {
    'spatula_tip_down': r2_height(112, 5.8, +1),   # 2.07
    'spatula_tip_up': r2_height(112, 5.8, -1),     # 9.53
    'front_edge_down': 0.0,                        # base plate edge on the floor
    'front_edge_up': r2_height(90, 3.0, -1),       # 6.0
    'rear_edge_down': r2_height(-90, 3.0, +1),     # 6.0
    'rear_edge_up': 0.0,
    'side_level': 3.0,                             # base plate underside at the axle
    'tyre_y': (-21.0, 21.0),                       # wheels flush with the flanks
    'height': 85.0, 'width': 160.0, 'wheel_top': 42.0,
}

R3 = {
    'tip_bottom': 0.5, 'tip_top': 1.5,             # lifter and wings, same line
    'ramp_deg': math.degrees(math.atan((23 - 8.79) / 30.0)),  # 25.3
    'pivot': (80.5, 11.0),                         # (y, height)
    'lift_max_deg': 60.0,
    'side_guard_bottom': 6.0, 'rail_bottom': 1.0,
    'lid_top': 37.0, 'wheel_top': 42.0, 'width': 168.0, 'height': 42.0,
    'track': 0.148, 'wheelbase': 0.118,
}
# R3 flank, front (+) to rear (-): (y0, y1, kind, bottom height)
R3_FLANK = [(86, 115, 'wing', 2.5), (37, 79, 'tyre', 0.0), (29, 37, 'rail', 1.0),
            (-29, 29, 'guard', 6.0), (-39, -29, 'rail', 1.0), (-81, -39, 'tyre', 0.0)]
R3_TIP_FORCE = {1: 519, 5: 245, 10: 182, 20: 137, 30: 115, 40: 99, 50: 84, 60: 67}  # N, ideal stall


def r3_lift_gain(slide_in_mm, lift_deg):
    """Height gained by a point `slide_in_mm` behind R3's tip when the wedge lifts."""
    tip = (115.0, 1.0)
    ang = math.radians(R3['ramp_deg'])
    p = (tip[0] - slide_in_mm * math.cos(ang), tip[1] + slide_in_mm * math.sin(ang))
    dy, dz = p[0] - R3['pivot'][0], p[1] - R3['pivot'][1]
    t = math.radians(lift_deg)
    return (R3['pivot'][1] + dy * math.sin(t) + dz * math.cos(t)) - p[1], math.hypot(dy, dz)


# --------------------------------------------------------------------------
# Uncertain inputs: every draw is independent. Ranges are declared, not fitted.
# --------------------------------------------------------------------------
PARAMS = {
    'mu_r3':        ('uniform', 0.6, 1.0, 'tyre-floor friction R3'),
    'mu_r2':        ('uniform', 0.6, 1.0, 'tyre-floor friction robot2 (rubber tyres assumed)'),
    'mass_r3':      ('normal_rel', M3, 0.08, 'R3 mass, +/-8% (1 sigma)'),
    'mass_r2':      ('normal_rel', M2, 0.08, 'robot2 mass, +/-8% (1 sigma)'),
    'eta':          ('uniform', 0.6, 0.9, 'gearbox efficiency under load (estimate)'),
    'volts':        ('uniform', 10.8, 12.4, '3S pack voltage under load'),
    'edge_sigma':   ('fixed', 0.7, None, 'height noise per edge (mm, 1 sigma): tolerance, tyre squash, floor'),
    'cg2_dy':       ('normal', CG2_Y, 5.0, 'robot2 CoM ahead of axle (mm)'),
    'mu_contact':   ('uniform', 0.2, 0.4, 'metal-on-metal friction at the wedge contact'),
    'slide_in':     ('uniform', 5.0, 25.0, 'how far robot2 edge slides onto R3 lifter before lifting (mm)'),
    'r2_lift_N':    ('uniform', 30.0, 150.0, 'robot2 spatula tip force (linkage unsolved in CAD: bracket)'),
    'servo_derate': ('uniform', 0.5, 1.0, 'fraction of ideal stall torque the R3 servo delivers'),
}


def draw(n):
    s = {}
    for k, (kind, a, b, _desc) in PARAMS.items():
        if kind == 'uniform':
            s[k] = RNG.uniform(a, b, n)
        elif kind == 'normal_rel':
            s[k] = a * (1 + RNG.normal(0, b, n))
        elif kind == 'normal':
            s[k] = RNG.normal(a, b, n)
        else:
            s[k] = np.full(n, a)
    return s


def push_forces(mu3, mu2, m3, m2, eta, volts, v_transfer=0.0):
    """Max quasi-static push each robot can hold, stalled (v=0).

    v_transfer: vertical load robot2's front puts on R3's ramp (moves normal
    force from robot2's wheels to R3's). R3 traction is capped when that load
    drops R3's nose to the floor and unloads its rear axle.
    """
    f3_motor = 4 * motor_force(0.0, volts, eta, 4.0)   # 8 A shared by 2 motors
    f2_motor = 2 * motor_force(0.0, volts, eta, 8.0)
    # R3 nose-down check about the front axle (tip 57 mm ahead, CoM behind).
    w3 = m3 * G
    nose_cap = w3 * max(58.0 - CG3_Y, 1.0) / 57.0     # extra load before rear unloads
    n3 = w3 + np.minimum(v_transfer, nose_cap)
    n2 = np.maximum(m2 * G - v_transfer, 0.0)
    return np.minimum(f3_motor, mu3 * n3), np.minimum(f2_motor, mu2 * n2), f3_motor, f2_motor


def run_mc():
    s = draw(N_MC)
    n = N_MC
    sig = s['edge_sigma'][0]
    noise = lambda: RNG.normal(0, sig, n)
    r3_top = R3['tip_top'] + noise()
    out = {}

    # ---------------- HEAD-ON ----------------
    # At impact robot2 decelerates -> pitches nose-down (inertia >> W*d).
    tip_impact = R2['spatula_tip_down'] + noise()
    under_impact = r3_top < tip_impact - 0.2
    # If R3 did not get under at impact, the push continues. robot2 rocks
    # nose-up when its push moment H*h beats its weight moment W*d.
    f3, f2, f3m, f2m = push_forces(s['mu_r3'], s['mu_r2'], s['mass_r3'], s['mass_r2'], s['eta'], s['volts'])
    rock_up = f2 * 0.003 > s['mass_r2'] * G * np.maximum(s['cg2_dy'], 0) / 1000.0
    tip_push = R2['spatula_tip_up'] + noise()
    under_later = (~under_impact) & rock_up & (r3_top < tip_push - 0.2)
    under = under_impact | under_later
    # Once under robot2's spatula: can R3 lift robot2's wheels off the floor?
    # robot2 first rocks onto its rear skid, then rotates about it; its wheels
    # (90 mm ahead of the skid) rise 90/202 of the spatula tip rise.
    need_mm = 112 * math.tan(2 * PITCH) + 3.0 * 202 / 90   # rock + 3 mm wheel clearance
    gain = np.array([r3_lift_gain(si, R3['lift_max_deg'])[0] for si in s['slide_in']])
    radius = np.array([r3_lift_gain(si, R3['lift_max_deg'])[1] for si in s['slide_in']]) / 1000.0
    v_needed = s['mass_r2'] * G * (90 + s['cg2_dy']) / 202.0
    lift_force = R3_TIP_FORCE[60] * 0.0359 / radius * s['servo_derate']   # weakest point of the stroke
    # robot2's spatula has to hold while it is lifted: 0.044 m lever about its
    # pivot, against its own 40 kg servo through an unsolved linkage.
    spatula_holds = v_needed * 0.044 < 3.92 * 0.25    # needs >=25% of stall via linkage
    lifts = under & (gain >= need_mm) & (lift_force >= v_needed) & spatula_holds
    # Weight transfer while under but before/without lifting.
    tan_a = np.tan(math.radians(R3['ramp_deg']) + np.arctan(s['mu_contact']))
    v_tr = np.minimum(f2 / tan_a, s['mass_r2'] * G * 0.45)
    f3u, f2u, _, _ = push_forces(s['mu_r3'], s['mu_r2'], s['mass_r3'], s['mass_r2'], s['eta'], s['volts'], v_tr)
    push_margin_jam = f3 - f2
    push_margin_under = f3u - f2u
    headon_r3 = lifts | (~lifts & under & (push_margin_under > 0)) | (~under & (push_margin_jam > 0))
    out['head_on'] = {
        'p_r3_under_at_impact': float(under_impact.mean()),
        'p_r3_under_during_push': float(under.mean()),
        'p_r3_lifts_robot2_wheels': float(lifts.mean()),
        'p_r3_wins_pure_push_jammed': float((push_margin_jam > 0).mean()),
        'p_r3_wins_push_when_under_no_lift': float((push_margin_under > 0).mean()),
        'p_r3_controls': float(headon_r3.mean()),
        'lift_gain_needed_mm': need_mm,
        'lift_gain_available_mm_median': float(np.median(gain)),
        'robot2_front_load_to_lift_N_median': float(np.median(v_needed)),
        'r3_lift_force_at_contact_N_min': float(lift_force.min()),
    }

    # ---------------- R3 -> robot2 FLANK ----------------
    # robot2 is passive (nose-down). Its flank bottom slopes 0 mm (front) to
    # 6 mm (rear); the tyres sit flush on the flank at the axle.
    hit = RNG.uniform(-101, 101, n)                    # centre of R3's 79 mm lifter along robot2's side
    lo, hi = hit - 39.5, hit + 39.5
    spans_tyre = (hi > R2['tyre_y'][0]) & (lo < R2['tyre_y'][1])
    # best (highest) flank bottom inside the span, clipped to robot2's length
    y_best = np.clip(lo, -90, 90)                      # rear-most point in span is highest when nose-down
    bottom = 3.0 - (-y_best) * -math.tan(PITCH) + noise()
    under_f = spans_tyre | (r3_top < bottom - 0.2)
    lift_ok = R3_TIP_FORCE[60] * s['servo_derate'] > 0.5 * s['mass_r2'] * G
    r3_wins_flank = under_f & lift_ok
    out['r3_hits_robot2_flank'] = {
        'p_r3_under': float(under_f.mean()),
        'p_r3_lifts_near_wheel': float((r3_wins_flank & spans_tyre).mean()),
        'p_r3_lifts_side': float(r3_wins_flank.mean()),
        'note': 'hit point uniform along robot2 flank; lifter 79 mm wide',
    }

    # ---------------- robot2 -> R3 FLANK ----------------
    # Impact phase: robot2 nose-down (tip ~2.1 mm). Sustained push: nose-up (9.5 mm).
    hit = RNG.uniform(-82 - 31, 115 + 31, n)           # centre of robot2's 62 mm spatula along R3's side
    lo, hi = hit - 31, hit + 31

    def r2_gets_under(tip):
        got = np.zeros(n, dtype=bool)
        for y0, y1, kind, bottom in R3_FLANK:
            inside = (hi > y0) & (lo < y1)
            if kind == 'tyre':
                got |= inside & (tip < 21.0)            # below the axle, a round tyre climbs the wedge
            elif kind in ('guard', 'wing'):
                got |= inside & (tip < bottom + noise() - 0.2)
        return got

    tip_imp = R2['spatula_tip_down'] + noise()
    tip_push = R2['spatula_tip_up'] + noise()
    g_imp, g_push = r2_gets_under(tip_imp), r2_gets_under(tip_push)
    r2_lift_ok = s['r2_lift_N'] > 0.5 * s['mass_r3'] * G
    out['robot2_hits_r3_flank'] = {
        'p_robot2_under_at_impact': float(g_imp.mean()),
        'p_robot2_under_while_pushing': float(g_push.mean()),
        'p_robot2_lifts_r3_side': float(((g_imp | g_push) & r2_lift_ok).mean()),
        'note': 'hit point uniform along R3 side incl. wing; tyres exposed at the corners',
    }
    # Same attack with the proposed fix: 1 mm skirts over the whole flank.
    fixed = [(y0, y1, 'guard', 1.0) for (y0, y1, _k, _b) in R3_FLANK]
    got = np.zeros(n, dtype=bool)
    for y0, y1, _k, b in fixed:
        inside = (hi > y0) & (lo < y1)
        got |= inside & (tip_imp < b + noise() - 0.2)
    out['robot2_hits_r3_flank_with_1mm_skirts'] = {'p_robot2_under': float(got.mean())}

    # ---------------- REAR ATTACKS ----------------
    # R3 -> robot2 rear. robot2 fleeing = accelerating = nose-up (rear edge on floor);
    # robot2 parked/turning = nose-down (rear edge 6 mm up).
    rear_edge = np.where(RNG.uniform(0, 1, n) < 0.5, R2['rear_edge_up'], R2['rear_edge_down']) + noise()
    under_r = r3_top < rear_edge - 0.2
    out['r3_hits_robot2_rear'] = {
        'p_r3_under_if_robot2_fleeing': float((r3_top < R2['rear_edge_up'] + noise() - 0.2).mean()),
        'p_r3_under_if_robot2_parked': float((r3_top < R2['rear_edge_down'] + noise() - 0.2).mean()),
        'p_r3_under_50_50': float(under_r.mean()),
        'note': 'lifting robot2 rear by 6 mm raises its wheels 3 mm (they sit mid-way to the front skid)',
    }
    # robot2 -> R3 rear. robot2's 160 mm front meets R3's rear: the cross
    # member is 1 mm off the floor, but both rear tyres sit at the corners.
    x_off = RNG.uniform(-60, 60, n)
    tyre_hit = (np.abs(x_off) + 80 >= 68)             # robot2 front reaches |x| 68..80 on R3
    out['robot2_hits_r3_rear'] = {
        'p_robot2_catches_r3_rear_tyre': float(tyre_hit.mean()),
        'note': 'robot2 front edge (base plate on the floor when nose-down) wedges under an exposed rear tyre',
    }

    # ---------------- PURE PUSHING ----------------
    f3a, f2a, _, _ = push_forces(s['mu_r3'], 0.4 * np.ones(n), s['mass_r3'], s['mass_r2'], s['eta'], s['volts'])
    out['push'] = {
        'r3_traction_N_median': float(np.median(f3)),
        'robot2_traction_N_median': float(np.median(f2)),
        'r3_motor_limit_N_median': float(np.median(f3m)),
        'robot2_motor_limit_N_median': float(np.median(f2m)),
        'p_r3_out_pushes_equal_tyres': float((f3 > f2).mean()),
        'p_r3_out_pushes_if_robot2_abs_wheels_mu0p4': float((f3a > f2a).mean()),
    }

    # sensitivity: correlation of each input with the head-on and flank outcomes
    sens = {}
    for name, outcome in [('head_on_r3_controls', headon_r3), ('push_margin_jam', push_margin_jam),
                          ('robot2_lifts_r3_flank', (g_imp | g_push) & r2_lift_ok)]:
        y = outcome.astype(float)
        row = {}
        for k in PARAMS:
            x = s[k]
            if np.std(x) > 0 and np.std(y) > 0:
                row[k] = float(np.corrcoef(x, y)[0, 1])
        row['edge_noise_r3_tip'] = float(np.corrcoef(r3_top, y)[0, 1]) if np.std(y) > 0 else 0.0
        sens[name] = row
    out['sensitivity_corr'] = sens

    with open(os.path.join(HERE, 'mc_samples.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        keys = list(PARAMS)
        w.writerow(keys + ['r3_tip_top', 'r2_tip_impact', 'under_headon', 'lifts_headon', 'push_margin_jam_N',
                           'push_margin_under_N', 'headon_r3_controls', 'r2_under_r3_flank'])
        for i in range(n):
            w.writerow([round(float(s[k][i]), 5) for k in keys] +
                       [round(float(r3_top[i]), 3), round(float(tip_impact[i]), 3), int(under[i]), int(lifts[i]),
                        round(float(push_margin_jam[i]), 3), round(float(push_margin_under[i]), 3),
                        int(headon_r3[i]), int(g_imp[i] | g_push[i])])
    return out, s, push_margin_jam, push_margin_under, r3_top, tip_impact


def charge_and_shove(gap=0.40, dt=1e-4, t_end=2.5, mu3=0.8, mu2=0.8, eta=0.75, volts=11.6, lift=True):
    """1-D head-on: both full throttle from `gap` apart; then shove.

    Returns time series. After contact R3 is under robot2's spatula; with
    lift=True the servo raises the wedge (0.16 s/60 deg -> 82 deg of servo
    in 0.22 s) and robot2's wheels leave the floor at the lift gain needed.
    """
    m3, m2 = M3, M2
    x3, x2, v3, v2 = 0.0, gap, 0.0, 0.0   # v2 positive = toward R3
    t, contact_t = 0.0, None
    rows = []
    need = 112 * math.tan(2 * PITCH) + 3.0 * 202 / 90
    while t < t_end:
        if contact_t is None:
            f3 = min(4 * motor_force(v3, volts, eta, 4.0), mu3 * m3 * G)
            f2 = min(2 * motor_force(v2, volts, eta, 8.0), mu2 * m2 * G)
            v3 += f3 / m3 * dt; v2 += f2 / m2 * dt
            x3 += v3 * dt; x2 -= v2 * dt
            if x2 - x3 <= 0:
                contact_t = t
                closing = v3 + v2
                v = (m3 * v3 - m2 * v2) / (m3 + m2)   # perfectly plastic
                v3 = v; v2 = -v
        else:
            ts = t - contact_t
            servo_deg = min(ts / 0.16 * 60, 81.8) if lift else 0.0
            lift_deg = np.interp(servo_deg, [0, 9.4, 21.4, 31, 45.2, 56.7, 66.5, 74.9, 81.8],
                                 [0, 1, 5, 10, 20, 30, 40, 50, 60])
            gain = r3_lift_gain(15.0, lift_deg)[0]
            wheel_load_frac = 1.0 if gain < need else 0.0
            f3 = min(4 * motor_force(max(v3, 0), volts, eta, 4.0), mu3 * m3 * G)
            f2 = min(2 * motor_force(max(-v3, 0), volts, eta, 8.0), mu2 * m2 * G * wheel_load_frac)
            # robot2's skids drag when its wheels are up (steel/Al on floor ~0.3)
            drag = 0.0 if wheel_load_frac else 0.3 * m2 * G * 0.55
            a = (f3 - f2 - drag * np.sign(v3 if v3 else 1)) / (m3 + m2)
            v3 += a * dt
            x3 += v3 * dt; x2 = x3
        if int(t / dt) % 50 == 0:
            rows.append((t, x3, x2, v3, v2 if contact_t is None else -v3))
        t += dt
    return rows, contact_t, closing


def main():
    out, s, pm_jam, pm_under, r3_top, tip_imp = run_mc()
    rows_lift, tc, closing = charge_and_shove(lift=True)
    rows_push, _, _ = charge_and_shove(lift=False)
    x_end_lift = rows_lift[-1][1]
    x_end_push = rows_push[-1][1]
    out['time_sim'] = {
        'start_gap_m': 0.40, 'contact_time_s': tc, 'closing_speed_m_s': closing,
        'impact_force_scaled_from_583N_at_0p5_m_s': 583.0 * closing / 0.5,
        'r3_displacement_after_2p5s_lift_m': x_end_lift,
        'r3_displacement_after_2p5s_no_lift_m': x_end_push,
    }
    turn = {}
    for name, m, track, wb, n_wheels_front in [('R3_4WD', M3, R3['track'], R3['wheelbase'], 4),
                                               ('robot2_2WD', M2, 0.1416, 0.0, 2)]:
        mu = 0.8
        drive_moment = mu * m * G * track / 2
        scrub = mu * m * G * wb / 2
        iz = m * (0.168 ** 2 + 0.197 ** 2) / 12 if 'R3' in name else m * (0.160 ** 2 + 0.202 ** 2) / 12
        alpha = max(drive_moment - scrub, 1e-6) / iz
        turn[name] = {'yaw_accel_rad_s2': alpha, 't_90deg_s': math.sqrt(2 * (math.pi / 2) / alpha)}
    out['turning'] = turn
    out['inputs'] = {
        'robot2_mass_kg': M2, 'robot2_cg_ahead_of_axle_mm': CG2_Y, 'robot2_cg_height_mm': CG2_Z,
        'r3_mass_kg': M3, 'r3_cg_y_mm': CG3_Y, 'r3_cg_height_mm': CG3_Z,
        'motor_Ke_V_s_per_rad_estimate': KE, 'motor_R_ohm': R_MOTOR,
        'stall_force_per_motor_N_at_11p1V_eta0p75': motor_force(0, 11.1, 0.75, 8.0),
        'no_load_speed_m_s_at_11p1V': (11.1 - I0 * R_MOTOR) / KE * R_WHEEL,
        'robot2_edges_mm': R2, 'r3_edges_mm': {k: v for k, v in R3.items()},
        'params': {k: list(v) for k, v in PARAMS.items()}, 'n_mc': N_MC, 'seed': 20260925,
    }
    # flip / invert checks
    out['flip_invert'] = {
        'r3_invertible_wheel_top_vs_lid_mm': R3['wheel_top'] - R3['lid_top'],
        'robot2_invertible_wheel_top_vs_cover_mm': R2['wheel_top'] - R2['height'],
        'tilt_to_flip_robot2_sideways_deg': math.degrees(math.atan(80 / CG2_Z)),
        'tilt_to_flip_r3_sideways_deg': math.degrees(math.atan(84 / CG3_Z)),
        'r3_max_tip_lift_mm': 35.9,
        'robot2_side_lift_needed_to_flip_mm': 160 * math.sin(math.atan(80 / CG2_Z)),
    }
    with open(os.path.join(HERE, 'results.json'), 'w') as f:
        json.dump(out, f, indent=2, default=float)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # 1. time sim
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for rows, label, c in [(rows_lift, 'R3 under robot2 + lifts its wheels', '#1f6fb2'), (rows_push, 'edge-to-edge, push only', '#c0504d')]:
        t = [r[0] for r in rows]; x = [r[1] * 100 for r in rows]
        ax.plot(t, x, label=label, color=c)
    ax.axvline(tc, color='gray', ls=':', lw=1)
    ax.set_xlabel('time (s)'); ax.set_ylabel('distance R3 has pushed (cm)')
    ax.set_title('Head-on charge & shove (nominal: mu 0.8, eta 0.75, 11.6 V)')
    ax.legend(); ax.grid(alpha=.3); fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'headon_time_sim.png'), dpi=140)
    # 2. push margin histogram
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(pm_jam, bins=80, alpha=.6, label='jammed edge-to-edge', color='#888')
    ax.hist(pm_under, bins=80, alpha=.6, label='R3 under spatula (weight transfer, no lift)', color='#1f6fb2')
    ax.axvline(0, color='k', lw=1)
    ax.set_xlabel('push margin R3 - robot2 (N)  (>0 = R3 pushes robot2)'); ax.set_ylabel('count')
    ax.set_title('Pushing contest, Monte Carlo N=%d' % N_MC); ax.legend(); fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'push_margin_hist.png'), dpi=140)
    # 3. edge heights
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(tip_imp - r3_top, bins=80, color='#1f6fb2', alpha=.8)
    ax.axvline(0.2, color='k', lw=1)
    ax.set_xlabel('robot2 spatula tip minus R3 tip top at impact (mm)  (>0.2 = R3 gets under)')
    ax.set_title('Head-on edge heights at impact (robot2 nose-down)'); fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'headon_edge_margin.png'), dpi=140)
    # 4. sensitivity (only outcomes that depend on the swept inputs)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for ax, name in zip(axes, ['head_on_r3_controls', 'push_margin_jam']):
        row = out['sensitivity_corr'][name]
        items = sorted(row.items(), key=lambda kv: abs(kv[1]))[-7:]
        ax.barh([k for k, _ in items], [v for _, v in items],
                color=['#1f6fb2' if v > 0 else '#c0504d' for _, v in items])
        ax.set_title(name + '  (blue helps R3, red helps robot2)', fontsize=9)
        ax.axvline(0, color='k', lw=.8); ax.tick_params(labelsize=8)
    fig.suptitle('Which input moves the outcome (correlation over %d Monte Carlo runs)' % N_MC)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, 'sensitivity.png'), dpi=140)
    # 5. scenario summary
    bars = [
        ('Head-on: R3 gets under\n(during push)', out['head_on']['p_r3_under_during_push'], '#1f6fb2'),
        ('Head-on: R3 lifts\nrobot2 wheels', out['head_on']['p_r3_lifts_robot2_wheels'], '#1f6fb2'),
        ('Pure push (jammed):\nR3 wins', out['head_on']['p_r3_wins_pure_push_jammed'], '#1f6fb2'),
        ('R3 hits robot2 flank:\nR3 lifts side', out['r3_hits_robot2_flank']['p_r3_lifts_side'], '#1f6fb2'),
        ('R3 hits robot2 rear\n(robot2 parked)', out['r3_hits_robot2_rear']['p_r3_under_if_robot2_parked'], '#1f6fb2'),
        ('R3 hits robot2 rear\n(robot2 fleeing)', out['r3_hits_robot2_rear']['p_r3_under_if_robot2_fleeing'], '#1f6fb2'),
        ('robot2 hits R3 flank:\nrobot2 lifts R3', out['robot2_hits_r3_flank']['p_robot2_lifts_r3_side'], '#c0504d'),
        ('...same, with 1 mm\nside skirts on R3', out['robot2_hits_r3_flank_with_1mm_skirts']['p_robot2_under'], '#e3a3a1'),
        ('robot2 hits R3 rear:\ncatches a rear tyre', out['robot2_hits_r3_rear']['p_robot2_catches_r3_rear_tyre'], '#c0504d'),
    ]
    fig, ax = plt.subplots(figsize=(15, 5.2))
    ax.bar(range(len(bars)), [b[1] * 100 for b in bars], color=[b[2] for b in bars])
    for i, b in enumerate(bars):
        ax.text(i, b[1] * 100 + 1.5, '%.0f%%' % (b[1] * 100), ha='center', fontsize=9)
    ax.set_xticks(range(len(bars))); ax.set_xticklabels([b[0] for b in bars], fontsize=7.5)
    ax.set_ylim(0, 110); ax.set_ylabel('probability (%)')
    ax.set_title('Engagement outcomes (blue = good for R3, red = good for robot2), N=%d' % N_MC)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, 'scenario_summary.png'), dpi=140)
    print(json.dumps(out, indent=2, default=float))


if __name__ == '__main__':
    main()
