"""V4 sizing: vertical dual-disc beater, 4WD JGA25-370 drive, mass budget.

Reduced-order hand calculations, not FEA. Every input that is an estimate
is marked ASSUMED; supplier numbers are cited in V4_Plan.md.
Run: python3 design_v4.py   (writes v4_design.json)
Units: SI unless the name ends in _mm or _g.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
STEEL = 7850.0

# ---------------------------------------------------------------- geometry
AXLE_Z_MM = 32.0          # rotor and wheel axle height above floor
WHEEL_D_MM = 64.0         # printed TPU wheel, protrudes above and below body
TRACK_M = 0.140           # wheel contact centre-to-centre (wheels inside 180 mm body)
WHEELBASE_M = 0.090
DISC_T_MM = 6.0           # laser-cut steel tooth plate thickness
DISC_GAP_MM = 40.0        # inner faces of the two discs
BASE_R_MM = 23.0          # disc base circle
TOOTH_R_MM = 29.5         # tooth tip radius -> floor clearance 2.5 mm
TOOTH_SPAN_DEG = 60.0     # angular width of each tooth
N_TEETH = 2
BORE_R_MM = 7.0           # clears the 608 inner race; disc retains the outer race
LIGHTEN_FRAC = 0.0        # holes are modelled explicitly below (same as the CAD)
DISC_HOLES = [(13.0, a, 1.7) for a in (45, 135, 225, 315)] + [(17.5, a, 4.0) for a in (90, 270)]  # (r, deg, hole r)


def disc_props(t_mm):
    """Mass and polar inertia of one tooth plate by polar integration."""
    r = np.linspace(BORE_R_MM, TOOTH_R_MM, 600) / 1000.0
    th = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    R, TH = np.meshgrid(r, th)
    tooth = np.zeros_like(TH, bool)
    for k in range(N_TEETH):
        c = 2 * math.pi * k / N_TEETH
        d = np.angle(np.exp(1j * (TH - c)))
        tooth |= np.abs(d) < math.radians(TOOTH_SPAN_DEG / 2)
    inside = (R <= BASE_R_MM / 1000.0) | tooth
    X, Y = R * np.cos(TH), R * np.sin(TH)
    for rc, ac, rh in DISC_HOLES:
        cx, cy = rc / 1000 * math.cos(math.radians(ac)), rc / 1000 * math.sin(math.radians(ac))
        inside &= (X - cx) ** 2 + (Y - cy) ** 2 > (rh / 1000) ** 2
    dA = (r[1] - r[0]) * (th[1] - th[0]) * R
    rho_t = STEEL * t_mm / 1000.0
    w = inside * dA * rho_t
    base = R <= BASE_R_MM / 1000.0
    w = np.where(base & ~tooth, w * (1 - LIGHTEN_FRAC), w)
    return float(w.sum()), float((w * R ** 2).sum())


def rotor(t_mm):
    m_d, i_d = disc_props(t_mm)
    # 6061 hub OD30/ID22 x 40 mm with belt groove, carries two 608 bearings (CAD 32.3 g)
    ro, ri, L = 0.015, 0.011, DISC_GAP_MM / 1000.0
    m_t = 2700.0 * math.pi * (ro ** 2 - ri ** 2) * L
    i_t = 0.5 * m_t * (ro ** 2 + ri ** 2)
    m_p, i_p = 0.0, 0.0                                  # groove is cut into the hub
    m_b, i_b = 0.024 + 0.008, 2 * 0.004 * 0.011 ** 2     # 2x 608 (12 g) + 8x M3 countersunk screws
    m = 2 * m_d + m_t + m_p + m_b
    i = 2 * i_d + i_t + i_p + i_b
    return {'disc_mass_g': m_d * 1000, 'rotor_mass_g': m * 1000, 'rotor_I_kgm2': i,
            'width_mm': DISC_GAP_MM + 2 * t_mm}


# ---------------------------------------------------------------- weapon motor
# Turnigy/DYS D3536 1250 kV, 102 g (seller data). R and I0 ASSUMED typical.
KV = 1250.0
KE = 60.0 / (2 * math.pi * KV)      # V*s/rad == N*m/A
R_M = 0.050
I0 = 1.5
J_MOTOR = 1.5e-5                    # outrunner bell ~50 g at r 17 mm (ASSUMED)
V_PACK = 11.1                       # 3S under load
I_LIMIT = 40.0                      # firmware throttle ramp keeps ESC/pack below this
ETA_BELT = 0.92


def spin_up(i_rotor, ratio, t_end=6.0, dt=1e-3):
    """Weapon speed vs time; ratio = motor rpm / weapon rpm."""
    J = J_MOTOR + i_rotor / ratio ** 2 / ETA_BELT
    wm, t, rows = 0.0, 0.0, []
    w_max = (V_PACK - I0 * R_M) / KE
    t90 = None
    while t < t_end:
        amps = min((V_PACK - KE * wm) / R_M, I_LIMIT)
        torque = KE * max(amps - I0, 0.0)
        wm += torque / J * dt
        t += dt
        if t90 is None and wm >= 0.9 * w_max:
            t90 = t
        rows.append((t, wm / ratio, amps))
    return w_max / ratio, t90, rows


def spin_up_ramp(i_rotor, ratio, ramp_us_per_s, t_end=4.0, dt=1e-4):
    """Firmware throttle ramp (1000->2000 us) with no ESC current limit: peak current and t90."""
    J = J_MOTOR + i_rotor / ratio ** 2 / ETA_BELT
    w_max = (V_PACK - I0 * R_M) / KE
    wm = t = peak = 0.0
    t90 = None
    while t < t_end:
        thr = min(1.0, ramp_us_per_s * t / 1000.0)
        amps = max((thr * V_PACK - KE * wm) / R_M, 0.0)
        peak = max(peak, amps)
        wm += KE * max(amps - I0, 0.0) / J * dt
        t += dt
        if t90 is None and wm >= 0.9 * w_max:
            t90 = t
    return {'ramp_us_per_s': ramp_us_per_s, 'peak_current_A': peak, 't_spin_90pct_s': t90}


# ---------------------------------------------------------------- drive
# JGA25-370 12 V, 400 rpm variant. Stall torque INTERPOLATED from the
# Precision Microdrives table (620 rpm 0.8 kg*cm, 280 rpm 1.7 kg*cm), stall 1.3 A.
DRIVE_NL_RPM_12V = 400.0
DRIVE_STALL_NM_12V = 0.8 * 620.0 / 400.0 * 0.0981
N_DRIVE = 4


def drive(mass_kg, mu):
    v = DRIVE_NL_RPM_12V * V_PACK / 12.0 * math.pi * WHEEL_D_MM / 1000.0 / 60.0
    f_motor = N_DRIVE * DRIVE_STALL_NM_12V * V_PACK / 12.0 / (WHEEL_D_MM / 2000.0)
    f_trac = mu * mass_kg * G
    return {'top_speed_m_s': v, 'motor_push_N': f_motor, 'traction_push_N': f_trac,
            'push_N': min(f_motor, f_trac), 'limited_by': 'motors' if f_motor < f_trac else 'traction'}


# ---------------------------------------------------------------- mass budget (g)
def mass_budget(rotor_g):
    items = {
        'weapon rotor (2 discs + tube + pulley + bearings)': rotor_g,
        'dead shaft M8x70 12.9 + nuts/washers': 30,
        '6061 shaft spacers ID8.2/OD11 (7 + 26 + 7 mm)': 5,
        'heat-set inserts + lid/wedgelet/brace screws': 15,
        'weapon motor D3536 1250kV (seller)': 102,
        'motor pulley + 5 mm PU round belt': 15,
        'weapon ESC Skywalker 40A (ASSUMED)': 35,
        'drive motors JGA25-370 x4 (PMD datasheet 110 g)': 440,
        'wheels x4: lightened PETG hub 20 g (80% of 25 g solid) + TPU tyre 25 g (Fusion)': 180,
        'ESP32 DevKit': 10,
        'DRV8871 x2': 10,
        '3S 850 mAh 80C LiPo (GNB, ASSUMED 75 g + leads)': 80,
        'sensors: GY-521 IMU, A3144 + magnet + PETG post, 2x DS18B20, divider, wires': 12,
        'power switch / removable link + fuse': 15,
        'wiring, connectors': 50,
        # solid TPU would be 516 g (Fusion 430.1 incl. skirt knuckles + 55.9 g lid, as Nylon 6, x1.21/1.14);
        # printed with 4 walls + 60% gyroid the part is ~85% of solid (ASSUMED, weigh the print)
        'tub + 2 mm lid TPU printed (85% of 516 g solid)': 439,
        'wedgelet hinge blocks + 4 carriers PETG (Fusion 18.0 g as Nylon 6, x1.27/1.14, 80% printed)': 16,
        'wedgelet hinge pins 3 mm steel x2 + E-clips (Fusion)': 6.5,
        'weapon uprights 6061 6 mm x2 (Fusion)': 66,
        'weapon braces + motor mount 6061 3 mm (Fusion ~20 g)': 20,
        'front steel wedgelets 2 mm x2 (Fusion 53.6 g)': 54,
        'hinged skirts: 1 mm Al plates 20.6 g + 1.5 mm wire 8.2 g + PETG clips 2.5 g (Fusion) + M2 screws/nuts 4 g': 35.3,
        'other fasteners (motor, upright, ESC mounts) (ASSUMED)': 25,
    }
    total = sum(items.values())
    return items, total


def main():
    out = {'assumptions': 'reduced-order; see V4_Plan.md'}
    variants = []
    for t in (6.0, 8.0):
        rot = rotor(t)
        for ratio in (1.0, 1.25, 1.5):
            w, t90, rows = spin_up(rot['rotor_I_kgm2'], ratio)
            w_op = 0.9 * w
            e = 0.5 * rot['rotor_I_kgm2'] * w_op ** 2
            items, total_g = mass_budget(rot['rotor_mass_g'])
            m = total_g / 1000.0
            restoring = m * G * TRACK_M / 2
            yaw_ok = restoring / (rot['rotor_I_kgm2'] * w_op)
            bite = 2.0 * 2 * math.pi / (N_TEETH * w_op) * 1000.0   # at 2 m/s closing
            variants.append({
                'disc_t_mm': t, 'ratio': ratio, **rot,
                'weapon_rpm_no_load': w * 60 / (2 * math.pi),
                'weapon_rpm_operating': w_op * 60 / (2 * math.pi),
                'tip_speed_m_s': w_op * TOOTH_R_MM / 1000.0,
                'energy_J': e, 't_spin_90pct_s': t90,
                'max_yaw_rate_before_wheel_lift_rad_s': yaw_ok,
                'bite_mm_at_2m_s': bite,
                'tooth_height_mm': TOOTH_R_MM - BASE_R_MM,
                'robot_mass_g': total_g,
            })
    out['variants'] = variants

    # chosen: highest energy that turns >= 10 rad/s without lifting a wheel and
    # keeps >= 100 g (5%) under the 2 kg cap for scale error, glue and wiring growth
    ok = [v for v in variants if v['max_yaw_rate_before_wheel_lift_rad_s'] >= 10.0
          and v['bite_mm_at_2m_s'] <= v['tooth_height_mm'] and v['robot_mass_g'] <= 1900]
    pick = max(ok, key=lambda v: v['energy_J']) if ok else max(variants, key=lambda v: v['max_yaw_rate_before_wheel_lift_rad_s'])
    out['chosen'] = pick
    items, total_g = mass_budget(pick['rotor_mass_g'])
    out['mass_budget_g'] = items
    out['mass_total_g'] = total_g
    out['mass_margin_to_2kg_g'] = 2000 - total_g
    out['firmware_ramp'] = [spin_up_ramp(pick['rotor_I_kgm2'], pick['ratio'], r) for r in (500, 1000, 1600, 2500)]
    # after a hit the rotor slows at full throttle: current with the rotor at 50% speed
    out['current_after_hit_50pct_speed_A'] = (V_PACK - KE * 0.5 * (V_PACK - I0 * R_M) / KE) / R_M
    out['drive'] = {f'mu_{mu}': drive(total_g / 1000.0, mu) for mu in (0.4, 0.6, 0.8)}

    # hit result (rough): fraction eta of rotor energy goes into opponent motion
    hits = {}
    for m_o in (1.0, 1.5, 2.0):
        for eta in (0.2, 0.4):
            v = math.sqrt(2 * eta * pick['energy_J'] / m_o)
            hits[f'{m_o}kg_eta{eta}'] = {'launch_v_m_s': v, 'launch_h_m_no_spin': v * v / (2 * G)}
    out['hit_launch_estimate'] = hits

    # dead shaft check: impulse from 1.5 kg launched at the eta=0.4 speed over 1 ms
    v = hits['1.5kg_eta0.4']['launch_v_m_s']
    F = 1.5 * v / 1e-3
    a = (DISC_T_MM / 2 + 5.0) / 1000.0           # disc centre to support face (ASSUMED 5 mm gap)
    M = F / 2 * a
    d = 0.008
    sigma = 32 * M / (math.pi * d ** 3)
    out['shaft_check'] = {
        'peak_force_N_assumed_1ms': F, 'moment_Nm': M, 'bending_MPa': sigma / 1e6,
        'grade_8.8_yield_MPa': 640, 'grade_12.9_yield_MPa': 1080,
        'bearing_608_static_C0_N': 1370, 'load_per_bearing_N': F / 2,
    }
    json.dump(out, open(os.path.join(HERE, 'v4_design.json'), 'w'), indent=2)

    print('variant  t  ratio  rotor_g  E_J  rpm   t90_s  yaw_ok  bite')
    for v in variants:
        print('%5.0f %5.2f %7.0f %5.0f %6.0f %6.2f %6.1f %5.1f' % (
            v['disc_t_mm'], v['ratio'], v['rotor_mass_g'], v['energy_J'], v['weapon_rpm_operating'],
            v['t_spin_90pct_s'] or -1, v['max_yaw_rate_before_wheel_lift_rad_s'], v['bite_mm_at_2m_s']))
    print('chosen:', {k: round(v, 4) if isinstance(v, float) else v for k, v in pick.items()})
    print('mass total g %.0f  margin %.0f' % (total_g, 2000 - total_g))
    print('drive', json.dumps(out['drive'], indent=1))
    print('hits', json.dumps(hits, indent=1))
    print('ramp', json.dumps(out['firmware_ramp'], indent=1), 'after-hit A', round(out['current_after_hit_50pct_speed_A']))
    print('shaft', json.dumps(out['shaft_check'], indent=1))


if __name__ == '__main__':
    main()
