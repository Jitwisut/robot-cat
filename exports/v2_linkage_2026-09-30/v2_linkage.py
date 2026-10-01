"""Four-bar kinematics and ideal stall forces of the robot2 (V2) lifter linkage.

Geometry is the as-built CAD from fusion_scripts/Fix123Stage2.py and
AddWedgeLifterOption.py (robot2 native axes: y toward the nose, z above the
floor, mm). All links move in the y-z plane:
  servo output axis S  (42, 22)     horn pin A0 (57, 22)   horn 15 mm
  rod A0 -> B0         (57, 22) -> (63, 45)
  lifter pivot P       (68, 31.5)   lever pin B0 (63, 45)  on the hinge boss
  spatula tip T        (112, 5.8)   lowest point of the 62 mm blade tip
Lift angle th > 0 raises the tip (Fusion joint 0..45 deg). The horn angle phi
is measured from its CAD rest position (pointing +y); the branch that is
continuous from phi = 0 is followed.

Tip force uses the same definition as the V3 R3 table (build_robot_v3_r3.py):
ideal stall torque 3.92 N*m (40 kgf*cm), no friction, force perpendicular to
the pivot-to-tip radius.  Run:  python3 v2_linkage.py
"""
import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
S = (42.0, 22.0)
A0 = (57.0, 22.0)
P = (68.0, 31.5)
B0 = (63.0, 45.0)
T0 = (112.0, 5.8)
HORN = math.dist(S, A0)
ROD = math.dist(A0, B0)
LEVER = math.dist(P, B0)
TIP_R = math.dist(P, T0)
STALL_NM = 3.92
LIFT_MAX = 45.0
BAR_W = 6.0          # horn/rod/lever plate width in CAD
BAR_END = 4.0        # bar extends 4 mm past each pin centre in CAD
TOGGLE_REST = False  # True only for the toggle option (0 deg is singular there)


def rot(p, c, a):
    dy, dz = p[0] - c[0], p[1] - c[1]
    ca, sa = math.cos(a), math.sin(a)
    return (c[0] + dy * ca - dz * sa, c[1] + dy * sa + dz * ca)


def horn_angle(th, guess):
    """Horn rotation phi (rad) that keeps |A B| = ROD, nearest to guess."""
    b = rot(B0, P, th)
    d = math.dist(S, b)
    c = (HORN ** 2 + d ** 2 - ROD ** 2) / (2 * HORN * d)
    if abs(c) > 1:
        return None
    base = math.atan2(b[1] - S[1], b[0] - S[0])
    rest = math.atan2(A0[1] - S[1], A0[0] - S[0])
    sols = [base + s * math.acos(c) - rest for s in (1, -1)]
    sols = [math.atan2(math.sin(x), math.cos(x)) for x in sols]
    return min(sols, key=lambda x: abs(x - guess))


def state(th, guess):
    phi = horn_angle(th, guess)
    if phi is None:
        return None
    a = rot(A0, S, phi)
    b = rot(B0, P, th)
    t = rot(T0, P, th)
    return phi, a, b, t


def perp(u, v):
    return abs(u[0] * v[1] - u[1] * v[0])


def analyse(th_deg, guess):
    th = math.radians(th_deg)
    phi, a, b, t = state(th, guess)
    h = 1e-5
    dphi = (horn_angle(th + h, phi) - horn_angle(th - h, phi)) / (2 * h)
    rod_u = ((b[0] - a[0]) / ROD, (b[1] - a[1]) / ROD)
    # Moment arms of the rod line about each fixed axis (mm).
    arm_servo = perp((a[0] - S[0], a[1] - S[1]), rod_u)
    arm_pivot = perp((b[0] - P[0], b[1] - P[1]), rod_u)
    # Transmission angle: between rod and lever (90 deg is best).
    lev_u = ((b[0] - P[0]) / LEVER, (b[1] - P[1]) / LEVER)
    mu = math.degrees(math.acos(max(-1, min(1, abs(rod_u[0] * lev_u[0] + rod_u[1] * lev_u[1])))))
    horn_u = ((a[0] - S[0]) / HORN, (a[1] - S[1]) / HORN)
    horn_rod = math.degrees(math.acos(max(-1, min(1, abs(rod_u[0] * horn_u[0] + rod_u[1] * horn_u[1])))))
    pivot_torque = STALL_NM * abs(dphi)            # N*m at the lifter pivot
    tip_perp = pivot_torque / (TIP_R / 1000)
    horiz = (t[0] - P[0]) / 1000                   # lever arm for a vertical load
    tip_vert = pivot_torque / horiz if horiz > 1e-6 else float('inf')
    rod_force = STALL_NM / (arm_servo / 1000) if arm_servo > 1e-9 else float('inf')
    return phi, {
        'lift_deg': th_deg,
        'servo_deg': round(math.degrees(phi), 2),
        'tip_height_mm': round(t[1], 1),
        'tip_y_mm': round(t[0], 1),
        'ratio_dphi_dth': round(dphi, 3),
        'pivot_torque_Nm_stall': round(pivot_torque, 3),
        'tip_force_perp_N_stall': round(tip_perp, 1),
        'tip_force_vertical_N_stall': round(tip_vert, 1),
        'rod_force_N_stall': round(rod_force, 1),
        'rod_arm_about_servo_mm': round(arm_servo, 2),
        'rod_arm_about_pivot_mm': round(arm_pivot, 2),
        'transmission_angle_deg': round(mu, 1),
        'horn_rod_angle_deg': round(horn_rod, 1),
        'horn_pin_yz': [round(a[0], 2), round(a[1], 2)],
        'lever_pin_yz': [round(b[0], 2), round(b[1], 2)],
    }


def seg_dist(p, a, b):
    ab = (b[0] - a[0], b[1] - a[1])
    t = max(0, min(1, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / (ab[0] ** 2 + ab[1] ** 2)))
    q = (a[0] + t * ab[0], a[1] + t * ab[1])
    return math.dist(p, q)


def sweep():
    rows, guess = [], 0.0
    for d in [i * 0.5 for i in range(int(LIFT_MAX * 2) + 1)]:
        guess, r = analyse(max(d, 0.5) if TOGGLE_REST else d, guess)
        r['lift_deg'] = d
        a, b = r['horn_pin_yz'], r['lever_pin_yz']
        # Rod bar (x -31.1..-28.1) shares an x-slab with the fixed 6 mm pivot pin.
        r['rod_edge_to_pin_surface_mm'] = round(seg_dist(P, a, b) - BAR_W / 2 - 3.0, 2)
        # Servo torque needed to hold 100 N pressing straight down on the tip.
        horiz = (r['tip_y_mm'] - P[0]) / 1000
        r['servo_torque_Nm_per_100N_down'] = round(100 * horiz / abs(r['ratio_dphi_dth']), 3)
        rows.append(r)
    return rows


def lowered_contacts():
    """Lowering past 0 deg: angle where the tip meets the floor and where the
    blade underside meets Lifter_Lower_Hard_Stop (y 82..89, top z 13)."""
    def blade_gap(th):
        a, b = rot((70, 27), P, th), rot(T0, P, th)
        return min(a[1] + (y - a[0]) / (b[0] - a[0]) * (b[1] - a[1]) - 13 for y in (82, 85.5, 89))
    out = {}
    for name, f in [('tip_on_floor_deg', lambda th: rot(T0, P, th)[1]), ('blade_on_hard_stop_deg', blade_gap)]:
        lo, hi = math.radians(-20), 0.0
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if f(mid) < 0 else (lo, mid)
        out[name] = round(math.degrees(hi), 2)
    out['blade_to_stop_gap_at_rest_mm'] = round(blade_gap(0.0), 2)
    return out


def summarise(rows):
    table = [r for r in rows if r['lift_deg'] in (0, 1, 5, 10, 15, 20, 25, 30, 35, 40, 45)]
    return {
        'geometry_mm': {'horn_pin_rest_yz': [round(A0[0], 2), round(A0[1], 2)], 'horn': round(HORN, 2),
                        'rod': round(ROD, 2), 'lever': round(LEVER, 2), 'tip_radius': round(TIP_R, 2)},
        'servo_rotation_for_0_to_45deg_deg': round(abs(rows[-1]['servo_deg']), 2),  # rest is servo 0
        'min_transmission_angle_deg': min(r['transmission_angle_deg'] for r in rows),
        'min_rod_to_pin_clearance_mm': min(r['rod_edge_to_pin_surface_mm'] for r in rows),
        'min_tip_force_perp_N_stall': min(r['tip_force_perp_N_stall'] for r in rows),
        'max_rod_force_N_stall': max(r['rod_force_N_stall'] for r in rows),
        'table': table,
    }


def main():
    global A0, ROD, TOGGLE_REST
    out = {'as_built_cad': summarise(sweep()), 'lowered_contacts_as_built': lowered_contacts()}
    rows_cad = sweep()
    # Option studied, not in CAD: same servo axis, lever pin and 15 mm horn, but
    # the horn points at the lever pin at rest (toggle, like V3 R3), rod shortened.
    u = ((B0[0] - S[0]) / math.dist(S, B0), (B0[1] - S[1]) / math.dist(S, B0))
    A0 = (S[0] + HORN * u[0], S[1] + HORN * u[1])
    ROD = math.dist(A0, B0)
    TOGGLE_REST = True
    rows_tog = sweep()
    out['option_toggle_at_rest'] = summarise(rows_tog)
    with open(os.path.join(HERE, 'v2_linkage_results.json'), 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2)
    for tag, rows in [('as_built', rows_cad), ('toggle_option', rows_tog)]:
        with open(os.path.join(HERE, 'v2_linkage_sweep_%s.csv' % tag), 'w', newline='', encoding='utf-8') as f:
            keys = [k for k in rows[0] if not isinstance(rows[0][k], list)]
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader(); w.writerows(rows)
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != 'table'} for k, v in out.items()}, indent=1))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for rows, label, c in [(rows_cad, 'V2 as built (CAD)', '#1f6fb2'), (rows_tog, 'option: toggle at rest', '#c0504d')]:
        x = [r['lift_deg'] for r in rows]
        ax[0].plot(x, [r['tip_force_perp_N_stall'] for r in rows], color=c, label=label)
        ax[1].plot(x, [r['servo_torque_Nm_per_100N_down'] for r in rows], color=c, label=label)
    ax[0].axhline(7.4, color='gray', ls=':', lw=1)
    ax[0].text(22, 9, 'lift one side of a ~1.5 kg robot (~7.4 N)', fontsize=8, color='gray')
    ax[0].set_ylim(0, 200); ax[0].set_xlabel('lift angle (deg)'); ax[0].set_ylabel('tip force at servo stall (N)')
    ax[0].set_title('Lifting force, 3.92 N*m ideal stall')
    ax[1].axhline(STALL_NM, color='gray', ls=':', lw=1)
    ax[1].text(1, STALL_NM + 0.2, 'servo stall 3.92 N*m', fontsize=8, color='gray')
    ax[1].set_xlabel('lift angle (deg)'); ax[1].set_ylabel('servo torque to hold 100 N down (N*m)')
    ax[1].set_title('Back-drive: hit on the tip')
    for a in ax:
        a.grid(alpha=0.3); a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, 'v2_linkage.png'), dpi=140)


if __name__ == '__main__':
    main()
