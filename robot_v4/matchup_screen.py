"""V4 matchup screening against opponent archetypes (reduced-order, Monte Carlo).

Not a 3D simulation and not a win-rate predictor. For each archetype it
reports the few quantities that decide the first exchange:
  * whose front edge is lower at contact (who gets under whom),
  * how hard V4's beater hits when it connects (launch speed/height),
  * pushing force vs the opponent,
  * for spinner-vs-spinner, the stored-energy ratio.
Opponent numbers for V2/R3 come from robot_v3/duel_sim; generic archetype
ranges are ASSUMED and declared below.
Run: python3 matchup_screen.py   (writes matchup_results.json)
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RNG = np.random.default_rng(20261003)
N = 50000
G = 9.81
SIG = 0.7                                   # edge height noise, mm (same as duel_sim)

design = json.load(open(os.path.join(HERE, 'v4_design.json')))
E_V4 = design['chosen']['energy_J']
M_V4 = design['mass_total_g'] / 1000.0

# V4 front, from the CAD (x across the nose, mm): (x0, x1, kind, edge height lo, hi)
# Wedgelets run under the uprights and side walls, so everything outside the rotor is a 0.5-1.5 mm edge.
V4_ZONES = [(-26, 26, 'rotor', 2.5, 2.5),
            (27, 88, 'wedgelet', 0.5, 1.5), (-88, -27, 'wedgelet', 0.5, 1.5)]
V4_PUSH = (6.2, 12.4)
AIM = 60.0                                   # opponent centre offset across V4's nose, +/- mm (ASSUMED)

# name: (mass kg, [(x0, x1, edge lo, hi)] about its own centre, push N, weapon J, kind)
OPP = {
    'V2 robot2 (wedge+lifter)':  (1.09, [(-31, 31, 2.07, 2.07), (31, 80, 0.0, 0.0), (-80, -31, 0.0, 0.0)],
                                  (8.5, 8.5), (0, 0), 'wedge'),
    'R3 (lifting wedge 4WD)':    (1.24, [(-84, 84, 0.5, 1.5)], (9.6, 9.6), (0, 0), 'wedge'),
    'Generic low wedge':         (1.8, [(-80, 80, 0.3, 3.0)], (6.0, 14.0), (0, 0), 'wedge'),
    'Horizontal bar spinner':    (1.8, [(-80, 80, 3.0, 8.0)], (4.0, 9.0), (40, 150), 'hspin'),
    'Vertical spinner':          (1.8, [(-30, 30, 1.0, 4.0), (30, 80, 0.5, 3.0), (-80, -30, 0.5, 3.0)],
                                  (5.0, 10.0), (30, 120), 'vspin'),
    'Control/pusher (no weapon)': (2.0, [(-80, 80, 1.0, 5.0)], (10.0, 16.0), (0, 0), 'pusher'),
}


def u(lo, hi):
    return RNG.uniform(lo, hi, N)


def main():
    out = {'v4_energy_J': E_V4, 'v4_mass_kg': M_V4, 'n': N, 'aim_offset_mm': AIM, 'matchups': {}}
    v4_push = u(*V4_PUSH)
    eta = u(0.2, 0.4)                       # share of rotor energy that becomes opponent motion
    v4_edge = {i: u(lo, hi) + RNG.normal(0, SIG, N) for i, (_a, _b, _k, lo, hi) in enumerate(V4_ZONES)}
    for name, (m, zones, push, energy, kind) in OPP.items():
        off = u(-AIM, AIM)
        v4_under = np.zeros(N, bool)
        opp_under = np.zeros(N, bool)
        rotor = np.zeros(N, bool)
        for (ox0, ox1, olo, ohi) in zones:
            oe = u(olo, ohi) + RNG.normal(0, SIG, N)
            for i, (x0, x1, kind_v4, _lo, _hi) in enumerate(V4_ZONES):
                overlap = (off + ox1 > x0) & (off + ox0 < x1)
                if kind_v4 == 'rotor':
                    rotor |= overlap          # the tooth meets this part whether it is above or below 2.5 mm
                    continue
                if kind_v4 == 'wedgelet':
                    v4_under |= overlap & (v4_edge[i] < oe - 0.2)
                opp_under |= overlap & (oe < v4_edge[i] - 0.2)
        launch_v = np.sqrt(2 * eta * E_V4 / m)
        r = {
            'p_rotor_contact': float(rotor.mean()),
            'p_v4_wedgelet_under_opponent': float(v4_under.mean()),
            'p_opponent_under_v4_somewhere': float(opp_under.mean()),
            'p_clean_hit (rotor contact, opponent not under)': float((rotor & ~opp_under).mean()),
            'p_bad (opponent under, no rotor contact)': float((opp_under & ~rotor).mean()),
            'launch_v_m_s_median': float(np.median(launch_v)),
            'launch_height_m_median': float(np.median(launch_v ** 2 / (2 * G))),
            'p_v4_wins_push': float((v4_push > u(*push)).mean()),
        }
        if kind in ('hspin', 'vspin'):
            e_opp = u(*energy)
            r['p_v4_energy_higher'] = float((E_V4 > e_opp).mean())
            r['opponent_energy_J_range'] = list(energy)
        out['matchups'][name] = r
    json.dump(out, open(os.path.join(HERE, 'matchup_results.json'), 'w'), indent=2)
    for k, v in out['matchups'].items():
        print(k, {a.split(' ')[0]: round(b, 2) if isinstance(b, float) else b for a, b in v.items()})


if __name__ == '__main__':
    main()
