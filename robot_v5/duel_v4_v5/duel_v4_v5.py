"""V4 (R4-DIY, vertical beater) vs V5 (SawBlaze-style hammersaw): reduced-order duel screen.

Not a 3D contact simulation and not a driver model. Same method and match scoring as
robot_v4/three_way_sim (its match() is imported): per engagement scenario a Monte Carlo over the
uncertain inputs gives immobilise / control / neutral probabilities, then a match strings N
engagements together (immobilisation ends it, otherwise more control points wins).

Both robots as they would be built now, from the repo:
  V4 = R4-DIY D1 (robot_v4/print_revision/r4_diy/output/build_report.json): 1,980 g, 2WD + nose skid,
       steel tube frame, 2 mm PC skins, 57 J beater (SS400 discs, same inertia as v4_design.json).
  V5 = budget build (robot_v5/cad, design_v5.py, budget_cuts_v5.py): 1,818 g CAD .. 1,950 g (tub not
       detailed), 4WD, 70 J disc on a servo arm, hinged 12x4 forks on PETG blocks, TPU tub.
Front geometry follows the CADs: V4 rotor |x|<26 (2.5 mm), hinged wedgelets 27..88; V5 fork bars
(4 mm wide) at |x| 15..19 and 54..58, tongue skid |x|<13.6 at 1 mm, tub/blocks at 4 mm elsewhere.
Material and behaviour thresholds without data are ASSUMED ranges, and each is swept.
Axes: Z up, nose +Y (AGENTS.md). Run: python3 robot_v5/duel_v4_v5/duel_v4_v5.py
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'robot_v4', 'three_way_sim'))
import three_way as tw  # noqa: E402  (match model, outcome labels, turning model)

G, N, SIG = 9.81, tw.N, 0.3                         # hinged plates/forks resting on the floor: 0 mm, sigma 0.3
A_IMMOB, B_IMMOB, A_CTRL, B_CTRL, NEUTRAL = tw.A_IMMOB, tw.B_IMMOB, tw.A_CTRL, tw.B_CTRL, tw.NEUTRAL

r4 = json.load(open(os.path.join(ROOT, 'robot_v4', 'print_revision', 'r4_diy', 'output', 'build_report.json')))
v4d = json.load(open(os.path.join(ROOT, 'robot_v4', 'v4_design.json')))
v5d = json.load(open(os.path.join(ROOT, 'robot_v5', 'v5_design.json')))
cut = json.load(open(os.path.join(ROOT, 'robot_v5', 'budget_cuts_v5.json')))
cad5 = json.load(open(os.path.join(ROOT, 'robot_v5', 'cad', 'output', 'build_report.json')))

M4 = r4['mass']['total_g'] / 1000
E4 = v4d['chosen']['energy_J']
V4_DRIVE_LOAD = r4['support_2wd']['drive_wheel_load_pct'] / 100
V4_MOTOR_N = 2 * 3.99144375                            # build_r4_diy.py F_MOTOR x 2
M5_RANGE = (cad5['mass']['total_g'] / 1000, 1.95)
E5 = cut['result']['saw_energy_J']
V5_SPIN_S = cut['result']['t_spin_90pct_s']
V5_STRIKE_S = v5d['configs'][v5d['recommended']]['strike']['t_strike_s']
V5_MOTOR_N = 4 * 14.065 / 4
SURVIVE_ARM_J = cad5['load_path']['survivable_hit_energy_J (all SF >= 1)']
MU = (0.4, 0.8)

# ASSUMED behaviour/material ranges; BASE uses the middle, SWEEP moves one at a time to its ends
P = {
    'eta_clean': (0.2, 0.4),            # share of V4 rotor energy into V5's motion (three_way base)
    'eta_contested': (0.05, 0.2),       # a V5 fork bar already under V4's nose
    'p_inv_v5': (0.4, 0.8),             # launched V5 lands upside down (CG z 57 vs V4 27: above three_way's 0.3-0.7)
    'p_side_stuck_v5': (0.05, 0.25),    # lands on its side / arm and cannot recover
    'p_selfright_v5': (0.7, 0.95),      # inverted V5 rights itself (CAD margin 1.85x vs the clutch)
    'E_petg_block_J': (2.0, 8.0),       # delivered energy that breaks the PETG fork blocks -> forks lost
    'p_tub_internal_v5': (0.03, 0.10),  # a V4 hit on the TPU tub front reaches the internals (3 mm wall < 4.8 mm bite)
    'p_v5_ready': (0.5, 0.9),           # V5 arrives with the disc spun up and the arm parked up
    'eta_weapon_on_weapon': (0.03, 0.12),  # arm down, disc meets V4's beater: same-direction teeth, glancing
    'p_strike_lands_front': (0.25, 0.55),  # V4 still on the forks 0.29 s later, head-on (V4 is hitting back)
    'p_strike_lands_side': (0.5, 0.8),  # V5 strike when it has V4's flank/rear on its forks
    'p_strike_disables_v4': (0.08, 0.25),  # landed 70 J down-strike on V4's 2 mm PC lid reaches electronics
    'v5_front_edge_mm': (4.0, 4.0),     # CAD: between the fork bars V5's rigid front is the tub/blocks at z 4
}
# Design fixes the duel points at (not in the CAD yet): a full-width hinged steel wedge between the forks
# (rigid front 0.5..1.5 mm like V4's) and fork hinges welded to the steel tongue instead of PETG blocks.
FIXED_V5 = {'v5_front_edge_mm': (0.5, 1.5), 'E_petg_block_J': (1e9, 1e9)}


def u(r, lo, hi):
    return r.uniform(lo, hi, N)


def v4_push(r, mu):
    w = M4 * G * (1 + 0.03 * r.standard_normal(N))
    return np.minimum(V4_MOTOR_N, mu * w * V4_DRIVE_LOAD) - 0.2 * w * (1 - V4_DRIVE_LOAD)


def v5_push(r, mu, m5):
    w = m5 * G
    return np.minimum(V5_MOTOR_N, mu * w * 0.98) - 0.2 * w * 0.02


def v5_fork_bars(off):
    """x ranges of V5's fork bars in V4's frame for a lateral offset off (V5 centre at x=off)."""
    return [(off + a, off + b) for a, b in ((-58, -54), (-19, -15), (15, 19), (54, 58))]


def headon(r, prm, forks=True):
    off = u(r, -60, 60)
    m5 = u(r, *M5_RANGE)
    mu4, mu5 = u(r, *MU), u(r, *MU)
    # --- who gets under whose rigid nose (hinged plates/forks just ride up; the rigid edge behind decides)
    v4_wedge = r.normal(0, SIG, N)                   # V4 hinged wedgelets, x 27..88 both sides
    fork_tip = r.normal(0, SIG, N)
    v5_under_v4 = np.zeros(N, bool)                  # a V5 fork bar under V4's wedgelet -> reaches V4's frame (6 mm)
    fork_in_rotor = np.zeros(N, bool)
    if forks:
        for x0, x1 in v5_fork_bars(off):
            in_wedge = ((x1 > 27) & (x0 < 88)) | ((x1 > -88) & (x0 < -27))
            in_rotor = (x1 > -26) & (x0 < 26)
            v5_under_v4 |= in_wedge & (fork_tip < v4_wedge - 0.2)
            fork_in_rotor |= in_rotor
    # V4's wedgelets meet V5's tub/blocks (4 mm) between the fork bars almost everywhere -> V4 under V5's nose
    v5_edge = u(r, *prm['v5_front_edge_mm']) + r.normal(0, SIG, N)
    v4_under_v5 = v4_wedge < v5_edge - 0.2
    # --- V4 rotor hit on V5's front (it always meets something: forks, tongue, posts, or the disc)
    # V5 not 'ready' = arm down on the stops: its spinning disc leads (front y 133) and meets V4's beater.
    # Both spin front-teeth-up, so at the contact the teeth move the same way: a glancing weapon-on-weapon
    # exchange with little energy into either robot (eta_weapon_on_weapon), and V5 cannot strike from there.
    ready = r.uniform(0, 1, N) < u(r, *prm['p_v5_ready'])
    eta = np.where(v5_under_v4, u(r, *prm['eta_contested']), u(r, *prm['eta_clean']))
    eta = np.where(ready, eta, u(r, *prm['eta_weapon_on_weapon']))
    e_del = eta * E4
    h = e_del / (m5 * G)
    launched = h > tw.HIT['h_min']
    inv = launched & (r.uniform(0, 1, N) < u(r, *prm['p_inv_v5']))
    side = launched & ~inv & (r.uniform(0, 1, N) < u(r, *prm['p_side_stuck_v5']))
    stuck_inv = inv & (r.uniform(0, 1, N) > u(r, *prm['p_selfright_v5']))
    internal = r.uniform(0, 1, N) < u(r, *prm['p_tub_internal_v5'])
    v5_immob = stuck_inv | side | internal
    forks_lost = forks & (e_del > u(r, *prm['E_petg_block_J'])) & (fork_in_rotor | (r.uniform(0, 1, N) < 0.5))
    # --- V5 strike: needs ready + not launched this exchange + V4 still on the forks after 0.29 s
    lands = ready & ~launched & (r.uniform(0, 1, N) < u(r, *prm['p_strike_lands_front']) * (1.0 if forks else 0.5))
    v4_immob = lands & (r.uniform(0, 1, N) < u(r, *prm['p_strike_disables_v4'])) & ~v5_immob
    # --- control if nobody is immobilised: lift advantage first, then the shove
    p4, p5 = v4_push(r, mu4), v5_push(r, mu5, m5)
    only4 = v4_under_v5 & ~v5_under_v4
    only5 = v5_under_v4 & ~v4_under_v5
    a_ctrl = (launched & ~v5_immob) | only4 | (~only5 & (p4 > p5))
    b_ctrl = ~a_ctrl & (only5 | lands | (p5 >= p4))
    d = tw.dist(v5_immob, v4_immob, a_ctrl, b_ctrl)
    extra = {'p_V4_launches_V5': float(launched.mean()), 'mean_launch_h_m': float(h.mean()),
             'p_V5_forks_lost': float(forks_lost.mean()), 'p_V5_strike_lands': float(lands.mean()),
             'p_V4_under_V5_nose': float(v4_under_v5.mean()), 'p_V5_fork_under_V4': float(v5_under_v4.mean())}
    return d, extra


def v4_attacks_v5_side(r, prm):
    """V4's rotor into V5's flank/rear: outboard wheels and the tub side; flips easier (prm p_inv + 0.1)."""
    m5 = u(r, *M5_RANGE)
    h = u(r, *prm['eta_clean']) * E4 / (m5 * G)
    launched = h > tw.HIT['h_min']
    inv = launched & (r.uniform(0, 1, N) < np.minimum(u(r, *prm['p_inv_v5']) + 0.1, 1))
    side = launched & ~inv & (r.uniform(0, 1, N) < u(r, *prm['p_side_stuck_v5']))
    immob = (inv & (r.uniform(0, 1, N) > u(r, *prm['p_selfright_v5']))) | side
    return tw.dist(immob, None, launched & ~immob, None)


def v5_attacks_v4_side(r, prm, forks=True):
    """V5's forks into V4's flank/rear (rigid skirts 0.5..1.5 mm): under -> lift + a cleaner strike."""
    skirt = u(r, 0.5, 1.5) + r.normal(0, 0.7, N)
    under = forks & (r.normal(0, SIG, N) < skirt - 0.2)
    ready = r.uniform(0, 1, N) < u(r, *prm['p_v5_ready'])
    lands = ready & (r.uniform(0, 1, N) < u(r, *prm['p_strike_lands_side']) * np.where(under, 1.0, 0.5))
    immob = lands & (r.uniform(0, 1, N) < u(r, *prm['p_strike_disables_v4']))
    ctrl = under | lands
    return tw.dist(None, immob, None, ctrl)


def scenarios(prm, seed):
    r = np.random.default_rng(seed)
    ho, ex = headon(r, prm, True)
    ho_off, _ = headon(r, prm, False)
    af = v4_attacks_v5_side(r, prm)
    bf = v5_attacks_v4_side(r, prm, True)
    bf_off = v5_attacks_v4_side(r, prm, False)
    on = {'head_on': ho, 'A_flank': af, 'A_rear': af, 'B_flank': bf, 'B_rear': bf}
    off = {'head_on': ho_off, 'A_flank': af, 'A_rear': af, 'B_flank': bf_off, 'B_rear': bf_off}
    return on, off, ex


T90_V4 = tw.t90(M4, 0.135, 0.0, 0.253, 0.206, yaw_cap=12.0)       # 2WD + nose skid, firmware yaw cap
T90_V5 = tw.t90(1.9, 0.176, 0.090, 0.228, 0.192,
                yaw_cap=v5d['configs'][v5d['recommended']]['yaw_rate_limit_rad_s (gyro lifts a wheel)'])


def grid(prm, seed=7, n_matches=3000):
    on, off, ex = scenarios(prm, seed)
    res = {}
    for hs in (0.3, 0.5, 0.7):
        for ne in (4, 6, 10):
            res[f'headon{int(hs * 100)}_eng{ne}'] = tw.match(on, off, ex['p_V5_forks_lost'], T90_V4, T90_V5, hs, ne,
                                                           n_matches, seed=seed, b_weapon_breakable=True)
    return on, ex, res


def band(res, k):
    v = [g[k] for g in res.values()]
    return [round(min(v), 3), round(float(np.median(v)), 3), round(max(v), 3)]


def main():
    out = {'method': 'reduced-order Monte Carlo + three_way_sim match model, not 3D physics',
           'V4': {'build': 'R4-DIY D1', 'mass_kg': M4, 'energy_J': E4, 't90_s': T90_V4,
                  'push_N_mu0.6': r4['support_2wd']['push_N_mu0.6']},
           'V5': {'build': 'budget concept CAD', 'mass_kg_range': M5_RANGE, 'energy_J': E5, 'spin_up_s': V5_SPIN_S,
                  'strike_s': V5_STRIKE_S, 't90_s': T90_V5, 'arm_survives_J': SURVIVE_ARM_J},
           'assumed_ranges': P}
    on, ex, res = grid(P)
    out['base'] = {'scenarios_A_is_V4': on, 'headon_extras': ex,
                   'V4_win': band(res, 'A_win'), 'V5_win': band(res, 'B_win'), 'draw': band(res, 'draw'),
                   'V4_by_immobilise': band(res, 'A_by_immobilise'), 'V5_by_immobilise': band(res, 'B_by_immobilise'),
                   'grid': res}
    sweep = {}
    for k, (lo, hi) in P.items():
        for tag, val in (('low', (lo, lo)), ('high', (hi, hi))):
            prm = dict(P)
            prm[k] = val
            _, _, rr = grid(prm, n_matches=1500)
            sweep[f'{k}={tag}'] = {'V4_win_median': band(rr, 'A_win')[1], 'V5_win_median': band(rr, 'B_win')[1]}
    out['sweep_one_at_a_time'] = sweep
    flips = {k: v for k, v in sweep.items() if v['V5_win_median'] > v['V4_win_median']}
    out['assumptions_that_flip_the_winner'] = flips
    # V5's best realistic case: every ASSUMED at its V5-favourable end
    best5 = dict(P, p_inv_v5=(0.4, 0.4), p_side_stuck_v5=(0.05, 0.05), p_selfright_v5=(0.95, 0.95),
                 E_petg_block_J=(8.0, 8.0), p_tub_internal_v5=(0.03, 0.03), p_v5_ready=(0.9, 0.9),
                 p_strike_lands_front=(0.55, 0.55), p_strike_lands_side=(0.8, 0.8), p_strike_disables_v4=(0.25, 0.25))
    _, ex5, rr5 = grid(best5)
    out['V5_best_case'] = {'V4_win': band(rr5, 'A_win'), 'V5_win': band(rr5, 'B_win'), 'extras': ex5}
    _, exf, rrf = grid(dict(P, **FIXED_V5))
    out['V5_with_fixes'] = {'fixes': FIXED_V5, 'V4_win': band(rrf, 'A_win'), 'V5_win': band(rrf, 'B_win'),
                            'draw': band(rrf, 'draw'), 'extras': exf}
    _, exfb, rrfb = grid(dict(best5, **FIXED_V5))
    out['V5_with_fixes_best_case'] = {'V4_win': band(rrfb, 'A_win'), 'V5_win': band(rrfb, 'B_win')}
    print('V5 with fixes: V4', out['V5_with_fixes']['V4_win'], 'V5', out['V5_with_fixes']['V5_win'],
          '| best case V4', out['V5_with_fixes_best_case']['V4_win'], 'V5', out['V5_with_fixes_best_case']['V5_win'])
    print('   fixes extras', {k: round(v, 3) for k, v in exf.items()})
    json.dump(out, open(os.path.join(HERE, 'results.json'), 'w'), indent=1, ensure_ascii=False)
    print('t90 V4 %.2f s  V5 %.2f s' % (T90_V4, T90_V5))
    print('head-on (A=V4):', {k: round(v, 3) for k, v in on['head_on'].items()})
    print('extras:', {k: round(v, 3) for k, v in ex.items()})
    for k in ('A_flank', 'B_flank'):
        print(k, {kk: round(v, 3) for kk, v in on[k].items()})
    print('BASE V4 win', out['base']['V4_win'], 'V5 win', out['base']['V5_win'], 'draw', out['base']['draw'])
    print('     V4 by KO', out['base']['V4_by_immobilise'], 'V5 by KO', out['base']['V5_by_immobilise'])
    print('V5 best case: V4', out['V5_best_case']['V4_win'], 'V5', out['V5_best_case']['V5_win'])
    for k, v in sweep.items():
        print('  sweep', k, v)
    print('flips:', list(flips))


if __name__ == '__main__':
    main()
