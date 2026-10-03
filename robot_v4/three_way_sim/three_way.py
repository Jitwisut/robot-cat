"""Three-way reduced-order duel screen: V2 (robot2) vs V3 (R3) vs V4 (vertical beater).

Not a 3D contact simulation. Same method as robot_v3/duel_sim (whose model is
imported and reused for V2 vs R3):
  1. per engagement scenario (head-on, A->B flank, B->A flank, A->B rear,
     B->A rear) a Monte Carlo over edge heights, friction, masses and energy
     transfer gives outcome probabilities: immobilise / control / neutral;
  2. a match model strings N engagements together, with scenario weights from
     each robot's turn agility, and scores immobilisation first, then control.
All robots are compared as built (V2 1.09 kg and R3 1.24 kg were drawn to the
old 1.5 kg target; V4 is 1.89 kg) on the new arena floor (plastic/tile).

Run: python3 three_way.py   (writes results.json, prints a summary)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'robot_v3', 'duel_sim'))
import duel_sim as ds  # noqa: E402

G = 9.81
N = 50000
SEED = 20261003
SIG = 0.7                                     # edge height noise (mm), same as duel_sim
MU_TILE = (0.4, 0.8)                          # rubber/TPU tyres on plastic or tile (ASSUMED)

v4 = json.load(open(os.path.join(ROOT, 'robot_v4', 'v4_design.json')))
E_V4 = v4['chosen']['energy_J']
M_V4 = v4['mass_total_g'] / 1000.0
V4_PUSH_MOTOR = v4['drive']['mu_0.4']['motor_push_N']

# ---------------------------------------------------------------- geometry (mm)
# V4 (from build_robot_v4.py): front = rotor zone |x|<26 at 2.5 mm, wedgelets 27..88 at 0.5..1.5;
# flank and rear = tub floor underside at 4 mm (floor runs under the side and rear walls).
V4_FRONT = [(-26, 26, 'rotor', 2.5, 2.5), (27, 88, 'wedge', 0.5, 1.5), (-88, -27, 'wedge', 0.5, 1.5)]
V4_SIDE_EDGE = 4.0
V4_HALF_WIDTH, V4_CG_Z = 88.0, 32.0
V4_CG_Y, V4_REAR_AXLE_Y, V4_TIP_Y = 5.0, -70.0, 128.0   # CoG y ASSUMED (rotor + weapon motor forward)
# V2 at impact (nose-down stance, duel_sim): spatula |x|<31 at 2.07, side wedges/base plate 0 mm
V2_FRONT = [(-31, 31, ds.R2['spatula_tip_down']), (31, 80, 0.0), (-80, -31, 0.0)]
R3_FRONT = [(-84, 84, None)]                  # None -> uniform tip 0.5..1.5
V2_LIFT_N = (44.0, 80.0)                      # solved linkage, exports/v2_linkage_2026-09-30
V2_LIFT_MM, R3_LIFT_MM = 44.4, 35.9
R3_LIFT_N_MIN = 67.0

# V4 hit assumptions; 'base' and a deliberately pessimistic set for V4 (main() runs both)
HIT = {'eta_clean': (0.2, 0.4),        # share of rotor energy into the opponent's motion, clean hit
       'eta_contested': (0.05, 0.2),   # opponent's wedge already under V4's wedgelets
       'p_inv': (0.3, 0.7),            # launched robot lands upside down
       'h_min': 0.3}                   # launch height (m) that counts as a real hit
HIT_SETS = {'base': dict(HIT),
            'pessimistic_v4': {'eta_clean': (0.1, 0.25), 'eta_contested': (0.01, 0.05), 'p_inv': (0.1, 0.3),
                               'h_min': 0.6}}

# outcome classes for robot A (first of the pair)
A_IMMOB, B_IMMOB, A_CTRL, B_CTRL, NEUTRAL = 'A_immobilises_B', 'B_immobilises_A', 'A_control', 'B_control', 'neutral'
OUTCOMES = [A_IMMOB, B_IMMOB, A_CTRL, B_CTRL, NEUTRAL]


def rng():
    return np.random.default_rng(SEED)


def u(r, lo, hi):
    return r.uniform(lo, hi, N)


# ---------------------------------------------------------------- shared physics
def traction_push(mass, mu, motor_limit):
    return np.minimum(motor_limit, mu * mass * G)


def v2_push(r, mu):
    m = ds.M2 * (1 + 0.08 * r.standard_normal(N))
    return traction_push(m, mu, 2 * ds.motor_force(0.0, 11.6, 0.75, 8.0))


def r3_push(r, mu):
    m = ds.M3 * (1 + 0.08 * r.standard_normal(N))
    return traction_push(m, mu, 4 * ds.motor_force(0.0, 11.6, 0.75, 4.0))


def v4_push(r, mu):
    m = M_V4 * (1 + 0.05 * r.standard_normal(N))
    return traction_push(m, mu, V4_PUSH_MOTOR)


def t90(mass, track, wheelbase, length, width, mu=0.6, yaw_cap=None):
    """Same turning model as duel_sim.main(); optional yaw-rate cap (rad/s)."""
    iz = mass * (length ** 2 + width ** 2) / 12
    alpha = max(mu * mass * G * (track - wheelbase) / 2, 1e-6) / iz
    t = math.sqrt(2 * (math.pi / 2) / alpha)
    if yaw_cap and alpha * t > yaw_cap:       # reaches the cap before 90 deg
        t_acc = yaw_cap / alpha
        t = t_acc + (math.pi / 2 - 0.5 * alpha * t_acc ** 2) / yaw_cap
    return t


def v4_flip_lift_needed_mm():
    """Edge lift that tips V4 over sideways (pivot on the far edge)."""
    tilt = math.atan2(V4_HALF_WIDTH, V4_CG_Z)
    return 2 * V4_HALF_WIDTH * math.sin(tilt), math.degrees(tilt)


# ---------------------------------------------------------------- V4 hits
def launch(r, mass_b, eta):
    """Launch height (m) of the struck robot if eta of the rotor energy becomes its motion."""
    return eta * E_V4 / (mass_b * G)


def v4_hit_outcome(r, mass_b, eta, p_inv):
    """Returns arrays: immobilises-B, control-A, launch height. A 'meaningful' launch is > 0.3 m."""
    h = launch(r, mass_b, eta)
    big = h > HIT['h_min']
    inverted = big & (r.uniform(0, 1, N) < p_inv)
    return inverted, ~inverted, h


def headon_v4(r, opp, weapon_on=True):
    """V4 (A) head-on vs V2 or R3 (B)."""
    off = u(r, -60, 60)
    mu_a, mu_b = u(r, *MU_TILE), u(r, *MU_TILE)
    v4_edges = {i: u(r, lo, hi) + r.normal(0, SIG, N) for i, (_x0, _x1, _k, lo, hi) in enumerate(V4_FRONT)}
    zones = V2_FRONT if opp == 'V2' else [(-84, 84, None)]
    rotor = np.zeros(N, bool)
    opp_under = np.zeros(N, bool)
    spatula_hit = np.zeros(N, bool)
    for zi, (ox0, ox1, oh) in enumerate(zones):
        oe = (u(r, 0.5, 1.5) if oh is None else oh) + r.normal(0, SIG, N)
        for i, (x0, x1, kind, _lo, _hi) in enumerate(V4_FRONT):
            ov = (off + ox1 > x0) & (off + ox0 < x1)
            if kind == 'rotor':
                rotor |= ov
                if opp == 'V2' and zi == 0:
                    spatula_hit |= ov
            else:
                opp_under |= ov & (oe < v4_edges[i] - 0.2)
    if opp == 'R3' and not weapon_on:
        opp_under[:] = False                  # R3 upside down: its plow is off the floor
    mass_b = (ds.M2 if opp == 'V2' else ds.M3) * (1 + 0.08 * r.standard_normal(N))
    eta = np.where(opp_under, u(r, *HIT['eta_contested']), u(r, *HIT['eta_clean']))
    p_inv = u(r, *HIT['p_inv'])
    inv, _ctl, h = v4_hit_outcome(r, mass_b, eta, p_inv)
    hit = rotor & (h > HIT['h_min'])
    a_immob = hit & inv
    # no meaningful launch: if B is under V4's wedgelets it controls - weight transfer onto its wedge wins
    # the shove (duel_sim rule) and a working lifter lifts V4's nose (needs ~7 N). V2's wedges are fixed,
    # so they still count with a broken lifter; otherwise it is a shoving match
    pa = v4_push(r, mu_a)
    pb = v2_push(r, mu_b) if opp == 'V2' else r3_push(r, mu_b)
    rest = ~hit
    b_lifts = rest & opp_under
    a_ctrl = (hit & ~inv) | (rest & ~b_lifts & (pa > pb))
    b_ctrl = b_lifts | (rest & ~b_lifts & (pb >= pa))
    # side effect: a rotor strike on V2's spatula can break its unprotected lifter linkage
    # (v2_linkage report: >51 N at rest back-drives the servo; strikes are kN) - ASSUMED 40-80 %
    disable_b = (spatula_hit & (eta > 0.1) & (r.uniform(0, 1, N) < u(r, 0.4, 0.8))) if opp == 'V2' else (hit & inv)
    return dist(a_immob, None, a_ctrl & ~a_immob, b_ctrl & ~a_immob), disable_b.mean(), h[rotor].mean()


def v4_attacks_side(r, opp):
    """V4 rotor into B's flank or rear: B cannot get under V4's front from the side."""
    mass_b = (ds.M2 if opp == 'V2' else ds.M3) * (1 + 0.08 * r.standard_normal(N))
    inv, _c, h = v4_hit_outcome(r, mass_b, u(r, *HIT['eta_clean']), u(r, *HIT['p_inv']))
    hit = h > HIT['h_min']
    return dist(hit & inv, None, hit & ~inv, None), float((hit & inv).mean())


def opp_attacks_v4_side(r, opp, weapon_on=True, edge=V4_SIDE_EDGE, sig=SIG):
    """V2 or R3 front into V4's flank or rear (4 mm floor edge as built; `edge`/`sig` for skirt studies)."""
    mu_a, mu_b = u(r, *MU_TILE), u(r, *MU_TILE)
    v4_edge = edge + r.normal(0, sig, N)
    if opp == 'V2':
        # nose-down stance at impact; side wedges at 0 mm, spatula 2.07 mm - take the lower part that meets V4
        oe = np.where(r.uniform(0, 1, N) < 0.6, 0.0, ds.R2['spatula_tip_down']) + r.normal(0, SIG, N)
    else:
        oe = u(r, 0.5, 1.5) + r.normal(0, SIG, N)
    under = oe < v4_edge - 0.2
    if opp == 'R3' and not weapon_on:
        under[:] = False                      # R3 upside down: its plow is off the floor
    # V2's side wedges are fixed: under V4's edge = control (wheels unloaded, weight transfer) even with a
    # broken lifter; R3 needs its lifting wedge, which is only usable right side up
    b_ctrl = under.copy()
    # not lifted: B pushes V4 sideways; V4 resists with sliding friction of its tyres
    pb = v2_push(r, mu_b) if opp == 'V2' else r3_push(r, mu_b)
    side_resist = mu_a * M_V4 * G
    b_ctrl |= ~b_ctrl & (pb > side_resist)
    return dist(None, None, None, b_ctrl), float(under.mean())


def dist(a_immob, b_immob, a_ctrl, b_ctrl):
    z = np.zeros(N, bool)
    a_immob = z if a_immob is None else a_immob
    b_immob = z if b_immob is None else b_immob
    a_ctrl = (z if a_ctrl is None else a_ctrl) & ~a_immob & ~b_immob
    b_ctrl = (z if b_ctrl is None else b_ctrl) & ~a_immob & ~b_immob & ~a_ctrl
    d = {A_IMMOB: a_immob.mean(), B_IMMOB: b_immob.mean(), A_CTRL: a_ctrl.mean(), B_CTRL: b_ctrl.mean()}
    d[NEUTRAL] = max(0.0, 1.0 - sum(d.values()))
    return {k: float(v) for k, v in d.items()}


def swap(d):
    return {A_IMMOB: d[B_IMMOB], B_IMMOB: d[A_IMMOB], A_CTRL: d[B_CTRL], B_CTRL: d[A_CTRL], NEUTRAL: d[NEUTRAL]}


# ---------------------------------------------------------------- V2 vs R3 (reused duel model)
def v2_r3(mu_range):
    """Scenario outcomes with A = V2, B = R3, from duel_sim.run_mc with the given friction range."""
    saved = dict(ds.PARAMS)
    ds.PARAMS['mu_r3'] = ('uniform', mu_range[0], mu_range[1], 'tyre-floor friction R3')
    ds.PARAMS['mu_r2'] = ('uniform', mu_range[0], mu_range[1], 'tyre-floor friction robot2')
    ds.PARAMS['r2_lift_N'] = ('uniform', V2_LIFT_N[0], V2_LIFT_N[1], 'robot2 spatula force, solved linkage')
    ds.RNG = np.random.default_rng(20260925)
    out = ds.run_mc()[0]
    ds.PARAMS.clear()
    ds.PARAMS.update(saved)
    ho = out['head_on']['p_r3_controls']
    sc = {
        'head_on': {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: 1 - ho, B_CTRL: ho, NEUTRAL: 0},
        'A_flank': {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: out['robot2_hits_r3_flank']['p_robot2_lifts_r3_side'],
                    B_CTRL: 0, NEUTRAL: 1 - out['robot2_hits_r3_flank']['p_robot2_lifts_r3_side']},
        'B_flank': {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: 0, B_CTRL: out['r3_hits_robot2_flank']['p_r3_lifts_side'],
                    NEUTRAL: 1 - out['r3_hits_robot2_flank']['p_r3_lifts_side']},
        'A_rear': {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: out['robot2_hits_r3_rear']['p_robot2_catches_r3_rear_tyre'],
                   B_CTRL: 0, NEUTRAL: 1 - out['robot2_hits_r3_rear']['p_robot2_catches_r3_rear_tyre']},
        'B_rear': {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: 0, B_CTRL: out['r3_hits_robot2_rear']['p_r3_under_50_50'],
                   NEUTRAL: 1 - out['r3_hits_robot2_rear']['p_r3_under_50_50']},
    }
    skirt = out['robot2_hits_r3_flank_with_1mm_skirts']['p_robot2_under']
    sc_skirt = dict(sc)
    sc_skirt['A_flank'] = {A_IMMOB: 0, B_IMMOB: 0, A_CTRL: skirt, B_CTRL: 0, NEUTRAL: 1 - skirt}
    return sc, sc_skirt, out


def v4_vs(opp):
    """Scenario outcomes with A = V4, B = opp; also the 'B weapon off' variants for the match model."""
    r = rng()
    ho, p_dis, h_mean = headon_v4(r, opp, True)
    ho_off, _, _ = headon_v4(r, opp, False)
    af, p_af_inv = v4_attacks_side(r, opp)
    ar, _ = v4_attacks_side(r, opp)
    bf, p_bf_under = opp_attacks_v4_side(r, opp, True)
    bf_off, _ = opp_attacks_v4_side(r, opp, False)
    br, _ = opp_attacks_v4_side(r, opp, True)
    br_off, _ = opp_attacks_v4_side(r, opp, False)
    on = {'head_on': ho, 'A_flank': af, 'B_flank': bf, 'A_rear': ar, 'B_rear': br}
    off = {'head_on': ho_off, 'A_flank': af, 'B_flank': bf_off, 'A_rear': ar, 'B_rear': br_off}
    return on, off, {'p_B_weapon_disabled_per_headon': float(p_dis), 'mean_launch_h_m_headon': float(h_mean),
                     'p_B_under_V4_side_edge': p_bf_under, 'p_V4_flank_hit_inverts_B': p_af_inv}


# ---------------------------------------------------------------- match model
def match(sc_on, sc_off, p_disable_headon, t_a, t_b, headon_share, n_eng, n_matches=20000, seed=1,
          b_weapon_breakable=False, b_inverts_disable=False):
    """String n_eng engagements. Side/rear access split by agility (1/t90); 70 % flank, 30 % rear.
    Immobilisation ends the match; otherwise more control points wins, equal = draw."""
    r = np.random.default_rng(seed)
    ag_a = (1 / t_a) / (1 / t_a + 1 / t_b)
    side = 1 - headon_share
    weights = {'head_on': headon_share, 'A_flank': side * ag_a * 0.7, 'A_rear': side * ag_a * 0.3,
               'B_flank': side * (1 - ag_a) * 0.7, 'B_rear': side * (1 - ag_a) * 0.3}
    names = list(weights)
    w = np.array([weights[k] for k in names])
    res = {'A_win': 0, 'B_win': 0, 'draw': 0, 'A_by_immobilise': 0, 'B_by_immobilise': 0}
    for _ in range(n_matches):
        score_a = score_b = 0
        b_weapon = True
        done = False
        for _e in range(n_eng):
            k = names[r.choice(len(names), p=w)]
            d = (sc_on if b_weapon else sc_off)[k]
            o = OUTCOMES[r.choice(5, p=np.array([d[x] for x in OUTCOMES]))]
            if o == A_IMMOB:
                if b_inverts_disable:          # R3 drives inverted: only its weapon is lost
                    b_weapon = False
                    score_a += 1
                    continue
                res['A_win'] += 1; res['A_by_immobilise'] += 1; done = True; break
            if o == B_IMMOB:
                res['B_win'] += 1; res['B_by_immobilise'] += 1; done = True; break
            if o == A_CTRL:
                score_a += 1
            elif o == B_CTRL:
                score_b += 1
            if b_weapon_breakable and k == 'head_on' and r.uniform() < p_disable_headon:
                b_weapon = False
        if not done:
            res['A_win' if score_a > score_b else 'B_win' if score_b > score_a else 'draw'] += 1
    return {k: v / n_matches for k, v in res.items()}


def main():
    out = {'method': 'reduced-order Monte Carlo + match model, not 3D physics', 'floor_mu': MU_TILE,
           'v4_energy_J': E_V4, 'masses_kg': {'V2': ds.M2, 'R3': ds.M3, 'V4': M_V4}}

    # regression check: the reused model must reproduce robot_v3/duel_sim/results.json at its own inputs
    ref = json.load(open(os.path.join(ROOT, 'robot_v3', 'duel_sim', 'results.json')))
    ds.RNG = np.random.default_rng(20260925)
    old = ds.run_mc()[0]
    out['regression'] = {
        'head_on_r3_controls': [old['head_on']['p_r3_controls'], ref['head_on']['p_r3_controls']],
        'v2_under_r3_flank': [old['robot2_hits_r3_flank']['p_robot2_under_at_impact'],
                              ref['robot2_hits_r3_flank']['p_robot2_under_at_impact']],
    }

    # agility (same turning model for all; V4 capped by its firmware yaw limiter while the weapon spins)
    t_v2 = t90(ds.M2, 0.1416, 0.0, 0.202, 0.160)
    t_r3 = t90(ds.M3, ds.R3['track'], ds.R3['wheelbase'], 0.197, 0.168)
    t_v4 = t90(M_V4, 0.140, 0.080, 0.238, 0.176, yaw_cap=12.0)
    out['t90_s'] = {'V2': t_v2, 'R3': t_r3, 'V4': t_v4}
    lift_needed, tilt = v4_flip_lift_needed_mm()
    out['v4_flip'] = {'edge_lift_needed_mm': lift_needed, 'tilt_deg': tilt,
                      'V2_max_lift_mm': V2_LIFT_MM, 'R3_max_lift_mm': R3_LIFT_MM,
                      'can_V2_flip_V4': V2_LIFT_MM >= lift_needed, 'can_R3_flip_V4': R3_LIFT_MM >= lift_needed}

    sc_23, sc_23_skirt, raw23 = v2_r3(MU_TILE)
    HIT.update(HIT_SETS['base'])
    sc_42_on, sc_42_off, ex42 = v4_vs('V2')
    sc_43_on, sc_43_off, ex43 = v4_vs('R3')
    HIT.update(HIT_SETS['pessimistic_v4'])
    p42_on, p42_off, pex42 = v4_vs('V2')
    p43_on, p43_off, pex43 = v4_vs('R3')
    HIT.update(HIT_SETS['base'])
    out['hit_assumption_sets'] = HIT_SETS
    out['scenarios'] = {'V2_vs_R3': sc_23, 'V2_vs_R3_skirted': sc_23_skirt,
                        'V4_vs_V2': sc_42_on, 'V4_vs_V2_lifter_broken': sc_42_off,
                        'V4_vs_R3': sc_43_on, 'V4_vs_R3_lifter_lost': sc_43_off,
                        'PESSIMISTIC_V4_vs_V2': p42_on, 'PESSIMISTIC_V4_vs_R3': p43_on}
    out['extras'] = {'V4_vs_V2': ex42, 'V4_vs_R3': ex43, 'PESSIMISTIC_V4_vs_V2': pex42, 'PESSIMISTIC_V4_vs_R3': pex43,
                     'V2_vs_R3_tile_headon_r3_controls': raw23['head_on']['p_r3_controls']}

    grid = {}
    for hs in (0.3, 0.5, 0.7):
        for ne in (4, 6, 10):
            key = f'headon{int(hs * 100)}_eng{ne}'
            grid[key] = {
                'V2_vs_R3': match(sc_23, sc_23, 0, t_v2, t_r3, hs, ne, 4000),
                'V2_vs_R3_skirted': match(sc_23_skirt, sc_23_skirt, 0, t_v2, t_r3, hs, ne, 4000),
                'V4_vs_V2': match(sc_42_on, sc_42_off, ex42['p_B_weapon_disabled_per_headon'], t_v4, t_v2, hs, ne,
                                  4000, b_weapon_breakable=True),
                'V4_vs_R3': match(sc_43_on, sc_43_off, 0, t_v4, t_r3, hs, ne, 4000, b_inverts_disable=True),
                'PESSIMISTIC_V4_vs_V2': match(p42_on, p42_off, pex42['p_B_weapon_disabled_per_headon'], t_v4, t_v2,
                                              hs, ne, 4000, b_weapon_breakable=True),
                'PESSIMISTIC_V4_vs_R3': match(p43_on, p43_off, 0, t_v4, t_r3, hs, ne, 4000, b_inverts_disable=True),
            }
    out['match_grid'] = grid

    # skirt study for V4's flank/rear: fixed 1 mm skirt, and a hinged skirt resting on the floor
    sk = {}
    for name, edge, sig in [('as_built_4mm', 4.0, SIG), ('fixed_skirt_1mm', 1.0, SIG), ('hinged_skirt_on_floor', 0.0, 0.3)]:
        sk[name] = {opp: opp_attacks_v4_side(rng(), opp, True, edge, sig)[1] for opp in ('V2', 'R3')}
    out['v4_side_skirt_study_p_opponent_under'] = sk

    def band(pair, k='A_win'):
        vals = [grid[g][pair][k] for g in grid]
        return [min(vals), float(np.median(vals)), max(vals)]
    out['summary_A_win_min_median_max'] = {p: band(p) for p in ('V2_vs_R3', 'V2_vs_R3_skirted', 'V4_vs_V2', 'V4_vs_R3', 'PESSIMISTIC_V4_vs_V2', 'PESSIMISTIC_V4_vs_R3')}
    out['summary_B_win_min_median_max'] = {p: band(p, 'B_win') for p in ('V2_vs_R3', 'V2_vs_R3_skirted', 'V4_vs_V2', 'V4_vs_R3', 'PESSIMISTIC_V4_vs_V2', 'PESSIMISTIC_V4_vs_R3')}
    json.dump(out, open(os.path.join(HERE, 'results.json'), 'w'), indent=2, default=float)

    print('regression', out['regression'])
    print('t90', {k: round(v, 3) for k, v in out['t90_s'].items()})
    print('v4 flip', out['v4_flip'])
    for k, v in out['scenarios'].items():
        print(k, {s: {o[:6]: round(p, 2) for o, p in d.items()} for s, d in v.items()})
    print('extras', json.dumps(out['extras'], indent=1))
    print('skirt', out['v4_side_skirt_study_p_opponent_under'])
    print('A win (min, median, max):', json.dumps(out['summary_A_win_min_median_max']))
    print('B win (min, median, max):', json.dumps(out['summary_B_win_min_median_max']))


if __name__ == '__main__':
    main()
