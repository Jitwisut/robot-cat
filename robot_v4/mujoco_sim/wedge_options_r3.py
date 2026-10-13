"""Low-wedge options on V4_PRINT_R3: wedgelet geometry variants and a big/slow rotor, mass- and balance-checked.

All runs use the R3 configuration with the weapon current clipped at 40 A (run_duels_r3 'r3_cap40').
Variants are applied by wrapping the configured V4 builder (sim.py itself is not edited):
  now            wedgelet plate 35.5 deg, 2.0 mm, tip y 128 (R3 CAD)
  w25_short      25 deg, 2.0 mm, same tip y 128 (plate starts lower, carriers reshaped)
  w25_long_t1.5  25 deg, 1.5 mm, same start (100, 20.5) -> tip y 142.9
  w20_long_t1.2  20 deg, 1.2 mm, same start -> tip y 154.9
  big_slow       rotor tip r 40 mm, axle z 42.5 mm, 1 tooth per disc, belt 1.5:1, same rotor mass, I x (40/29.5)^2
Each variant: static rear load (rest test as simulate.py), then scripted head-on charges vs R3/V2, charging and
parked, 64 seeds x 2 contact stiffnesses, same seeds for every variant (paired comparison).
Run: ../../.venv/bin/python wedge_options_r3.py [n_seeds] [workers]   (writes wedge_options_r3.json)
"""
import json
import math
import os
import re
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim  # noqa: E402
import validate  # noqa: E402
import run_duels_r3  # noqa: E402

PIN = (0.102, 0.0145)
PLATE_NOW = {'len': math.hypot(0.028, 0.020), 't': 0.002}
PLATE_G_NOW = 26.5836                      # CAD Wedgelet_L (steel)
CARRIERS_G = 1.6557 + 1.311                # CAD PETG carriers per side (the hinge block is fixed: chassis mass)


def plate(angle_deg, t, tip_y=None, start=(0.100, 0.0205)):
    """Underside start/tip (y, z) for a plate at angle_deg whose tip rests 0.5 mm above the floor."""
    a = math.radians(angle_deg)
    if tip_y is None:
        y0, z0 = start
        tip = (y0 + (z0 - 0.0005) / math.tan(a), 0.0005)
    else:
        tip = (tip_y, 0.0005)
        y0 = 0.100
        z0 = 0.0005 + (tip_y - y0) * math.tan(a)
    length = math.hypot(tip[0] - y0, tip[1] - z0)
    mass_g = PLATE_G_NOW * length * t / (PLATE_NOW['len'] * PLATE_NOW['t'])
    return {'start': (y0, z0), 'tip': tip, 't': t, 'plate_g': mass_g, 'angle_deg': angle_deg}


VARIANTS = {
    'now': {'wedge': plate(35.54, 0.002, tip_y=0.128)},
    'w25_short': {'wedge': plate(25.0, 0.002, tip_y=0.128)},
    'w25_long_t1.5': {'wedge': plate(25.0, 0.0015)},
    'w20_long_t1.2': {'wedge': plate(20.0, 0.0012)},
    'big_slow': {'wedge': plate(35.54, 0.002, tip_y=0.128), 'rotor': {'tip_r': 0.040, 'z': 0.0425, 'teeth': 1, 'ratio': 1.5,
                                                                      'I_scale': (0.040 / 0.0295) ** 2}},
    # same, with the inertia of a real 1-tooth r40 disc at equal mass (5.37 mm plate, design_v4.disc_props): x1.37
    'big_slow_realI': {'wedge': plate(35.54, 0.002, tip_y=0.128), 'rotor': {'tip_r': 0.040, 'z': 0.0425, 'teeth': 1,
                                                                            'ratio': 1.5, 'I_scale': 1.372}},
}
CURRENT = {'name': 'now'}
DEFAULTS = {'V4_TIP_R': sim.V4_TIP_R, 'ROTOR_RATIO': sim.ROTOR_RATIO}


def init(cfg='r3_cap40'):
    run_duels_r3.init(cfg)
    configured = sim.v4

    def variant_v4(prefix, *args):
        body, act, meta = configured(prefix, *args)
        v = VARIANTS[CURRENT['name']]
        w = v['wedge']
        for side, x0, x1 in (('L', -0.088, -0.027), ('R', 0.027, 0.088)):
            pat = r'<body name="%s_wedge%s" pos="[^"]*">.*?</body>' % (prefix, side)
            hard = 'friction="0.3 0.005 0.0001" ' + args[-1]
            new = ('<body name="%s_wedge%s" pos="%s">' % (prefix, side, sim.f(0, *PIN)) +
                   '<joint name="%s_wedge%s_j" type="hinge" axis="1 0 0" range="-0.035 0.244" damping="0.002" limited="true"/>'
                   % (prefix, side) +
                   sim.ramp('%s_wedge%s_g' % (prefix, side), x0, x1, w['start'][0] - PIN[0], w['start'][1] - PIN[1],
                            w['tip'][0] - PIN[0], w['tip'][1] - PIN[1], w['t'],
                            'mass="%s" %s' % (sim.f((w['plate_g'] + CARRIERS_G) / 1000), hard)) + '</body>')
            body, n = re.subn(pat, lambda m: new, body, flags=re.S)
            assert n == 1
        r = v.get('rotor')
        if r:
            body, n = re.subn(r'(<body name="%s_rotor" pos=")[^"]+' % prefix, lambda m: m.group(1) + sim.f(0, 0.095, r['z']), body)
            assert n == 1

            def scale_inertia(m):
                vals = [float(x) for x in m.group(2).split()]
                return m.group(1) + sim.f(vals[0] * r['I_scale'], vals[1] * r['I_scale'], vals[2] * r['I_scale'])
            body, n = re.subn(r'(<body name="%s_rotor"[^>]*>.*?<inertial pos="0 0 0" mass="[^"]+" diaginertia=")([^"]+)' % prefix,
                              scale_inertia, body, flags=re.S)
            assert n == 1
            if r['teeth'] == 1:
                body = re.sub(r'<geom name="%s_tooth[LR]1\d"[^>]*/>' % prefix, '', body)
        return body, act, meta
    sim.v4 = variant_v4
    sim.BUILDERS['V4'] = variant_v4


def set_variant(name):
    CURRENT['name'] = name
    sim.V4_TIP_R, sim.ROTOR_RATIO = DEFAULTS['V4_TIP_R'], DEFAULTS['ROTOR_RATIO']
    r = VARIANTS[name].get('rotor')
    if r:
        sim.V4_TIP_R, sim.ROTOR_RATIO = r['tip_r'], r['ratio']


def rest(name):
    """Static wheel loads and mass, as print_revision/simulate.py; plus tooth-floor gap during spin-up."""
    set_variant(name)
    sys.path.insert(0, run_duels_r3.PR)
    from simulate import wheel_loads
    m, d, r = validate.make('V4')
    loads = []

    def hold(t):
        r.wheel_cmd(0, 0)
        if t > 0.5:
            loads.append(wheel_loads(m, d))
    validate.run(m, d, 0.8, hold)
    mean = np.mean(loads, axis=0)
    mass = float(m.body_subtreemass[r.body])
    m2, d2, r2 = validate.make('V4')
    tooth = [i for i in range(m2.ngeom) if (sim.mujoco.mj_id2name(m2, sim.mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('A_tooth')]
    gap = [1.0]

    def sp(t):
        r2.rotor_step(validate.CTRL)
        r2.wheel_cmd(0, 0)
        if t > 0.2:
            gap[0] = min(gap[0], min(validate.geom_min_z(m2, d2, sim.mujoco.mj_id2name(m2, sim.mujoco.mjtObj.mjOBJ_GEOM, i))
                                     for i in tooth))
    validate.run(m2, d2, 2.5, sp)
    w = r2.rotor_rpm() * 2 * math.pi / 60
    I_r = m2.body_inertia[sim.mujoco.mj_name2id(m2, sim.mujoco.mjtObj.mjOBJ_BODY, 'A_rotor')][0]
    return {'mass_g': mass * 1000, 'rear_load_fraction': float(mean[2:].sum() / (mass * sim.G)),
            'wheel_loads_N': mean.round(3).tolist(), 'spinup_rotor_rpm': r2.rotor_rpm(),
            'rotor_energy_J': 0.5 * I_r * w ** 2, 'min_tooth_floor_gap_mm': gap[0] * 1000}


def headon(job):
    name, opp, still, sr, seed = job
    set_variant(name)
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    face = math.pi if not still or seed % 2 == 0 else math.pi / 2
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.03, 0.03), gap / 2), face + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True,
                       solref=(sr, 1.0), b_still=still)
    if 'error' in s:
        return job, None
    return job, (s['B']['max_com_z'], bool(s['B']['final_inverted']), s['B']['lifted_s'], s['A']['lifted_s'],
                 s['A']['max_com_z'])


def score(row):
    """Paired ranking of one run: launched > lifted > nothing; V4 lifted counts against."""
    z, inv, lift, v4lift, _ = row
    return (2 if z > 0.1 else 0) + (1 if lift > 0.25 else 0) + (1 if inv else 0) - (2 if v4lift > 0.25 else 0)


def duel(job):
    name, a, b, mu, sr, arena, seed = job
    set_variant(name)
    s = sim.engagement(a, b, seed, arena=arena, mu_tyre=mu, solref=(sr, 1.0), dt=2e-5, t_end=6.0)
    if 'error' in s:
        return job, 'diverged'
    return job, sim.classify(s)


def run_duels(names, n, workers):
    from run_duels import MU, SOLREF, ARENA
    jobs = [(v, 'V4', b, mu, sr, ar, seed) for v in names for b in ('V2', 'R3') for mu in MU for sr in SOLREF
            for ar in ARENA for seed in range(n)]
    with Pool(workers, initializer=init) as p:
        res = p.map(duel, jobs, chunksize=1)
    good = {'A_immobilises_B': 1, 'A_control': 1}
    bad = {'B_immobilises_A': 1, 'B_control': 1}
    cls = {job: c for job, c in res}
    out = {}
    for v in names:
        for b in ('V2', 'R3'):
            rows = [c for job, c in res if job[0] == v and job[2] == b]
            out.setdefault(v, {})['V4_vs_' + b] = {
                'n': len(rows), 'v4_better': sum(c in good for c in rows), 'opp_better': sum(c in bad for c in rows),
                'immobilised_opp': sum(c == 'A_immobilises_B' for c in rows), 'diverged': rows.count('diverged')}
            if v != 'now':
                up = dn = 0
                for job, c in res:
                    if job[0] != v or job[2] != b:
                        continue
                    base = cls[('now',) + job[1:]]
                    rank = lambda k: 1 if k in good else (-1 if k in bad else 0)
                    up += rank(c) > rank(base)
                    dn += rank(c) < rank(base)
                out[v]['V4_vs_' + b]['paired_vs_now'] = {'better': up, 'worse': dn}
    return out


if __name__ == '__main__' and len(sys.argv) > 1 and sys.argv[1] == 'duels':
    names = sys.argv[2].split(',')
    t0 = time.time()
    res = run_duels(names, int(sys.argv[3]) if len(sys.argv) > 3 else 24, 14)
    res['wall_s'] = round(time.time() - t0, 1)
    json.dump(res, open(os.path.join(HERE, 'wedge_options_duels_r3.json'), 'w'), indent=1)
    print(json.dumps(res, indent=1))
    sys.exit(0)

if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 64
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 14
    t0 = time.time()
    with Pool(workers, initializer=init) as p:
        statics = dict(zip(VARIANTS, p.map(rest, list(VARIANTS))))
        jobs = [(v, opp, still, sr, seed) for v in VARIANTS for opp in ('R3', 'V2') for still in (False, True)
                for sr in (0.004, 0.002) for seed in range(n)]
        res = p.map(headon, jobs, chunksize=2)
    by = {}
    raw = {}
    for job, row in res:
        if row is None:
            continue
        v, opp, still, sr, seed = job
        case = '%s|%s' % (opp, 'parked' if still else 'charging')
        by.setdefault((v, case), []).append(row)
        raw[job] = row
    summary = {}
    for (v, case), rows in sorted(by.items()):
        z = np.array([r[0] for r in rows])
        summary.setdefault(v, {})[case] = {
            'n': len(rows), 'p_launched_over_10cm': round(float(np.mean(z > 0.1)), 3),
            'p_opp_lifted_over_0.25s': round(float(np.mean([r[2] > 0.25 for r in rows])), 3),
            'p_inverted': round(float(np.mean([r[1] for r in rows])), 3),
            'opp_com_z_mm_median': round(float(np.median(z)) * 1000, 1),
            'p_v4_lifted': round(float(np.mean([r[3] > 0.25 for r in rows])), 3)}
    paired = {}
    for v in VARIANTS:
        if v == 'now':
            continue
        for case in ('R3|charging', 'R3|parked', 'V2|charging', 'V2|parked'):
            opp, mode = case.split('|')
            better = worse = 0
            for sr in (0.004, 0.002):
                for seed in range(n):
                    a = raw.get(('now', opp, mode == 'parked', sr, seed))
                    b = raw.get((v, opp, mode == 'parked', sr, seed))
                    if a is None or b is None:
                        continue
                    better += score(b) > score(a)
                    worse += score(b) < score(a)
            paired.setdefault(v, {})[case] = {'better': better, 'worse': worse}
    out = {'n_seeds': n, 'solref': [0.004, 0.002], 'config': 'r3_cap40', 'wall_s': round(time.time() - t0, 1),
           'variants': {k: {'wedge': v['wedge'], **({'rotor': v['rotor']} if 'rotor' in v else {})} for k, v in VARIANTS.items()},
           'static': statics, 'summary': summary, 'paired_vs_now': paired}
    json.dump(out, open(os.path.join(HERE, 'wedge_options_r3.json'), 'w'), indent=1, default=float)
    for k, v in statics.items():
        print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items()})
    for v, cases in summary.items():
        for c, s in cases.items():
            print('%-15s %-12s %s' % (v, c, s))
    print(json.dumps(paired))
    print('wall', out['wall_s'])
