"""Timestep convergence for head-on V4 rotor strikes (scripted straight charge, rotor pre-spun).

A single strike depends on the tooth phase at contact, so each setting runs 24 strikes with random rotor
phase and start gap, and compares the distributions.
Run: ../../.venv/bin/python convergence.py   (writes convergence.json)
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
N = 24


def one(args):
    opp, dt, sr, seed = args
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.02, 0.02), gap / 2), 3.14159265)]
    s = sim.engagement('V4', opp, seed, dt=dt, t_end=1.5, start=start, prespin=True, scripted=True, solref=(sr, 1.0))
    if 'error' in s:
        return None
    return (s['B']['max_com_z'], s['B']['final_inverted'], s['A']['max_com_z'])


if __name__ == '__main__':
    out = {}
    jobs = [(opp, dt, sr, seed) for opp in ('V2', 'R3') for dt in (4e-5, 2e-5, 1e-5) for sr in (0.004, 0.002)
            for seed in range(N)]
    t0 = time.time()
    with Pool(8) as p:
        res = p.map(one, jobs)
    for (opp, dt, sr, seed), r in zip(jobs, res):
        out.setdefault('%s_dt%g_solref%g' % (opp, dt, sr), []).append(r)
    summ = {}
    for k, rows in out.items():
        ok = [r for r in rows if r]
        h = np.array([r[0] for r in ok]) * 1000
        summ[k] = {'n': len(ok), 'diverged': len(rows) - len(ok),
                   'opp_max_com_z_mm_p25_p50_p75': [round(float(np.percentile(h, q)), 1) for q in (25, 50, 75)],
                   'p_opp_ends_inverted': round(float(np.mean([r[1] for r in ok])), 2),
                   'v4_max_com_z_mm_p50': round(float(np.median([r[2] for r in ok])) * 1000, 1)}
        print(k, summ[k], flush=True)
    summ['_wall_s_total'] = round(time.time() - t0, 1)
    json.dump(summ, open(os.path.join(HERE, 'convergence.json'), 'w'), indent=1)
    print('wall', summ['_wall_s_total'])
