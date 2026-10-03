"""Random-start engagements for each pair across friction, contact stiffness and arena size.

Each run: both robots start 0.5-1.0 m apart with noisy headings and chase each other with the same driver
for 6 s (V4's weapon spins up from rest with the firmware ramp). Outcome classes match three_way_sim.
Run: ../../.venv/bin/python run_duels.py [n_per_cell]   (writes duels.json)
"""
import itertools
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PAIRS = [('V4', 'V2'), ('V4', 'R3'), ('V2', 'R3')]
MU = (0.4, 0.8)
SOLREF = (0.004, 0.002)
ARENA = (2.4, 1.5)
CLASSES = ['A_immobilises_B', 'B_immobilises_A', 'A_control', 'B_control', 'neutral']


def one(job):
    a, b, mu, sr, arena, seed = job
    dt = 2e-5 if 'V4' in (a, b) else 1e-4
    s = sim.engagement(a, b, seed, arena=arena, mu_tyre=mu, solref=(sr, 1.0), dt=dt, t_end=6.0)
    if 'error' in s:
        return job, 'diverged', None
    keep = {k: {x: (float(v) if isinstance(v, (int, float, np.floating)) else v) for x, v in s[k].items()} for k in ('A', 'B')}
    return job, sim.classify(s), keep


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    jobs = [(a, b, mu, sr, ar, seed) for (a, b), mu, sr, ar in itertools.product(PAIRS, MU, SOLREF, ARENA)
            for seed in range(n)]
    t0 = time.time()
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
    cells = {}
    runs = []
    for job, cls, keep in res:
        a, b, mu, sr, ar, seed = job
        key = '%s_vs_%s|mu%g|solref%g|arena%g' % (a, b, mu, sr, ar)
        cells.setdefault(key, {c: 0 for c in CLASSES + ['diverged']})[cls] += 1
        runs.append({'pair': '%s_vs_%s' % (a, b), 'mu': mu, 'solref': sr, 'arena': ar, 'seed': seed, 'class': cls,
                     'stats': keep})
    summary = {}
    for (a, b) in PAIRS:
        pair = '%s_vs_%s' % (a, b)
        shares = []
        for key, cnt in cells.items():
            if key.startswith(pair + '|'):
                tot = sum(v for k, v in cnt.items() if k != 'diverged')
                shares.append({c: cnt[c] / tot for c in CLASSES})
        summary[pair] = {c: [round(min(s[c] for s in shares), 2), round(float(np.median([s[c] for s in shares])), 2),
                             round(max(s[c] for s in shares), 2)] for c in CLASSES}
        summary[pair]['A_better'] = [round(min(s['A_immobilises_B'] + s['A_control'] for s in shares), 2),
                                     round(float(np.median([s['A_immobilises_B'] + s['A_control'] for s in shares])), 2),
                                     round(max(s['A_immobilises_B'] + s['A_control'] for s in shares), 2)]
        summary[pair]['B_better'] = [round(min(s['B_immobilises_A'] + s['B_control'] for s in shares), 2),
                                     round(float(np.median([s['B_immobilises_A'] + s['B_control'] for s in shares])), 2),
                                     round(max(s['B_immobilises_A'] + s['B_control'] for s in shares), 2)]
    out = {'n_per_cell': n, 'wall_s': round(time.time() - t0, 1), 'cells': cells, 'summary_min_median_max': summary,
           'runs': runs}
    json.dump(out, open(os.path.join(HERE, 'duels.json'), 'w'), indent=1)
    print(json.dumps(summary, indent=1))
    print('wall', out['wall_s'])
