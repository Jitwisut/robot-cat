"""Fixed (rigid) 35 deg skirts with a random as-built bottom-edge height: does R3 / V2 get under?

Height ~ U(lo, hi) mm per run stands in for build tolerance and floor flatness (the duels and flank tests
otherwise place every edge exactly). Run: ../../.venv/bin/python skirt_tolerance.py
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flank_test  # noqa: E402
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BANDS = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)]


def job(args):
    band, attacker, side, mu, seed = args
    h = np.random.default_rng(1000 + seed).uniform(*band)
    sim.V4_SKIRT_MODE = 'fixed%.3f' % h
    return args, h, flank_test.one((attacker, side, True, mu, seed))[1]


if __name__ == '__main__':
    jobs = [(band, a, side, mu, seed) for band in BANDS for a in ('R3', 'V2') for side in ('flank', 'rear')
            for mu in (0.4, 0.8) for seed in range(12)]
    with Pool(8) as p:
        res = p.map(job, jobs, chunksize=1)
    out = {}
    for (band, a, side, mu, seed), h, r in res:
        out.setdefault('%g-%g mm' % band, {}).setdefault('%s_%s' % (a, side), []).append(r['under'])
    summ = {b: {k: round(float(np.mean(v)), 2) for k, v in d.items()} for b, d in out.items()}
    json.dump(summ, open(os.path.join(HERE, 'skirt_tolerance.json'), 'w'), indent=1)
    for b, d in summ.items():
        print(b, d)
