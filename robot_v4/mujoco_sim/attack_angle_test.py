"""Where should V4 hit R3 and V2? Scripted charge with the rotor at speed into the opponent's front, side
or rear (opponent stationary). 24 runs per case. Run: ../../.venv/bin/python attack_angle_test.py
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FACING = {'front': math.pi, 'side': math.pi / 2, 'rear': 0.0}


def one(args):
    opp, face, sr, seed = args
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.04, 0.04), gap / 2), FACING[face] + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True, solref=(sr, 1.0),
                       b_still=True)
    return args, s['B']['max_com_z'], s['B']['final_inverted'], s['B']['lifted_s']


if __name__ == '__main__':
    jobs = [(opp, face, sr, seed) for opp in ('R3', 'V2') for face in FACING for sr in (0.004, 0.002) for seed in range(12)]
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
    out = {}
    for (opp, face, sr, seed), z, inv, lift in res:
        out.setdefault('%s_%s' % (opp, face), []).append((z, inv, lift))
    summ = {k: {'opp_max_com_z_mm_median': round(float(np.median([r[0] for r in v])) * 1000, 1),
                'p_launched_over_10cm': round(float(np.mean([r[0] > 0.1 for r in v])), 2),
                'p_ends_inverted': round(float(np.mean([r[1] for r in v])), 2)} for k, v in sorted(out.items())}
    json.dump(summ, open(os.path.join(HERE, 'attack_angle_test.json'), 'w'), indent=1)
    for k, v in summ.items():
        print(k, v)
