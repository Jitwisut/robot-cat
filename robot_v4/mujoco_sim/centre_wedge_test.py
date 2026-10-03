"""Does the centre wedgelet stop a low wedge (R3) from slipping under V4's rotor head-on?

Scripted straight charge, rotor pre-spun, random rotor phase / gap / small lateral offset; 24 runs each,
with and without the centre wedgelet. Run: ../../.venv/bin/python centre_wedge_test.py
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


def one(args):
    centre, opp, sr, seed = args
    sim.V4_CENTRE_WEDGE = centre
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.03, 0.03), gap / 2), math.pi)]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True, solref=(sr, 1.0))
    return args, s['B']['max_com_z'], s['B']['lifted_s'], s['A']['lifted_s'], s['B']['final_inverted']


if __name__ == '__main__':
    jobs = [(c, opp, sr, seed) for c in (False, True) for opp in ('R3', 'V2') for sr in (0.004, 0.002) for seed in range(24)]
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
    out = {}
    for (c, opp, sr, seed), z, bl, al, inv in res:
        out.setdefault('%s_centre%s_solref%g' % (opp, 'ON' if c else 'OFF', sr), []).append((z, bl, al, inv))
    summ = {k: {'opp_max_com_z_mm_p25_p50_p75': [round(float(np.percentile([r[0] for r in v], q)) * 1000, 1) for q in (25, 50, 75)],
                'p_opp_launched_over_10cm': round(float(np.mean([r[0] > 0.1 for r in v])), 2),
                'p_opp_inverted': round(float(np.mean([r[3] for r in v])), 2),
                'opp_lifted_s_median': round(float(np.median([r[1] for r in v])), 2),
                'v4_lifted_s_median': round(float(np.median([r[2] for r in v])), 2)} for k, v in sorted(out.items())}
    json.dump(summ, open(os.path.join(HERE, 'centre_wedge_test.json'), 'w'), indent=1)
    for k, v in summ.items():
        print(k, v)
