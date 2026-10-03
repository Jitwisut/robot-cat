"""More bite for V4: faster drive and/or a belt reduction on the weapon. Scripted head-on charges with the
rotor at speed, against a charging opponent and a parked one. Run: ../../.venv/bin/python bite_test.py
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
VARIANTS = {'now_400rpm_1to1': (400, 1.0), 'drive620_1to1': (620, 1.0), 'drive400_1.5to1': (400, 1.5),
            'drive620_1.5to1': (620, 1.5)}


def one(args):
    var, opp, still, seed = args
    sim.V4_DRIVE_RPM, sim.ROTOR_RATIO = VARIANTS[var]
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    face = math.pi if not still or seed % 2 == 0 else math.pi / 2       # parked: half front-on, half side-on
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.03, 0.03), gap / 2), face + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True,
                       solref=(0.004, 1.0), b_still=still)
    return args, s['B']['max_com_z'], s['B']['final_inverted']


if __name__ == '__main__':
    jobs = [(v, opp, still, seed) for v in VARIANTS for opp in ('R3', 'V2') for still in (False, True) for seed in range(16)]
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
    out = {}
    for (v, opp, still, seed), z, inv in res:
        out.setdefault('%s|%s|%s' % (v, opp, 'parked' if still else 'charging'), []).append((z, inv))
    summ = {k: {'p_launched_over_10cm': round(float(np.mean([r[0] > 0.1 for r in x])), 2),
                'opp_max_com_z_mm_median': round(float(np.median([r[0] for r in x])) * 1000, 1),
                'p_inverted': round(float(np.mean([r[1] for r in x])), 2)} for k, x in sorted(out.items())}
    json.dump(summ, open(os.path.join(HERE, 'bite_test.json'), 'w'), indent=1)
    for k, v in summ.items():
        print(k, v)
