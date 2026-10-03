"""Do V4's tilted hinged skirts stop a low wedge from getting under its flank and rear?

V4 sits still (wheels unpowered, weapon off). V2 or R3 charges straight into its right flank or its rear at full
throttle from 0.30-0.50 m, contact point randomised along the side; the attacker's lifter fires as in the duels.
Same runs with the skirts removed (V4's 4 mm tub-floor edge exposed) as the baseline.
"Gets under" = V4's wheel load drops below 25 % of its weight for > 0.25 s, or V4 rolls/pitches > 5 deg.
Run: ../../.venv/bin/python flank_test.py   (writes flank_test.json)
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
N = 24


def one(job):
    attacker, side, skirts, mu, seed = job
    sim.V4_SKIRTS = skirts
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.30, 0.50)
    if side == 'flank':      # attacker on +x facing -x, hitting V4's right side anywhere along it
        start_b = ((0.10 + gap, rng.uniform(-0.08, 0.08)), math.pi / 2)
    else:                    # attacker behind V4 facing +y
        start_b = ((rng.uniform(-0.06, 0.06), -0.12 - gap), 0.0)
    xml, metas = sim.world([('V4', 'A', (0, 0), 0.0), (attacker, 'B', start_b[0], start_b[1])], 2.4, mu,
                           (0.004, 1.0), dt=2e-5)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    A, B = sim.Robot(m, d, 'A', metas['A']), sim.Robot(m, d, 'B', metas['B'])
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    t, run, lifted, tilt = 0.0, 0.0, 0.0, 0.0
    while t < 2.0:
        A.wheel_cmd(0.0, 0.0)
        B.wheel_cmd(1.0, 1.0)
        dist = float(np.linalg.norm(A.pose()[0][:2] - B.pose()[0][:2]))
        if t >= B.cool_until and dist < 0.20:
            B.lift_until, B.cool_until = t + 0.6, t + 1.0
        B.servo_step(B.meta['lift_max'] if t < B.lift_until else 0.0, 1e-3)
        mujoco.mj_step(m, d, nstep=50)
        t += 1e-3
        frac = sim.wheel_load_fraction(m, d, A, floor)
        run = run + 1e-3 if frac < 0.25 else 0.0
        if run > 0.25:
            lifted += 1e-3
        up = A.pose()[2]
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, up[2])))))
    return job, {'lifted_s': lifted, 'max_tilt_deg': tilt, 'under': bool(lifted > 0 or tilt > 5.0)}


if __name__ == '__main__':
    jobs = [(a, side, sk, mu, seed) for a in ('V2', 'R3') for side in ('flank', 'rear') for sk in (True, False)
            for mu in (0.4, 0.8) for seed in range(N if side == 'flank' else N // 2)]
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
    out = {}
    for (a, side, sk, mu, seed), r in res:
        key = '%s_hits_V4_%s_%s' % (a, side, 'skirts' if sk else 'no_skirts')
        out.setdefault(key, []).append(r)
    summ = {k: {'n': len(v), 'p_gets_under': round(float(np.mean([r['under'] for r in v])), 2),
                'median_tilt_deg': round(float(np.median([r['max_tilt_deg'] for r in v])), 1),
                'median_lifted_s': round(float(np.median([r['lifted_s'] for r in v])), 2)} for k, v in sorted(out.items())}
    json.dump(summ, open(os.path.join(HERE, 'flank_test.json'), 'w'), indent=1)
    for k, v in summ.items():
        print(k, v)
