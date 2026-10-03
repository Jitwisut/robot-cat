"""Lower the tooth tip (2.5 -> 1.5 mm off the floor, nose skid 1.0 -> 0.5 mm): bite under V2/R3 edges?
Also checks that spin-up does not drive the teeth into the floor. Run: ../../.venv/bin/python tooth_test.py
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402
import validate  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VARIANTS = {'now_tip2.5_skid1.0': (0.0295, 0.001), 'tip1.5_skid0.5': (0.0305, 0.0005)}


def one(args):
    var, opp, still, seed = args
    sim.V4_TIP_R, sim.V4_SKID_Z = VARIANTS[var]
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    face = math.pi if not still or seed % 2 == 0 else math.pi / 2
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.03, 0.03), gap / 2), face + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True,
                       solref=(0.004, 1.0), b_still=still)
    return args, s['B']['max_com_z'], s['B']['final_inverted']


def spinup_floor(var):
    sim.V4_TIP_R, sim.V4_SKID_Z = VARIANTS[var]
    m, d, r = validate.make('V4')
    rotor = sim.mujoco.mj_name2id(m, sim.mujoco.mjtObj.mjOBJ_BODY, 'A_rotor')
    rg = {g for g in range(m.ngeom) if m.geom_bodyid[g] == rotor}
    hits = {'n': 0}

    def spin(t):
        r.rotor_step(validate.CTRL)
        r.wheel_cmd(1 if 1.0 < t < 2.0 else 0, 1 if 1.0 < t < 2.0 else 0)    # spin up, then drive off
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in rg or c.geom2 in rg:
                hits['n'] += 1
    validate.run(m, d, 2.5, spin)
    return var, {'rotor_floor_contact_steps': hits['n'], 'rpm': round(r.rotor_rpm())}


if __name__ == '__main__':
    jobs = [(v, opp, still, seed) for v in VARIANTS for opp in ('R3', 'V2') for still in (False, True) for seed in range(16)]
    with Pool(8) as p:
        res = p.map(one, jobs, chunksize=1)
        floor = dict(p.map(spinup_floor, list(VARIANTS)))
    out = {}
    for (v, opp, still, seed), z, inv in res:
        out.setdefault('%s|%s|%s' % (v, opp, 'parked' if still else 'charging'), []).append((z, inv))
    summ = {k: {'p_launched_over_10cm': round(float(np.mean([r[0] > 0.1 for r in x])), 2),
                'opp_max_com_z_mm_median': round(float(np.median([r[0] for r in x])) * 1000, 1),
                'p_inverted': round(float(np.mean([r[1] for r in x])), 2)} for k, x in sorted(out.items())}
    summ['spinup_and_drive'] = floor
    json.dump(summ, open(os.path.join(HERE, 'tooth_test.json'), 'w'), indent=1)
    for k, v in summ.items():
        print(k, v)
