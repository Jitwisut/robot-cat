"""Compare V4 skirt variants: flank/rear attack test + V4's own push force and rest wheel load.

Run: ../../.venv/bin/python skirt_variants.py   (writes skirt_variants.json)
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
MODES = ['hinged', 'limit5', 'spring', 'fixed1.0', 'fixed0.5']
N = 16


def attack(job):
    mode, attacker, side, mu, seed = job
    sim.V4_SKIRT_MODE = mode
    return job, flank_test.one((attacker, side, True, mu, seed))[1]


def own(mode):
    import validate
    sim.V4_SKIRT_MODE = mode
    v = validate.validate('V4')
    m, d, r = validate.make('V4')
    validate.run(m, d, 0.8, lambda t: r.wheel_cmd(0, 0))
    floor = sim.mujoco.mj_name2id(m, sim.mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    return mode, {'push_N': v['push_N_mu0.6'], 't90_s': v['t90_s'], 'top_speed': v['top_speed_m_s'],
                  'rest_wheel_load_frac': round(sim.wheel_load_fraction(m, d, r, floor), 3),
                  'skirt_min_z_mm': {k: v['edge_min_z_mm'][k] for k in ('A_skirtR_g', 'A_skirtB_g')}}


if __name__ == '__main__':
    jobs = [(mode, a, side, mu, seed) for mode in MODES for a in ('R3', 'V2') for side in ('flank', 'rear')
            for mu in (0.4, 0.8) for seed in range(N if side == 'flank' else N // 2)]
    with Pool(8) as p:
        res = p.map(attack, jobs, chunksize=1)
        owns = dict(p.map(own, MODES))
    out = {}
    for (mode, a, side, mu, seed), r in res:
        out.setdefault(mode, {}).setdefault('%s_%s' % (a, side), []).append(r)
    summ = {}
    for mode in MODES:
        summ[mode] = {k: {'p_gets_under': round(float(np.mean([r['under'] for r in v])), 2),
                          'median_tilt_deg': round(float(np.median([r['max_tilt_deg'] for r in v])), 1)}
                      for k, v in out[mode].items()}
        summ[mode]['own'] = owns[mode]
    json.dump(summ, open(os.path.join(HERE, 'skirt_variants.json'), 'w'), indent=1)
    for mode, v in summ.items():
        print(mode, {k: x['p_gets_under'] for k, x in v.items() if k != 'own'}, v['own'])
