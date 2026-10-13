"""Re-run the low-wedge tests (tooth_test / bite_test / attack_angle_test / centre_wedge_test) on V4_PRINT_R3.

Same scripted head-on charges and seeds as the originals, rotor pre-spun. Configs (one Pool each, because
print_revision/simulate.configure() must run once per process - see run_duels_r3.init):
  baseline   archived V4 (must reproduce tooth_test.json / attack_angle_test.json "now" numbers)
  r3         V4_PRINT_R3 mass/inertia/axles/electrical (12.6 V)
  r3_cap40   r3 with weapon current clipped at 40 A
Fix variants re-tried on r3: 1.5:1 belt reduction, tip 1.5 mm + skid 0.5 mm, centre wedgelet.
Run: ../../.venv/bin/python wedge_test_r3.py [workers]   (writes wedge_test_r3.json)
"""
import json
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim  # noqa: E402
import validate  # noqa: E402
import run_duels_r3  # noqa: E402

DEFAULTS = {'V4_TIP_R': sim.V4_TIP_R, 'V4_SKID_Z': sim.V4_SKID_Z, 'ROTOR_RATIO': sim.ROTOR_RATIO,
            'V4_CENTRE_WEDGE': sim.V4_CENTRE_WEDGE}
VARIANTS = {'now': {}, 'belt_1.5to1': {'ROTOR_RATIO': 1.5}, 'tip1.5_skid0.5': {'V4_TIP_R': 0.0305, 'V4_SKID_Z': 0.0005},
            'centre_wedge': {'V4_CENTRE_WEDGE': True}}
CONFIGS = {'baseline': ['now'], 'r3': list(VARIANTS), 'r3_cap40': ['now']}
FACING = {'front': math.pi, 'side': math.pi / 2, 'rear': 0.0}


def set_variant(var):
    for k, v in {**DEFAULTS, **VARIANTS[var]}.items():
        setattr(sim, k, v)


def probe():
    xml, _ = sim.world([('V4', 'A', (0, 0), 0.0)], 2.4, 0.6, (0.004, 1.0), dt=2e-5)
    m = sim.mujoco.MjModel.from_xml_string(xml)
    return {'v4_mass_kg': round(float(m.body_subtreemass[sim.mujoco.mj_name2id(m, sim.mujoco.mjtObj.mjOBJ_BODY, 'A')]), 5),
            'KE': sim.KE, 'V_PACK': sim.V_PACK, 'amp_cap': run_duels_r3.AMP_CAP, 'ratio': sim.ROTOR_RATIO}


def headon(job):
    """tooth_test / bite_test: V4 charges straight; opponent charges back (or parks: half front-on, half side-on)."""
    var, opp, still, seed = job
    set_variant(var)
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    face = math.pi if not still or seed % 2 == 0 else math.pi / 2
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.03, 0.03), gap / 2), face + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True,
                       solref=(0.004, 1.0), b_still=still)
    return ('headon', job, s['B']['max_com_z'], s['B']['final_inverted'], s['B']['lifted_s'], s['A']['lifted_s'],
            s['A']['peak_amps'], probe())


def angle(job):
    """attack_angle_test: opponent parked, V4 hits its front / side / rear."""
    var, opp, face, sr, seed = job
    set_variant(var)
    rng = np.random.default_rng(seed)
    gap = rng.uniform(0.40, 0.60)
    start = [((0.0, -gap / 2), 0.0), ((rng.uniform(-0.04, 0.04), gap / 2), FACING[face] + rng.normal(0, 0.1))]
    s = sim.engagement('V4', opp, seed, dt=2e-5, t_end=1.5, start=start, prespin=True, scripted=True, solref=(sr, 1.0),
                       b_still=True)
    return ('angle', job, s['B']['max_com_z'], s['B']['final_inverted'], s['B']['lifted_s'], s['A']['lifted_s'],
            s['A']['peak_amps'], probe())


def spinup_floor(var):
    """tooth_test.spinup_floor: spin up from rest, then drive off; count rotor-floor contact steps."""
    set_variant(var)
    m, d, r = validate.make('V4')
    rotor = sim.mujoco.mj_name2id(m, sim.mujoco.mjtObj.mjOBJ_BODY, 'A_rotor')
    floor = sim.mujoco.mj_name2id(m, sim.mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    rg = {g for g in range(m.ngeom) if m.geom_bodyid[g] == rotor}
    hits = {'n': 0}

    def spin(t):
        r.rotor_step(validate.CTRL)
        r.wheel_cmd(1 if 1.0 < t < 2.0 else 0, 1 if 1.0 < t < 2.0 else 0)
        for i in range(d.ncon):
            c = d.contact[i]
            if floor in (c.geom1, c.geom2) and (c.geom1 in rg or c.geom2 in rg):
                hits['n'] += 1
    validate.run(m, d, 2.5, spin)
    return ('spinup', var, {'rotor_floor_contact_steps': hits['n'], 'rpm': round(r.rotor_rpm())}, probe())


def dispatch(job):
    return {'headon': headon, 'angle': angle, 'spinup': spinup_floor}[job[0]](job[1])


def summary(rows):
    z = np.array([r[0] for r in rows])
    return {'n': len(rows), 'p_launched_over_10cm': round(float(np.mean(z > 0.1)), 2),
            'opp_max_com_z_mm_median': round(float(np.median(z)) * 1000, 1),
            'p_inverted': round(float(np.mean([r[1] for r in rows])), 2),
            'opp_lifted_s_mean': round(float(np.mean([r[2] for r in rows])), 2),
            'v4_lifted_s_mean': round(float(np.mean([r[3] for r in rows])), 2),
            'v4_peak_amps_median': round(float(np.median([r[4] for r in rows])), 1)}


if __name__ == '__main__':
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 14
    out = {'model': 'V4 collision primitives; r3 configs take mass/centroid/inertia, axles, part masses, lid roof '
                    'and electrical constants from the R3 build via print_revision/simulate.configure()',
           'note': 'centre_wedge adds its 25 g on top of the R3 CAD mass (configure replaces the chassis inertial)',
           'results': {}, 'probes': {}}
    t0 = time.time()
    for cfg, variants in CONFIGS.items():
        jobs = [('headon', (v, opp, still, seed)) for v in variants for opp in ('R3', 'V2') for still in (False, True)
                for seed in range(16)]
        jobs += [('angle', ('now', opp, face, sr, seed)) for opp in ('R3', 'V2') for face in FACING for sr in (0.004, 0.002)
                 for seed in range(12)]
        jobs += [('spinup', v) for v in variants if v in ('now', 'tip1.5_skid0.5')]
        with Pool(workers, initializer=run_duels_r3.init, initargs=(cfg,)) as p:
            res = p.map(dispatch, jobs, chunksize=1)
        groups, probes = {}, set()
        for r in res:
            probes.add(json.dumps(r[-1], sort_keys=True))
            if r[0] == 'spinup':
                groups['spinup|%s' % r[1]] = r[2]
                continue
            kind, job = r[0], r[1]
            if kind == 'headon':
                var, opp, still, _ = job
                key = 'headon|%s|%s|%s' % (var, opp, 'parked' if still else 'charging')
            else:
                var, opp, face, _, _ = job
                key = 'angle|%s|%s_%s' % (var, opp, face)
            groups.setdefault(key, []).append(r[2:7])
        out['results'][cfg] = {k: (v if k.startswith('spinup') else summary(v)) for k, v in sorted(groups.items())}
        out['probes'][cfg] = [json.loads(x) for x in probes]
        print(cfg, 'done', round(time.time() - t0), 's', flush=True)
    out['wall_s'] = round(time.time() - t0, 1)
    json.dump(out, open(os.path.join(HERE, 'wedge_test_r3.json'), 'w'), indent=1)
    for cfg, res in out['results'].items():
        for k, v in res.items():
            print(cfg, k, v)
    print(json.dumps(out['probes']))
