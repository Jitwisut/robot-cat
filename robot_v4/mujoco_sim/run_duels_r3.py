"""Re-run the V4 duels with the V4_PRINT_R3 CAD mass/inertia/axles/electrical model.

Same grid and seeds as run_duels.py (V4_vs_V2, V4_vs_R3; the opponent "R3" is the V3 lifting wedge, not the
print revision). print_revision/simulate.configure() patches the V4 builder from the R3 build report; collision
shapes stay the V4 primitives (plus the R3 lid-roof box).

Variants:
  r3         R3 as configured (pack 12.6 V full charge, SURPASS 1300 KV)
  r3_11v1    R3 but pack forced to 11.1 V, to separate the voltage effect from the R3 changes
  baseline   unconfigured sim.py (archived V4) - spot-check against duels.json
  *_cap40    same, with weapon current clipped at 40 A (Skywalker 40A rating); sim.py itself has no limit
Run: ../../.venv/bin/python run_duels_r3.py r3 [n_per_cell] [workers]   (writes duels_r3.json / duels_r3_11v1.json)
"""
import hashlib
import itertools
import json
import os
import sys
import time
from multiprocessing import Pool

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PR = os.path.join(HERE, '..', 'print_revision')
R3_DIR = os.path.join(PR, 'r3')
sys.path.insert(0, HERE)
import sim  # noqa: E402
from run_duels import MU, SOLREF, ARENA, CLASSES  # noqa: E402

PAIRS = [('V4', 'V2'), ('V4', 'R3')]


def load_r3():
    report = json.load(open(os.path.join(R3_DIR, 'output_R3', 'build_report.json')))
    config = json.load(open(os.path.join(R3_DIR, 'inputs_R3.json')))
    sys.path.insert(0, PR)
    from inputs import geometry_payload
    if hashlib.sha256(json.dumps(geometry_payload(config), sort_keys=True).encode()).hexdigest() != report['input_sha256']:
        raise RuntimeError('R3 inputs changed after CAD build; run build.py first')
    return report, config


AMP_CAP = None


def capped_rotor_step(self, dt_ctrl, spin=True):
    """sim.Robot.rotor_step with the ESC current clipped at AMP_CAP."""
    target = 1.0 if spin else 0.0
    self.throttle = min(target, self.throttle + sim.RAMP_US_PER_S / 1000.0 * dt_ctrl) if target > self.throttle else target
    w = self.d.qvel[self.rotor_dof] * sim.ROTOR_RATIO
    amps = float(np.clip((self.throttle * sim.V_PACK - sim.KE * w) / sim.R_M, 0.0, AMP_CAP))
    self.d.ctrl[self.rotor_act] = sim.KE * max(amps - sim.I0, 0.0) * sim.ROTOR_RATIO * 0.92
    return amps


def init(variant):
    """Pool initializer: configure() wraps sim.v4, so it must run exactly once per (spawned) process."""
    global AMP_CAP
    if variant.endswith('_cap40'):
        AMP_CAP = 40.0
        sim.Robot.rotor_step = capped_rotor_step
        variant = variant[:-len('_cap40')]
    if variant == 'baseline':
        return
    sys.path.insert(0, PR)
    import simulate
    report, config = load_r3()
    simulate.configure(report, config)
    if variant == 'r3_11v1':
        sim.V_PACK = 11.1


def one(job):
    a, b, mu, sr, arena, seed = job
    xml, metas = sim.world([('V4', 'A', (0, 0), 0.0)], 2.4, mu, (sr, 1.0), dt=2e-5)
    m = mujoco.MjModel.from_xml_string(xml)
    probe = {'v4_mass_kg': float(m.body_subtreemass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'A')]),
             'KE': sim.KE, 'V_PACK': sim.V_PACK, 'n_roof': xml.count('_lid_roof'),
             'amp_cap': AMP_CAP}
    s = sim.engagement(a, b, seed, arena=arena, mu_tyre=mu, solref=(sr, 1.0), dt=2e-5, t_end=6.0)
    if 'error' in s:
        return job, 'diverged', None, probe
    keep = {k: {x: (float(v) if isinstance(v, (int, float, np.floating)) else v) for x, v in s[k].items()} for k in ('A', 'B')}
    return job, sim.classify(s), keep, probe


def summarise(cells):
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
        for side, keys in (('A_better', ('A_immobilises_B', 'A_control')), ('B_better', ('B_immobilises_A', 'B_control'))):
            v = [sum(s[k] for k in keys) for s in shares]
            summary[pair][side] = [round(min(v), 2), round(float(np.median(v)), 2), round(max(v), 2)]
    return summary


if __name__ == '__main__':
    variant = sys.argv[1] if len(sys.argv) > 1 else 'r3'
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 24
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 14
    seeds = range(n)
    jobs = [(a, b, mu, sr, ar, seed) for (a, b), mu, sr, ar in itertools.product(PAIRS, MU, SOLREF, ARENA) for seed in seeds]
    t0 = time.time()
    with Pool(workers, initializer=init, initargs=(variant,)) as p:
        res = p.map(one, jobs, chunksize=1)
    probes = {json.dumps(r[3], sort_keys=True) for r in res}
    cells, runs = {}, []
    for job, cls, keep, _ in res:
        a, b, mu, sr, ar, seed = job
        key = '%s_vs_%s|mu%g|solref%g|arena%g' % (a, b, mu, sr, ar)
        cells.setdefault(key, {c: 0 for c in CLASSES + ['diverged']})[cls] += 1
        runs.append({'pair': '%s_vs_%s' % (a, b), 'mu': mu, 'solref': sr, 'arena': ar, 'seed': seed, 'class': cls,
                     'stats': keep})
    out = {'variant': variant, 'n_per_cell': n, 'wall_s': round(time.time() - t0, 1),
           'worker_model_probe': [json.loads(x) for x in probes], 'cells': cells,
           'summary_min_median_max': summarise(cells), 'runs': runs}
    if not variant.startswith('baseline'):
        report, _ = load_r3()
        out['build_sha256'] = report['build_sha256']
        out['model'] = ('V4 collision primitives + R3 CAD mass/centroid/inertia, axles, wheel/wedgelet/skirt masses, '
                        'lid-roof box and electrical constants via print_revision/simulate.configure()')
    name = {'r3': 'duels_r3.json', 'r3_11v1': 'duels_r3_11v1.json', 'baseline': 'duels_baseline_check.json',
            'r3_cap40': 'duels_r3_cap40.json', 'baseline_cap40': 'duels_baseline_cap40.json'}[variant]
    json.dump(out, open(os.path.join(HERE, name), 'w'), indent=1)
    print(json.dumps(out['worker_model_probe']))
    print(json.dumps(out['summary_min_median_max'], indent=1))
    print('wall', out['wall_s'])
