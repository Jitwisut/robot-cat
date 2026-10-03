"""Check each MuJoCo robot alone against the numbers from the earlier models before any duel.

Run: ../../.venv/bin/python validate.py   (writes validation.json)
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 2e-5
CTRL = 1e-3


def make(kind, xy=(0, 0), yaw=0.0, inverted=False, mu=0.6, arena=3.0):
    xml, metas = sim.world([(kind, 'A', xy, yaw)], arena, mu, dt=DT)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    r = sim.Robot(m, d, 'A', metas['A'])
    if inverted:
        top = {'V2': 0.085, 'R3': 0.042, 'V4': 0.064}[kind]
        adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'A_free')]
        d.qpos[adr:adr + 3] = [xy[0], xy[1], top + 0.003]
        d.qpos[adr + 3:adr + 7] = [0, 0, 1, 0]           # rolled 180 deg about y
    mujoco.mj_forward(m, d)
    return m, d, r


def run(m, d, seconds, fn=None):
    n = int(round(CTRL / DT))
    t = 0.0
    while t < seconds:
        if fn:
            fn(t)
        mujoco.mj_step(m, d, nstep=n)
        t += CTRL
    return t


def geom_min_z(m, d, name):
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
        return float(d.geom_xpos[g][2] - m.geom_size[g][0])
    R = d.geom_xmat[g].reshape(3, 3)
    s = m.geom_size[g]
    zs = [d.geom_xpos[g][2] + R[2] @ (np.array([sx, sy, sz]) * s)
          for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return float(min(zs))


EDGES = {'V2': ['A_spat_g', 'A_base', 'A_wedgeL'], 'R3': ['A_wingL', 'A_lift_g', 'A_guardL', 'A_rear'],
         'V4': ['A_wedgeL_g', 'A_wedgeR_g', 'A_skirtL_g', 'A_skirtR_g', 'A_skirtB_g', 'A_tub', 'A_skid', 'A_toothL00']}
TARGET = {'V2': {'mass': 1.087, 't90': 0.204}, 'R3': {'mass': 1.237, 't90': 0.446}, 'V4': {'mass': sim.V4_MASS, 't90': 0.36}}


def validate(kind):
    out = {}
    # rest: mass, CoG, edge heights after settling
    m, d, r = make(kind)
    run(m, d, 0.8, lambda t: r.wheel_cmd(0, 0))
    out['mass_kg'] = float(m.body_subtreemass[r.body])
    com = d.subtree_com[r.body] - d.xpos[r.body]
    R = d.xmat[r.body].reshape(3, 3)
    out['com_body_mm'] = [round(float(v) * 1000, 1) for v in R.T @ com]
    out['com_height_mm'] = round(float(d.subtree_com[r.body][2]) * 1000, 1)
    out['pitch_deg'] = round(math.degrees(math.asin(float(R[2, 1]))), 2)
    out['edge_min_z_mm'] = {n: round(geom_min_z(m, d, n) * 1000, 2) for n in EDGES[kind]}
    # top speed
    m, d, r = make(kind, xy=(0, -3.5), arena=8.0)
    run(m, d, 2.0, lambda t: r.wheel_cmd(1, 1))
    out['top_speed_m_s'] = round(float(np.linalg.norm(d.qvel[0:2])), 3)
    # push: drive into a wall and read the wall contact force
    m, d, r = make(kind, xy=(0, 1.5 - 0.20), arena=3.0)
    forces = []

    def push(t):
        r.wheel_cmd(1, 1)
        if t > 1.0:
            f6 = np.zeros(6)
            tot = 0.0
            for i in range(d.ncon):
                c = d.contact[i]
                names = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or '',
                         mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or '')
                if any(nm.startswith('wall') for nm in names):
                    mujoco.mj_contactForce(m, d, i, f6)
                    tot += abs(f6[0] * c.frame[1]) if False else abs(np.dot(c.frame[:3], [0, 1, 0]) * f6[0])
            forces.append(tot)
    run(m, d, 2.0, push)
    out['push_N_mu0.6'] = round(float(np.mean(forces)), 2)
    out['push_traction_limit_N'] = round(0.6 * out['mass_kg'] * sim.G, 2)
    # t90 in place
    m, d, r = make(kind)
    yaw = {'t90': None}

    def turn(t):
        r.wheel_cmd(-1, 1)
        R = d.xmat[r.body].reshape(3, 3)
        a = math.atan2(-R[0, 1], R[1, 1])
        if yaw['t90'] is None and abs(a) >= math.pi / 2:
            yaw['t90'] = t
    run(m, d, 1.5, turn)
    out['t90_s'] = yaw['t90']
    # inverted drive
    m, d, r = make(kind, inverted=True)
    p0 = d.xpos[r.body].copy()
    run(m, d, 0.3, lambda t: r.wheel_cmd(0, 0))
    p1 = d.xpos[r.body].copy()
    run(m, d, 1.5, lambda t: r.wheel_cmd(-1, -1))   # throttle flipped as the drivers do when inverted
    p2 = d.xpos[r.body].copy()
    out['inverted_drive_m_in_1.5s'] = round(float(np.linalg.norm((p2 - p1)[:2])), 3)
    out['target'] = TARGET[kind]
    return out


def v4_gyro():
    """Spin the rotor up, then turn in place at full command without the limiter: yaw rate at first wheel lift."""
    m, d, r = make('V4')
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    res = {'lift_yaw_rad_s': None}

    def spin(t):
        r.rotor_step(CTRL)
        r.wheel_cmd(0, 0)
    run(m, d, 2.5, spin)
    res['rotor_rpm'] = round(r.rotor_rpm())
    res['energy_J'] = round(0.5 * sim.ROTOR_I * (r.rotor_rpm() * 2 * math.pi / 60) ** 2, 1)
    # wheel lift = the robot rolls more than 3 deg (a side's wheels leave the floor); contact counts are too
    # noisy here because the rear wheels carry almost no load
    wheel_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'A_w%d_g' % n) for n in range(4)]
    peak = {'yaw': 0.0}

    def turn(t):
        r.rotor_step(CTRL)
        r.wheel_cmd(-1, 1)
        yaw = abs(d.cvel[r.body][2])
        peak['yaw'] = max(peak['yaw'], yaw)
        touching = set()
        for i in range(d.ncon):
            c = d.contact[i]
            for g in (c.geom1, c.geom2):
                if g in wheel_ids and floor in (c.geom1, c.geom2):
                    touching.add(g)
        R = d.xmat[r.body].reshape(3, 3)
        lost = abs(math.degrees(math.asin(float(R[2, 0])))) > 3.0
        res.setdefault('_run', 0.0)
        res['_run'] = res['_run'] + CTRL if lost else 0.0
        if res['lift_yaw_rad_s'] is None and res['_run'] >= 0.02:
            res['lift_yaw_rad_s'] = round(yaw, 2)
            res['lift_t_s'] = round(t, 3)
    run(m, d, 1.5, turn)
    res.pop('_run', None)
    res['peak_yaw_rad_s'] = round(peak['yaw'], 2)
    res['predicted_lift_yaw_rad_s'] = 14.7
    return res


if __name__ == '__main__':
    out = {k: validate(k) for k in ('V2', 'R3', 'V4')}
    out['V4_gyro'] = v4_gyro()
    json.dump(out, open(os.path.join(HERE, 'validation.json'), 'w'), indent=2)
    print(json.dumps(out, indent=1))
