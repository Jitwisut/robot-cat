"""R4-steel set A in MuJoCo: does the 2WD robot rock, nose over, scrape its teeth or lift a wheel?

One robot alone on the floor. Mass, CG and part positions come from
print_revision/r4_steel/output/build_report.json; rotor inertia from print_revision/r3/calc/r3_calc.json;
drive motor ORTECH JB361 400 rpm (R3 nominal stall torque 0.1216 N m at 12 V, linear torque-speed).
Collision shapes are boxes/cylinders on the CAD outline (same style as sim.py). Contact parameters are the
sim.py defaults; tyre mu 0.6, UHMW skids mu 0.2, steel/PC 0.3.

Tests (12.6 V, rotor at its no-load speed unless stated):
  rest             2 s settle: which contacts carry the weight
  launch           full forward from rest, 1.5 s
  brake            full forward 1.5 s, then full reverse 1 s (hardest nose-down pitch)
  reverse_launch   full reverse from rest
  spin_rotor_on    spin in place, full stick, firmware yaw limit 12 rad/s, rotor spinning (gyro)
  spin_rotor_off   same with the rotor stopped
Outputs: pitch/roll extremes, which skid/wedgelet/body touched the floor, lowest tooth-tip height, minimum
wheel load fraction (both drive wheels), top speed. Writes r4_steel_2wd.json.
Run: .venv/bin/python robot_v4/mujoco_sim/r4_steel_2wd.py   (the venv with mujoco)
"""
import json
import math
import os

import mujoco
import numpy as np

from sim import CYL_X, box, f, ramp

HERE = os.path.dirname(os.path.abspath(__file__))
PR = os.path.join(HERE, '..', 'print_revision')
rep = json.load(open(os.path.join(PR, 'r4_steel', 'output', 'build_report.json')))
r3c = json.load(open(os.path.join(PR, 'r3', 'calc', 'r3_calc.json')))

M = rep['mass']['total_g'] / 1000
CG = np.array(rep['mass']['cg_mm']) / 1000
rot = r3c['1_rotor']
W_ROTOR = rot['12.6V_full']['rpm_no_load'] * 2 * math.pi / 60
I_ROTOR = 2 * rot['12.6V_full']['energy_J_no_load'] / W_ROTOR ** 2
M_ROTOR = rot['rotor_mass_g'] / 1000
M_WHEEL = 0.0399                              # 6061 hub 28.0 g + 2 x P52 O-ring 5.95 g (build_report wheel_hub)
M_WEDGE = 0.0256
V = 12.6
STALL = 0.1216 * V / 12                      # N m at the wheel shaft, JB361 nominal (R3 inputs)
W0 = 400 * V / 12 * 2 * math.pi / 60          # rad/s no-load
YAW_LIMIT = 12.0                               # rad/s firmware limiter
WHEEL_X, WHEEL_Y, WHEEL_R, WHEEL_HW = 0.0675, 0.018, 0.032, 0.008
AXLE = (0.095, 0.032)
TIP_R = 0.0295
DT = 5e-5


def model():
    # chassis = everything except wheels, rotor and wedgelets; its CG is solved so the whole robot's CG matches CAD
    m_ch = M - 2 * M_WHEEL - M_ROTOR - 2 * M_WEDGE
    parts = [(2 * M_WHEEL, np.array([0, WHEEL_Y, 0.032])), (M_ROTOR, np.array([0, AXLE[0], AXLE[1]])),
             (2 * M_WEDGE, np.array([0, 0.114, 0.011]))]
    cg_ch = (M * CG - sum(m * p for m, p in parts)) / m_ch
    ixx = m_ch / 12 * (0.235 ** 2 + 0.057 ** 2)
    iyy = m_ch / 12 * (0.166 ** 2 + 0.057 ** 2)
    izz = m_ch / 12 * (0.166 ** 2 + 0.235 ** 2)
    hard = 'friction="0.3 0.005 0.0001"'
    skid = 'friction="0.2 0.005 0.0001"'
    tyre = 'friction="0.6 0.005 0.0001"'
    b = ['<body name="r" pos="0 0 0.0005"><freejoint name="free"/>',
         '<inertial pos="%s" mass="%s" diaginertia="%s"/>' % (f(*cg_ch), f(m_ch), f(ixx, iyy, izz)),
         box('body', -0.083, 0.083, -0.110, 0.062, 0.004, 0.063, 'mass="0" ' + hard),
         box('railL', -0.058, -0.048, 0.062, 0.098, 0.006, 0.016, 'mass="0" ' + hard),
         box('railR', 0.048, 0.058, 0.062, 0.098, 0.006, 0.016, 'mass="0" ' + hard),
         box('beamL', -0.088, -0.037, 0.088, 0.098, 0.006, 0.016, 'mass="0" ' + hard),
         box('beamR', 0.037, 0.088, 0.088, 0.098, 0.006, 0.016, 'mass="0" ' + hard),
         box('uprL', -0.033, -0.027, 0.064, 0.112, 0.007, 0.057, 'mass="0" ' + hard),
         box('uprR', 0.027, 0.033, 0.064, 0.112, 0.007, 0.057, 'mass="0" ' + hard),
         box('nose_skid', -0.020, 0.020, 0.064, 0.074, 0.001, 0.004, 'mass="0" ' + skid),
         box('rear_skid', -0.020, 0.020, -0.108, -0.096, 0.001, 0.004, 'mass="0" ' + skid),
         # released R3 skirts: rigid, bottom edge 0.5 mm over the floor (sim.py 'fixed0.5')
         box('skirtL', -0.103, -0.091, -0.110, 0.097, 0.0005, 0.0165, 'mass="0" ' + hard),
         box('skirtR', 0.091, 0.103, -0.110, 0.097, 0.0005, 0.0165, 'mass="0" ' + hard),
         box('skirtB', -0.088, 0.088, -0.125, -0.113, 0.0005, 0.0165, 'mass="0" ' + hard)]
    for n, x in enumerate((-WHEEL_X, WHEEL_X)):
        b.append('<body name="w%d" pos="%s"><joint name="w%d_j" type="hinge" axis="1 0 0" damping="0.0005"/>'
                 '<geom name="w%d_g" type="cylinder" size="%s" %s mass="%s" %s/></body>' % (
                     n, f(x, WHEEL_Y, 0.032), n, n, f(WHEEL_R, WHEEL_HW), CYL_X, f(M_WHEEL), tyre))
    i_perp = M_ROTOR * (3 * 0.024 ** 2 + 0.052 ** 2) / 12
    b.append('<body name="rotor" pos="0 %s"><joint name="rotor_j" type="hinge" axis="1 0 0" damping="0"/>'
             '<inertial pos="0 0 0" mass="%s" diaginertia="%s"/>'
             '<geom name="tips" type="cylinder" size="%s 0.026" %s mass="0" contype="0" conaffinity="0"/></body>' % (
                 f(*AXLE), f(M_ROTOR), f(I_ROTOR, i_perp, i_perp), f(TIP_R), CYL_X))
    for side, x0, x1 in (('L', -0.088, -0.027), ('R', 0.027, 0.088)):
        b.append('<body name="wedge%s" pos="0 0.102 0.0145">' % side +
                 '<joint name="wedge%s_j" type="hinge" axis="1 0 0" range="-0.035 0.26" damping="0.002" limited="true"/>' % side +
                 ramp('wedge%s_g' % side, x0, x1, -0.002, 0.006, 0.026, -0.014, 0.002, 'mass="%s" %s' % (f(M_WEDGE), hard)) + '</body>')
    b.append('</body>')
    xml = """<mujoco model="r4_steel_2wd">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="%s" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default><geom solref="0.004 1" solimp="0.9 0.95 0.001" condim="3"/></default>
  <worldbody><geom name="floor" type="plane" size="5 5 0.1" friction="0.6 0.005 0.0001"/>%s</worldbody>
  <actuator><motor name="m0" joint="w0_j" gear="1"/><motor name="m1" joint="w1_j" gear="1"/><motor name="mr" joint="rotor_j" gear="1"/></actuator>
</mujoco>""" % (f(DT), '\n'.join(b))
    return mujoco.MjModel.from_xml_string(xml)


def run(test, yaw_limit=YAW_LIMIT):
    m = model()
    d = mujoco.MjData(m)
    gid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g): g for g in range(m.ngeom)}
    floor = gid['floor']
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'r')
    wdof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'w%d_j' % n)] for n in range(2)]
    rdof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'rotor_j')]
    rotor_on = test != 'spin_rotor_off'
    if rotor_on:
        d.qvel[rdof] = W_ROTOR
    mujoco.mj_forward(m, d)
    t_end = {'rest': 2.0, 'launch': 1.5, 'brake': 2.5, 'reverse_launch': 1.5, 'spin_rotor_on': 2.0, 'spin_rotor_off': 2.0}[test]
    stats = {'max_pitch_nose_up_deg': 0.0, 'max_pitch_nose_down_deg': 0.0, 'max_roll_deg': 0.0, 'min_tip_z_mm': 99.0,
             'touched': set(), 'top_speed_m_s': 0.0, 'max_yaw_rate': 0.0}
    load_acc, n_acc = {}, 0
    win, win_n, min_win = np.zeros(2), 0, [9.0, 9.0]     # per-wheel load, 50 ms windows
    F6 = np.zeros(6)
    weight = M * 9.81
    for step in range(int(t_end / DT)):
        t = step * DT
        if test == 'rest':
            cmd = (0.0, 0.0)
        elif test == 'launch':
            cmd = (1.0, 1.0)
        elif test == 'reverse_launch':
            cmd = (-1.0, -1.0)
        elif test == 'brake':
            cmd = (1.0, 1.0) if t < 1.5 else (-1.0, -1.0)
        else:
            R = d.xmat[body].reshape(3, 3)
            yaw_rate = float((R.T @ d.cvel[body][:3])[2]) if False else float(d.qvel[5])
            st = 1.0 if abs(yaw_rate) < yaw_limit else yaw_limit / abs(yaw_rate)
            cmd = (-st, st) if t > 0.3 else (0.0, 0.0)
        for k in range(2):
            w = -d.qvel[wdof[k]]
            tq = float(np.clip(STALL * (cmd[k] - w / W0), -STALL, STALL))
            d.ctrl[k] = -tq
        if rotor_on:      # hold the rotor at no-load speed (motor torque only replaces losses here)
            d.ctrl[2] = 0.05 * (W_ROTOR - d.qvel[rdof])
        mujoco.mj_step(m, d)
        if step % 20: continue
        R = d.xmat[body].reshape(3, 3)
        fwd, up = R[:, 1], R[:, 2]
        pitch = math.degrees(math.asin(np.clip(fwd[2], -1, 1)))
        roll = math.degrees(math.asin(np.clip(R[:, 0][2], -1, 1)))
        stats['max_pitch_nose_up_deg'] = max(stats['max_pitch_nose_up_deg'], pitch)
        stats['max_pitch_nose_down_deg'] = min(stats['max_pitch_nose_down_deg'], pitch)
        stats['max_roll_deg'] = max(stats['max_roll_deg'], abs(roll))
        axle = d.xpos[body] + R @ np.array([0, AXLE[0], AXLE[1]])
        stats['min_tip_z_mm'] = min(stats['min_tip_z_mm'], (axle[2] - TIP_R) * 1000)
        v = d.qvel[:3]
        stats['top_speed_m_s'] = max(stats['top_speed_m_s'], float(abs(np.dot(v, fwd))))
        stats['max_yaw_rate'] = max(stats['max_yaw_rate'], abs(float(d.qvel[5])))
        wl = np.zeros(2)
        for i in range(d.ncon):
            c = d.contact[i]
            other = c.geom2 if c.geom1 == floor else c.geom1 if c.geom2 == floor else None
            if other is None: continue
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, other)
            mujoco.mj_contactForce(m, d, i, F6)
            if name in ('w0_g', 'w1_g'):
                wl[int(name[1])] += F6[0]
            if F6[0] > 0.02 * weight:
                stats['touched'].add(name)
            if t > t_end - 0.5:
                load_acc[name] = load_acc.get(name, 0.0) + F6[0]
        win += wl; win_n += 1
        if win_n == 50:                                   # 50 samples x 20 steps x 5e-5 s = 50 ms
            if t > 0.3:
                for k in range(2):
                    min_win[k] = min(min_win[k], win[k] / win_n / weight)
            win[:], win_n = 0.0, 0
        if t > t_end - 0.5:
            n_acc += 1
    stats['touched'] = sorted(stats['touched'])
    stats['min_50ms_wheel_load_pct'] = {'left': round(100 * min_win[0], 1), 'right': round(100 * min_win[1], 1)}
    stats['last_0.5s_mean_load_pct'] = {k: round(100 * v / n_acc / weight, 1) for k, v in sorted(load_acc.items()) if v / n_acc / weight > 0.005}
    return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in stats.items()}


if __name__ == '__main__':
    out = {'mass_kg': M, 'cg_mm': rep['mass']['cg_mm'], 'rotor_I': I_ROTOR, 'rotor_rad_s': W_ROTOR, 'stall_Nm': STALL, 'w0': W0, 'tests': {}}
    for test in ('rest', 'launch', 'brake', 'reverse_launch', 'spin_rotor_on', 'spin_rotor_off'):
        out['tests'][test] = run(test)
        print(test, json.dumps(out['tests'][test]))
    # gyro: rotor angular momentum x yaw rate tips the robot about its fore-aft axis; find a yaw limit that keeps
    # both drive wheels loaded (half-track 67.5 mm)
    H = I_ROTOR * W_ROTOR
    out['gyro'] = {'H_Nms': H, 'tipping_yaw_rate_rad_s': M * 9.81 * WHEEL_X / H}
    for lim in (10.0, 8.0, 6.0):
        key = 'spin_rotor_on_yawlimit_%g' % lim
        out['tests'][key] = run('spin_rotor_on', lim)
        print(key, json.dumps(out['tests'][key]))
    json.dump(out, open(os.path.join(HERE, 'r4_steel_2wd.json'), 'w'), indent=1)
