"""MuJoCo 3D duel model for V2 (robot2), V3 (R3) and V4 (vertical beater).

Collision shapes are primitives (boxes, cylinders, rotated plates) built from the
numbers already used in robot_v3/duel_sim (V2, R3) and robot_v4/build_robot_v4.py
(V4). This is rigid-body physics with MuJoCo's soft contacts: launch heights and
energy transfer depend on the contact parameters (solref/solimp/friction), which
are swept, not known.

Robot frame: x right, y forward (nose), z up; metres, kilograms, radians.
"""
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
V4D = json.load(open(os.path.join(HERE, '..', 'v4_design.json')))

ROTOR_I = V4D['chosen']['rotor_I_kgm2']
ROTOR_M = V4D['chosen']['rotor_mass_g'] / 1000.0
V4_MASS = V4D['mass_total_g'] / 1000.0
KE, R_M, I0, V_PACK = 60.0 / (2 * math.pi * 1250.0), 0.050, 1.5, 11.1      # D3536 model (design_v4.py)
RAMP_US_PER_S = 1600.0                                                     # firmware weapon ramp
YAW_LIMIT = 12.0                                                           # rad/s, firmware limiter
V4_CHASSIS_CG_Y = -0.0042                                                  # gives whole-robot CoG y +8.7 mm
SERVO_RATE = math.radians(60) / 0.16                                       # Repeat 40 kg class: 60 deg in 0.16 s
V4_SKIRTS = True                                                           # False = baseline without skirts (flank test)
# skirt variants (flank_test / skirt_variants.py): 'hinged' (25 deg free outward), 'limit5' (5 deg), 'spring'
# (torsion spring pressing it down), 'fixedX' (rigid, bottom edge X mm above the floor, e.g. 'fixed0.5')
V4_SKIRT_MODE = 'fixed0.5'                                                  # chosen 4 Oct (skirt_variants/tolerance)
V4_DRIVE_RPM = 400                                                         # JGA25-370 variant (400 or 620 rpm)
ROTOR_RATIO = 1.0
V4_TIP_R = 0.0295                                                          # tooth tip radius (axle at 32 mm -> 2.5 mm off the floor)
V4_SKID_Z = 0.001                                                          # nose skid underside height                                                          # motor rpm / rotor rpm (belt reduction)
V4_CENTRE_WEDGE = False                                                    # hinged wedgelet between the discs: tested 4 Oct, no gain vs R3 -> off
G = 9.81


# ------------------------------------------------------------------ MJCF helpers
def f(*v):
    return ' '.join('%.6g' % x for x in v)


def box(name, x0, x1, y0, y1, z0, z1, extra=''):
    return '<geom name="%s" type="box" pos="%s" size="%s" %s/>' % (
        name, f((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), f(abs(x1 - x0) / 2, abs(y1 - y0) / 2, abs(z1 - z0) / 2),
        extra)


def ramp(name, x0, x1, ya, za, yb, zb, t, extra=''):
    """Plate whose UNDERSIDE runs from (ya, za) to (yb, zb) in the y-z plane, thickness t, spanning x0..x1.
    Give the points with yb > ya so the thickness goes to the upper side."""
    dy, dz = yb - ya, zb - za
    length = math.hypot(dy, dz)
    a = math.atan2(dz, dy)
    ny, nz = -math.sin(a), math.cos(a)                      # normal pointing to the upper side
    cy, cz = (ya + yb) / 2 + ny * t / 2, (za + zb) / 2 + nz * t / 2
    q = (math.cos(a / 2), math.sin(a / 2), 0, 0)
    return '<geom name="%s" type="box" pos="%s" quat="%s" size="%s" %s/>' % (
        name, f((x0 + x1) / 2, cy, cz), f(*q), f(abs(x1 - x0) / 2, length / 2, t / 2), extra)


CYL_X = 'quat="0.7071068 0 0.7071068 0"'                    # cylinder axis along x


def wheel(prefix, name, x, y, z, r, hw, mass, tyre):
    return ('<body name="%s_%s" pos="%s"><joint name="%s_%s_j" type="hinge" axis="1 0 0" damping="0.0005"/>'
            '<geom name="%s_%s_g" type="cylinder" size="%s" %s mass="%s" %s/></body>') % (
        prefix, name, f(x, y, z), prefix, name, prefix, name, f(r, hw), CYL_X, f(mass), tyre)


def quat_yaw(yaw):
    return (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))


# ------------------------------------------------------------------ robots
# Each builder returns (body_xml, actuator_xml, meta). Contact filtering bits are injected by `world()`.
def v4(prefix, pos, yaw, mu_tyre, col):
    tyre = 'friction="%s 0.005 0.0001" %s' % (f(mu_tyre), col)
    hard = 'friction="0.3 0.005 0.0001" %s' % col
    m_wheel, m_wedge, m_skirt_side, m_skirt_rear = 0.045, 0.030, 0.0106, 0.0084
    m_chassis = V4_MASS - 4 * m_wheel - ROTOR_M - 2 * m_wedge - 2 * m_skirt_side - m_skirt_rear - (0.025 if V4_CENTRE_WEDGE else 0)
    ixx = m_chassis / 12 * (0.238 ** 2 + 0.056 ** 2)
    iyy = m_chassis / 12 * (0.176 ** 2 + 0.056 ** 2)
    izz = m_chassis / 12 * (0.176 ** 2 + 0.238 ** 2)
    p = prefix
    b = ['<body name="%s" pos="%s" quat="%s"><freejoint name="%s_free"/>' % (p, f(*pos), f(*quat_yaw(yaw)), p),
         # chassis CoG chosen so the whole robot's CoG lands at y +8.7 mm, z 30 mm: the part-by-part estimate
         # from the CAD layout (rotor, weapon motor, uprights and wedgelets all forward) - only 1.3 mm behind
         # the front axle (y +10), so the rear wheels carry almost nothing
         '<inertial pos="0 %s 0.030" mass="%s" diaginertia="%s"/>' % (f(V4_CHASSIS_CG_Y), f(m_chassis), f(ixx, iyy, izz)),
         box(p + '_tub', -0.088, 0.088, -0.110, 0.064, 0.004, 0.060, 'mass="0" ' + hard),
         box(p + '_wallL', -0.088, -0.080, 0.064, 0.097, 0.007, 0.058, 'mass="0" ' + hard),
         box(p + '_wallR', 0.080, 0.088, 0.064, 0.097, 0.007, 0.058, 'mass="0" ' + hard),
         box(p + '_uprL', -0.033, -0.027, 0.064, 0.112, 0.017, 0.057, 'mass="0" ' + hard),
         box(p + '_uprR', 0.027, 0.033, 0.064, 0.112, 0.017, 0.057, 'mass="0" ' + hard),
         box(p + '_sideL', -0.050, -0.034, 0.064, 0.096, 0.007, 0.050, 'mass="0" ' + hard),
         box(p + '_sideR', 0.034, 0.050, 0.064, 0.096, 0.007, 0.050, 'mass="0" ' + hard),
         # UHMW nose skid 1 mm off the floor (build_robot_v4.py): catches the nose before the teeth reach the floor
         box(p + '_skid', -0.020, 0.020, 0.064, 0.074, V4_SKID_Z, 0.004, 'mass="0" friction="0.15 0.005 0.0001" ' + col)]
    for n, (x, y) in enumerate([(-0.070, 0.010), (0.070, 0.010), (-0.070, -0.070), (0.070, -0.070)]):
        b.append(wheel(p, 'w%d' % n, x, y, 0.032, 0.032, 0.008, m_wheel, tyre))
    # rotor: two toothed discs + hub, inertia from design_v4.py
    i_perp = ROTOR_M * (3 * 0.024 ** 2 + 0.052 ** 2) / 12
    rg = ['<body name="%s_rotor" pos="0 0.095 0.032"><joint name="%s_rotor_j" type="hinge" axis="1 0 0" damping="0"/>' % (p, p),
          '<inertial pos="0 0 0" mass="%s" diaginertia="%s"/>' % (f(ROTOR_M), f(ROTOR_I, i_perp, i_perp)),
          '<geom name="%s_hub" type="cylinder" size="0.015 0.020" %s mass="0" %s/>' % (p, CYL_X, hard)]
    for side, x in (('L', -0.023), ('R', 0.023)):
        rg.append('<geom name="%s_disc%s" type="cylinder" pos="%s" size="0.023 0.003" %s mass="0" %s/>' % (
            p, side, f(x, 0, 0), CYL_X, hard))
        # each 60 deg tooth = two narrow boxes at +-15 deg, so no box corner sweeps beyond r 29.9 mm
        # (one wide box reached r 31.8 mm and scraped the floor, which the real 29.5 mm tooth does not)
        for k, a0 in enumerate((0.0, math.pi)):
            for j, da in enumerate((-0.2618, 0.2618)):
                a = a0 + da
                half = (V4_TIP_R - 0.023) / 2
                c = 0.023 + half
                q = (math.cos(a / 2), math.sin(a / 2), 0, 0)
                rg.append('<geom name="%s_tooth%s%d%d" type="box" pos="%s" quat="%s" size="0.003 %s 0.005" mass="0" %s/>' % (
                    p, side, k, j, f(x, c * math.cos(a), c * math.sin(a)), f(*q), f(half), hard))
    rg.append('</body>')
    b += rg
    # centre wedgelet between the discs (|x| < 19.3 mm, under the hub): pin (y 0.0765, z 0.0052), 1.5 mm steel
    # plate whose underside runs from (0.079, 0.0055) to the tip (0.124, 0.0005); it rests on the floor and
    # makes a low wedge that slips under the rotor climb into the teeth
    if V4_CENTRE_WEDGE:
        b.append('<body name="%s_wedgeC" pos="0 0.0765 0.0052">' % p +
                 '<joint name="%s_wedgeC_j" type="hinge" axis="1 0 0" range="-0.03 0.25" damping="0.002" limited="true"/>' % p +
                 ramp('%s_wedgeC_g' % p, -0.0193, 0.0193, 0.0025, 0.0003, 0.0475, -0.0047, 0.0015,
                      'mass="0.025" %s' % hard) + '</body>')
    # hinged wedgelets: pin at (y 0.102, z 0.0145); plate underside from (0.100, 0.0205) to (0.128, 0.0005)
    for side, x0, x1 in (('L', -0.088, -0.027), ('R', 0.027, 0.088)):
        b.append('<body name="%s_wedge%s" pos="0 0.102 0.0145">' % (p, side) +
                 '<joint name="%s_wedge%s_j" type="hinge" axis="1 0 0" range="-0.035 0.244" damping="0.002" limited="true"/>' % (p, side) +
                 ramp('%s_wedge%s_g' % (p, side), x0, x1, -0.002, 0.006, 0.026, -0.014, 0.002,
                      'mass="%s" %s' % (f(m_wedge), hard)) + '</body>')
    # hinged skirts (build_robot_v4.py 4 Oct, final): 0.8 mm plates on 35 deg slanted PETG clips, wire at z 0.015,
    # 1 mm outside the wall. Plate centre (8.14, -6.98) mm from the wire, 19 mm long. Floor loads swing it
    # outward (free to 25 deg); sideways pushes swing it inward onto the clip/wall stop at 1.2 deg.
    cy, sy = math.cos(math.radians(17.5)), math.sin(math.radians(17.5))
    mode = V4_SKIRT_MODE
    lift_dz = float(mode[5:]) / 1000 if mode.startswith('fixed') else 0.0
    out_lim = 0.087 if mode == 'limit5' else 0.436
    spring = 'stiffness="0.3" springref="%s"' if mode == 'spring' else ''

    def joint(name, axis, lo, hi, ref):
        if mode.startswith('fixed'):
            return ''
        sp = (spring % f(ref)) if spring else ''
        return '<joint name="%s" type="hinge" axis="%s" range="%s" limited="true" damping="0.0005" %s/>' % (
            name, axis, f(lo, hi), sp)
    for side, sgn in ((('R', 1), ('L', -1)) if V4_SKIRTS else ()):
        lo, hi = (-out_lim, 0.021) if sgn > 0 else (-0.021, out_lim)
        q = (cy, 0, -sy * sgn, 0)                      # rotate -35 deg (R) / +35 deg (L) about y
        b.append('<body name="%s_skirt%s" pos="%s">%s'
                 '<geom name="%s_skirt%s_g" type="box" pos="%s" quat="%s" size="0.0004 0.1035 0.0095" mass="%s" %s/></body>' % (
                     p, side, f(sgn * 0.089, -0.0065, 0.015 + lift_dz),
                     joint('%s_skirt%s_j' % (p, side), '0 1 0', lo, hi, 0.05 * sgn),
                     p, side, f(sgn * 0.00814, 0, -0.00698), f(*q), f(m_skirt_side), hard))
    q = (math.cos(math.radians(72.5)), math.sin(math.radians(72.5)), 0, 0)   # 145 deg about x
    if V4_SKIRTS:
        b.append('<body name="%s_skirtB" pos="%s">%s'
                 '<geom name="%s_skirtB_g" type="box" pos="0 -0.00814 -0.00698" quat="%s" size="0.088 0.0004 0.0095" mass="%s" %s/></body>' % (
                     p, f(0, -0.111, 0.015 + lift_dz), joint('%s_skirtB_j' % p, '1 0 0', -out_lim, 0.021, 0.05),
                     p, f(*q), f(m_skirt_rear), hard))
    b.append('</body>')
    act = ''.join('<motor name="%s_m%d" joint="%s_w%d_j" gear="1" ctrllimited="false"/>' % (p, n, p, n) for n in range(4))
    act += '<motor name="%s_rotor_m" joint="%s_rotor_j" gear="1" ctrllimited="false"/>' % (p, p)
    meta = {'kind': 'V4', 'wheels': [0, 1, 2, 3], 'left': [0, 2], 'right': [1, 3], 'r_wheel': 0.032,
            # JGA25-370 400 rpm at 11.1 V: stall 0.113 N*m, no-load 38.7 rad/s at the wheel
            'stall': 0.8 * 620 / V4_DRIVE_RPM * 0.0981 * V_PACK / 12, 'w0': V4_DRIVE_RPM * V_PACK / 12 * 2 * math.pi / 60,
            'invertible': True, 'weapon': 'rotor'}
    return '\n'.join(b), act, meta


def v2(prefix, pos, yaw, mu_tyre, col):
    tyre = 'friction="%s 0.005 0.0001" %s' % (f(mu_tyre), col)
    hard = 'friction="0.3 0.005 0.0001" %s' % col
    m_wheel, m_spat = 0.026, 0.035
    m_chassis = 1.087 - 2 * m_wheel - m_spat
    p = prefix
    b = ['<body name="%s" pos="%s" quat="%s"><freejoint name="%s_free"/>' % (p, f(*pos), f(*quat_yaw(yaw)), p),
         # CoG 0.5 mm ahead of the axle, 30.5 mm up (duel_sim, maker masses)
         '<inertial pos="0 0.0005 0.0305" mass="%s" diaginertia="%s"/>' % (
             f(m_chassis), f(m_chassis / 12 * (0.18 ** 2 + 0.082 ** 2), m_chassis / 12 * (0.16 ** 2 + 0.082 ** 2),
                             m_chassis / 12 * (0.16 ** 2 + 0.18 ** 2))),
         box(p + '_base', -0.080, 0.080, -0.090, 0.090, 0.003, 0.007, 'mass="0" ' + hard),
         box(p + '_body', -0.080, 0.080, -0.090, 0.040, 0.007, 0.085, 'mass="0" ' + hard),
         # steep front side wedges (CAD: x 35..80, y 40..90, z 5..83)
         ramp(p + '_wedgeL', -0.080, -0.035, 0.040, 0.083, 0.090, 0.005, 0.002, 'mass="0" ' + hard),
         ramp(p + '_wedgeR', 0.035, 0.080, 0.040, 0.083, 0.090, 0.005, 0.002, 'mass="0" ' + hard)]
    for n, x in enumerate((-0.0708, 0.0708)):
        b.append(wheel(p, 'w%d' % n, x, 0.0, 0.021, 0.021, 0.008, m_wheel, tyre))
    # spatula lifter: pivot (y 0.068, z 0.0315), tip (0.112, 0.0058); servo 0..45 deg, ~60 N at the tip
    b.append('<body name="%s_spat" pos="0 0.068 0.0315"><joint name="%s_spat_j" type="hinge" axis="1 0 0" '
             'range="0 0.785" limited="true" damping="0.01"/>%s</body>' % (
                 p, p, ramp(p + '_spat_g', -0.031, 0.031, 0.0, 0.0, 0.044, -0.0257, 0.003,
                            'mass="%s" %s' % (f(m_spat), hard))))
    b.append('</body>')
    act = ''.join('<motor name="%s_m%d" joint="%s_w%d_j" gear="1" ctrllimited="false"/>' % (p, n, p, n) for n in range(2))
    act += ('<position name="%s_spat_s" joint="%s_spat_j" kp="60" ctrlrange="0 0.785" ctrllimited="true" '
            'forcerange="-3 3" forcelimited="true"/>' % (p, p))
    meta = {'kind': 'V2', 'wheels': [0, 1], 'left': [0], 'right': [1], 'r_wheel': 0.021,
            'stall': 0.346, 'w0': 1.8 / 0.021, 'invertible': False, 'weapon': 'lifter', 'lift_max': 0.785}
    return '\n'.join(b), act, meta


def r3(prefix, pos, yaw, mu_tyre, col):
    tyre = 'friction="%s 0.005 0.0001" %s' % (f(mu_tyre), col)
    hard = 'friction="0.3 0.005 0.0001" %s' % col
    m_wheel, m_lift = 0.0262, 0.040
    m_chassis = 1.237 - 4 * m_wheel - m_lift
    p = prefix
    b = ['<body name="%s" pos="%s" quat="%s"><freejoint name="%s_free"/>' % (p, f(*pos), f(*quat_yaw(yaw)), p),
         '<inertial pos="0 0.0147 0.0147" mass="%s" diaginertia="%s"/>' % (
             f(m_chassis), f(m_chassis / 12 * (0.197 ** 2 + 0.04 ** 2), m_chassis / 12 * (0.168 ** 2 + 0.04 ** 2),
                             m_chassis / 12 * (0.168 ** 2 + 0.197 ** 2))),
         box(p + '_tub', -0.064, 0.064, -0.082, 0.076, 0.004, 0.037, 'mass="0" ' + hard),
         box(p + '_guardL', -0.084, -0.064, -0.029, 0.029, 0.006, 0.030, 'mass="0" ' + hard),
         box(p + '_guardR', 0.064, 0.084, -0.029, 0.029, 0.006, 0.030, 'mass="0" ' + hard),
         box(p + '_rear', -0.064, 0.064, -0.084, -0.080, 0.001, 0.020, 'mass="0" ' + hard),
         box(p + '_bulk', -0.084, 0.084, 0.076, 0.086, 0.010, 0.030, 'mass="0" ' + hard),
         # fixed plow wings: tip 1 mm, 25.3 deg ramp (duel_sim), x 40.5..84
         ramp(p + '_wingL', -0.084, -0.0405, 0.086, 0.0147, 0.115, 0.001, 0.002, 'mass="0" ' + hard),
         ramp(p + '_wingR', 0.0405, 0.084, 0.086, 0.0147, 0.115, 0.001, 0.002, 'mass="0" ' + hard)]
    for n, (x, y) in enumerate([(-0.074, 0.058), (0.074, 0.058), (-0.074, -0.060), (0.074, -0.060)]):
        b.append(wheel(p, 'w%d' % n, x, y, 0.021, 0.021, 0.008, m_wheel, tyre))
    # centre lifting wedge: pivot (0.0805, 0.011), tip (0.115, 0.001); servo 0..60 deg
    b.append('<body name="%s_lift" pos="0 0.0805 0.011"><joint name="%s_lift_j" type="hinge" axis="1 0 0" '
             'range="0 1.047" limited="true" damping="0.01"/>%s</body>' % (
                 p, p, ramp(p + '_lift_g', -0.0395, 0.0395, 0.0, 0.0, 0.0345, -0.010, 0.003,
                            'mass="%s" %s' % (f(m_lift), hard))))
    b.append('</body>')
    act = ''.join('<motor name="%s_m%d" joint="%s_w%d_j" gear="1" ctrllimited="false"/>' % (p, n, p, n) for n in range(4))
    act += ('<position name="%s_lift_s" joint="%s_lift_j" kp="80" ctrlrange="0 1.047" ctrllimited="true" '
            'forcerange="-4 4" forcelimited="true"/>' % (p, p))
    meta = {'kind': 'R3', 'wheels': [0, 1, 2, 3], 'left': [0, 2], 'right': [1, 3], 'r_wheel': 0.021,
            'stall': 0.33, 'w0': 1.8 / 0.021, 'invertible': True, 'weapon': 'lifter', 'lift_max': 1.047}
    return '\n'.join(b), act, meta


BUILDERS = {'V2': v2, 'R3': r3, 'V4': v4}


def world(robots, arena=2.4, mu_tyre=0.6, solref=(0.004, 1.0), solimp=(0.9, 0.95, 0.001), dt=2e-5):
    """robots: list of (kind, prefix, (x, y), yaw)."""
    bits = [2, 4]
    bodies, acts, metas = [], [], {}
    for i, (kind, prefix, xy, yaw) in enumerate(robots):
        other = 1 | (bits[1 - i] if len(robots) > 1 else 0)
        col = 'contype="%d" conaffinity="%d"' % (bits[i], other)
        bx, ax, meta = BUILDERS[kind](prefix, (xy[0], xy[1], 0.002), yaw, mu_tyre, col)
        bodies.append(bx)
        acts.append(ax)
        metas[prefix] = meta
    half = arena / 2
    walls = ''.join(
        '<geom name="wall%d" type="box" pos="%s" size="%s" contype="1" conaffinity="6" friction="0.3 0.005 0.0001"/>' % (
            n, f(*pos), f(*size))
        for n, (pos, size) in enumerate([((0, half + 0.05, 0.1), (half + 0.1, 0.05, 0.1)),
                                         ((0, -half - 0.05, 0.1), (half + 0.1, 0.05, 0.1)),
                                         ((half + 0.05, 0, 0.1), (0.05, half + 0.1, 0.1)),
                                         ((-half - 0.05, 0, 0.1), (0.05, half + 0.1, 0.1))]))
    xml = """<mujoco model="duel">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="%s" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default><geom solref="%s" solimp="%s" condim="3"/></default>
  <visual><global offwidth="960" offheight="720"/></visual>
  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="%s 0.1" contype="1" conaffinity="6" friction="0.3 0.005 0.0001" rgba="0.75 0.75 0.72 1"/>
    %s
    %s
  </worldbody>
  <actuator>%s</actuator>
</mujoco>""" % (f(dt), f(*solref), f(*solimp), f(half + 0.2, half + 0.2), walls, '\n'.join(bodies), ''.join(acts))
    return xml, metas


# ------------------------------------------------------------------ runtime
class Robot:
    def __init__(self, m, d, prefix, meta):
        self.p, self.meta, self.m, self.d = prefix, meta, m, d
        self.body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, prefix)
        self.wheel_act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, '%s_m%d' % (prefix, n)) for n in meta['wheels']]
        self.wheel_dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, '%s_w%d_j' % (prefix, n))]
                          for n in meta['wheels']]
        self.wheel_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, '%s_w%d_g' % (prefix, n)) for n in meta['wheels']}
        self.geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in self.bodies()}
        if meta['weapon'] == 'rotor':
            self.rotor_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, prefix + '_rotor_m')
            self.rotor_dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefix + '_rotor_j')]
            self.throttle = 0.0
        else:
            name = prefix + ('_spat_s' if meta['kind'] == 'V2' else '_lift_s')
            self.servo = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.lift_until, self.cool_until = -1.0, -1.0
            self.servo_cmd = 0.0

    def servo_step(self, target, dt):
        """Slew the lifter target at the real servo speed (a bare position actuator snaps like a flipper)."""
        step = SERVO_RATE * dt
        self.servo_cmd += float(np.clip(target - self.servo_cmd, -step, step))
        self.d.ctrl[self.servo] = self.servo_cmd

    def bodies(self):
        root = self.body
        return {b for b in range(self.m.nbody) if self._root(b) == root}

    def _root(self, b):
        while self.m.body_parentid[b] != 0:
            b = self.m.body_parentid[b]
        return b

    def pose(self):
        x = self.d.xpos[self.body]
        R = self.d.xmat[self.body].reshape(3, 3)
        return x, R[:, 1], R[:, 2]          # position, forward (body y), up (body z)

    def wheel_cmd(self, left, right):
        """left/right in -1..1; DC motor torque with back-EMF; positive = drive forward when upright."""
        meta = self.meta
        for k, (a, dof) in enumerate(zip(self.wheel_act, self.wheel_dof)):
            cmd = left if meta['wheels'][k] in meta['left'] else right
            w = -self.d.qvel[dof]                     # wheel spin that drives forward (see wheel sign note)
            tq = meta['stall'] * (cmd - w / meta['w0'])
            tq = float(np.clip(tq, -meta['stall'], meta['stall']))
            self.d.ctrl[a] = -tq

    def rotor_step(self, dt_ctrl, spin=True):
        target = 1.0 if spin else 0.0
        self.throttle = min(target, self.throttle + RAMP_US_PER_S / 1000.0 * dt_ctrl) if target > self.throttle else target
        w = self.d.qvel[self.rotor_dof] * ROTOR_RATIO                 # motor speed
        amps = max((self.throttle * V_PACK - KE * w) / R_M, 0.0)
        self.d.ctrl[self.rotor_act] = KE * max(amps - I0, 0.0) * ROTOR_RATIO * 0.92   # belt efficiency
        return amps

    def rotor_rpm(self):
        return self.d.qvel[self.rotor_dof] * 60 / (2 * math.pi)


_F6 = np.zeros(6)


def wheel_load_fraction(m, d, robot, floor_id):
    """Sum of floor normal force on the robot's wheels / its weight. Below ~0.25 the drive has little grip."""
    tot = 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        if (c.geom1 == floor_id and c.geom2 in robot.wheel_geoms) or (c.geom2 == floor_id and c.geom1 in robot.wheel_geoms):
            mujoco.mj_contactForce(m, d, i, _F6)
            tot += _F6[0]
    return tot / (m.body_subtreemass[robot.body] * G)


def touching(m, d, ra, rb):
    for i in range(d.ncon):
        c = d.contact[i]
        if (c.geom1 in ra.geoms and c.geom2 in rb.geoms) or (c.geom2 in ra.geoms and c.geom1 in rb.geoms):
            return True
    return False


def drive_towards(robot, target_xy, yaw_rate, rotor_fast):
    x, fwd, up = robot.pose()
    v = np.array(target_xy) - x[:2]
    fwd2 = fwd[:2] / (np.linalg.norm(fwd[:2]) + 1e-9)
    err = math.atan2(fwd2[0] * v[1] - fwd2[1] * v[0], fwd2[0] * v[0] + fwd2[1] * v[1])
    thr = max(math.cos(err), 0.0) if abs(err) < 1.2 else 0.0
    st = float(np.clip(2.5 * err, -1, 1))
    if robot.meta['kind'] == 'V4' and rotor_fast and abs(yaw_rate) > YAW_LIMIT:
        st *= YAW_LIMIT / abs(yaw_rate)
    inverted = up[2] < 0
    if inverted and robot.meta['invertible']:
        thr = -thr                                   # only throttle flips when rolled over (see firmware)
    # CCW (err > 0) needs the right side faster
    robot.wheel_cmd(float(np.clip(thr - st, -1, 1)), float(np.clip(thr + st, -1, 1)))
    return err, float(np.linalg.norm(v))


def engagement(kind_a, kind_b, seed, arena=2.4, mu_tyre=0.6, solref=(0.004, 1.0), dt=2e-5, t_end=6.0,
               ctrl_dt=1e-3, record=None, start=None, prespin=False, scripted=False, b_still=False):
    """Both robots chase each other with the same driver. Returns per-robot outcome stats.
    record: optional dict; receives 'xml' and 'frames' [(time, qpos)] every record['frame_dt'] s (default 1/30)."""
    rng = np.random.default_rng(seed)
    if start is None:
        sep = rng.uniform(0.5, min(1.0, arena * 0.6))
        ang = rng.uniform(0, 2 * math.pi)
        ca = np.array([math.cos(ang), math.sin(ang)]) * sep / 2
        ya = math.atan2(-ca[0], ca[1]) + math.pi * 0 + rng.normal(0, 0.8)   # roughly facing the centre, noisy
        yb = math.atan2(ca[0], -ca[1]) + rng.normal(0, 0.8)
        start = [((ca[0], ca[1]), ya), ((-ca[0], -ca[1]), yb)]
    xml, metas = world([(kind_a, 'A', start[0][0], start[0][1]), (kind_b, 'B', start[1][0], start[1][1])],
                       arena, mu_tyre, solref, dt=dt)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    if record is not None:
        record['xml'] = xml
        record.setdefault('frames', [])
    A, B = Robot(m, d, 'A', metas['A']), Robot(m, d, 'B', metas['B'])
    for r in (A, B):
        if prespin and r.meta['weapon'] == 'rotor':      # start with the weapon already at no-load speed
            d.qvel[r.rotor_dof] = (V_PACK - I0 * R_M) / KE / ROTOR_RATIO
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, r.p + '_rotor_j')]] = rng.uniform(0, 6.2832)
            r.throttle = 1.0
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    wall_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'wall%d' % n) for n in range(4)}
    nsub = max(1, int(round(ctrl_dt / dt)))
    stats = {r.p: {'inverted_s': 0.0, 'inv_run': 0.0, 'inv_run_max': 0.0, 'lifted_s': 0.0, 'lift_run_max': 0.0,
                   'lift_run': 0.0, 'pinned_s': 0.0, 'max_com_z': 0.0, 'max_rpm': 0.0, 'peak_amps': 0.0,
                   'hits_launch_10cm': 0}
             for r in (A, B)}
    next_frame = 0.0
    t = 0.0
    while t < t_end:
        for me, opp in ((A, B), (B, A)):
            x_opp = opp.pose()[0][:2]
            yaw_rate = d.cvel[me.body][2]
            rotor_fast = me.meta['weapon'] == 'rotor' and abs(me.rotor_rpm()) > 3000
            if scripted and b_still and me is B:          # target parked (wheels unpowered)
                me.wheel_cmd(0.0, 0.0)
                err, dist = 1.0, float(np.linalg.norm(x_opp - me.pose()[0][:2]))
            elif scripted:                               # straight full-throttle charge, no steering
                me.wheel_cmd(1.0, 1.0)
                err, dist = 0.0, float(np.linalg.norm(x_opp - me.pose()[0][:2]))
            else:
                err, dist = drive_towards(me, x_opp, yaw_rate, rotor_fast)
            if me.meta['weapon'] == 'rotor':
                amps = me.rotor_step(ctrl_dt)
                stats[me.p]['peak_amps'] = max(stats[me.p]['peak_amps'], amps)
                stats[me.p]['max_rpm'] = max(stats[me.p]['max_rpm'], abs(me.rotor_rpm()))
            else:
                if t >= me.cool_until and dist < 0.17 and abs(err) < 0.5 and me.pose()[2][2] > 0:
                    me.lift_until, me.cool_until = t + 0.6, t + 1.0
                me.servo_step(me.meta['lift_max'] if t < me.lift_until else 0.0, ctrl_dt)
        mujoco.mj_step(m, d, nstep=nsub)
        t += ctrl_dt
        for me, opp in ((A, B), (B, A)):
            s = stats[me.p]
            up = me.pose()[2]
            if up[2] < -0.3:
                s['inverted_s'] += ctrl_dt
                s['inv_run'] += ctrl_dt
            else:
                s['inv_run'] = 0.0
            s['inv_run_max'] = max(s['inv_run_max'], s['inv_run'])
            # 'lifted' counts only sustained loss of grip (>0.25 s): wheels chatter for a few ms at speed
            if wheel_load_fraction(m, d, me, floor) < 0.25:
                s['lift_run'] += ctrl_dt
                if s['lift_run'] > 0.25:
                    s['lifted_s'] += ctrl_dt
            else:
                s['lift_run'] = 0.0
            s['lift_run_max'] = max(s['lift_run_max'], s['lift_run'])
            if touching(m, d, me, opp):
                for i in range(d.ncon):
                    c = d.contact[i]
                    if (c.geom1 in wall_geoms and c.geom2 in me.geoms) or (c.geom2 in wall_geoms and c.geom1 in me.geoms):
                        s['pinned_s'] += ctrl_dt
                        break
            s['max_com_z'] = max(s['max_com_z'], float(d.subtree_com[me.body][2]))
        if record is not None and t >= next_frame:
            record['frames'].append((t, d.qpos.copy()))
            next_frame += record.get('frame_dt', 1 / 30)
        if not np.all(np.isfinite(d.qpos)):
            return {'error': 'diverged', 't': t}
    for r in (A, B):
        stats[r.p]['kind'] = r.meta['kind']
        stats[r.p]['final_inverted'] = bool(r.pose()[2][2] < -0.3)
    return stats


def classify(stats):
    """Same classes as three_way_sim: A_immobilises_B / B_immobilises_A / A_control / B_control / neutral."""
    a, b = stats['A'], stats['B']

    def immob(s):
        inv_dead = (not BUILD_INVERTIBLE[s['kind']]) and s['final_inverted'] and s['inv_run_max'] >= 1.5
        beached = s['lift_run_max'] >= 2.0
        return inv_dead or beached
    if immob(b) and not immob(a):
        return 'A_immobilises_B'
    if immob(a) and not immob(b):
        return 'B_immobilises_A'
    ca = b['lifted_s'] + b['pinned_s'] + b['inverted_s'] * (0 if BUILD_INVERTIBLE[b['kind']] else 1)
    cb = a['lifted_s'] + a['pinned_s'] + a['inverted_s'] * (0 if BUILD_INVERTIBLE[a['kind']] else 1)
    if ca - cb > 0.5:
        return 'A_control'
    if cb - ca > 0.5:
        return 'B_control'
    return 'neutral'


BUILD_INVERTIBLE = {'V2': False, 'R3': True, 'V4': True}
