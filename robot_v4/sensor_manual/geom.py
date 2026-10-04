"""V4 geometry for the sensor installation manual.

Every number here is copied from robot_v4/build_robot_v4.py (commit 9321d02) or
computed from those numbers. The manual never types a coordinate by hand.

CAD frame (build_robot_v4.py): x right, y forward (nose +Y), z up, floor z = 0, mm.
Manual frame (ROS REP-103 style): X forward, Y left, Z up.
  Origin O = on the floor, on the left-right centre line, midway between the
  front and rear axles (CAD y = -30).
  X_m = y_cad + 30,  Y_m = -x_cad,  Z_m = z_cad
"""
import math

ORIGIN_Y_CAD = -30.0          # midpoint of WHEEL_Y = (10, -70)


def to_manual(x, y, z):
    return (y - ORIGIN_Y_CAD, -x, z)


# ---- constants straight from build_robot_v4.py ------------------------------
AXLE_Z = 32.0
ROTOR_Y = 95.0
MOTOR_Y = 44.0
WHEEL_R = 32.0
WHEEL_Y = (10.0, -70.0)
TIP_R, BASE_R = 29.5, 23.0
LID_SCREWS_Y = (-95, -15, 55)

TUB = dict(x0=-88, x1=88, y0=-110, y1=64, z0=4, z1=60)   # floor bottom z 4, lid top z 60
FLOOR_TOP_Z = 7.0
LID_UNDERSIDE_Z = 58.0
FRONT_BULKHEAD = dict(y0=62, y1=64)
REAR_WALL = dict(y0=-110, y1=-104)
SIDE_WALL = dict(x_in=80, x_out=88, y0=-110, y1=97)

# skirts (35 deg, 0.8 mm, bottom edge 0.5 mm above floor)
S35, C35 = math.sin(math.radians(35)), math.cos(math.radians(35))
SKIRT_P_TOP = (90.5 + 1.5 * S35, 16.8 - 1.5 * C35 + 0.5)
SKIRT_P_BOT = (90.5 + 16.8 / C35 * S35, 0.5)
SKIRT_T = 0.8
SKIRT_OUT_U = SKIRT_P_BOT[0] + SKIRT_T * C35          # outermost |x| of the side skirts
REAR_SKIRT_OUT_Y = -(SKIRT_OUT_U + 22)                  # rear skirt outermost y
WEDGELET_TIP_Y = 128.0
OVERALL = dict(x0=-SKIRT_OUT_U, x1=SKIRT_OUT_U, y0=REAR_SKIRT_OUT_Y, y1=WEDGELET_TIP_Y, z0=0.0, z1=64.0)

# boxes: name -> (x0, x1, y0, y1, z0, z1)
BOX = {
    'battery':  (-37.5, 37.5, -38, -3, 8, 33),
    'esc':      (-60, -5, -40, -15, 34, 46),
    'esp32':    (4, 55.5, -42, -13.7, 34, 48),
    'drv_l':    (39, 59, -45, -25, 8, 14),
    'drv_r':    (39, 59, -24, -4, 8, 14),
    'imu_pad':  (-10.5, 10.5, -56, -40, 7, 8),
    'imu':      (-10.5, 10.5, -56, -40, 8, 11.5),
    'hall_post': (8.0, 12.0, 40, 48, 7, 43),
    'hall':     (6.5, 8.0, 42, 46, 37, 41),
    'switch':   (66, 80, -34, -26, 20, 30),
    'motor_mount': (-45, -42, 26, 62, 10, 54),
    'skid':     (-20, 20, 64, 74, 1, 4),
    'top_brace': (-33, 33, 64, 74, 57, 60),
    'bot_brace': (-33, 33, 64, 74, 4, 7),
}

# cylinders along x: name -> (x0, x1, y, z, r)
CYL = {
    'weapon_motor': (-41, -5, MOTOR_Y, AXLE_Z, 17.5),
    'pulley':       (-4, 4, MOTOR_Y, AXLE_Z, 13.0),
    'hub':          (-20, 20, ROTOR_Y, AXLE_Z, 15.0),
    'disc_l':       (-26.5, -19.5, ROTOR_Y, AXLE_Z, TIP_R),
    'disc_r':       (19.5, 26.5, ROTOR_Y, AXLE_Z, TIP_R),
}
for wy, pos in zip(WHEEL_Y, ('f', 'r')):
    CYL['motor_%s_l' % pos] = (-56, -4, wy, AXLE_Z, 12.5)
    CYL['motor_%s_r' % pos] = (4, 56, wy, AXLE_Z, 12.5)
    CYL['wheel_%s_l' % pos] = (-78, -62, wy, AXLE_Z, WHEEL_R)
    CYL['wheel_%s_r' % pos] = (62, 78, wy, AXLE_Z, WHEEL_R)

UPRIGHT_PROFILE = [(64, 7), (64, 57), (104, 57), (112, 40), (112, 16.5), (100, 25), (98, 25), (98, 7)]  # (y, z)
UPRIGHT_X = [(-33, -27), (27, 33)]
WEDGELET_PROFILE = [(128, 0.5), (128, 2.5), (100, 22.5), (100, 20.5)]   # (y, z)
WEDGELET_X = [(-88, -27), (27, 88)]

# hall magnet: dia 3 x 2 pressed into the pulley's +x face (x = 4) at r = 7 mm (manufacturing/README.md)
MAGNET_R_ON_PULLEY = 7.0
PULLEY_FACE_X = 4.0
HALL_GAP = BOX['hall'][0] - PULLEY_FACE_X        # 2.5 mm


def centre(b):
    x0, x1, y0, y1, z0, z1 = b
    return ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def fmt(v, nd=1):
    s = ('%.' + str(nd) + 'f') % v
    if '.' in s:
        s = s.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


# ---- sensor records ---------------------------------------------------------
def sensor_table():
    """Per-sensor reference point in both frames plus distances to real faces."""
    imu = BOX['imu']
    hall = BOX['hall']
    out = {}
    # S1: reference = centre of the PCB underside (sits on the foam)
    cx, cy, _ = centre(imu)
    out['S1'] = dict(cad=(cx, cy, imu[4]), size=(imu[1] - imu[0], imu[3] - imu[2], imu[5] - imu[4]))
    # S2: reference = centre of the TO-92 body
    out['S2'] = dict(cad=centre(hall), size=(hall[1] - hall[0], hall[3] - hall[2], hall[5] - hall[4]))
    for k, v in out.items():
        x, y, z = v['cad']
        v['man'] = to_manual(x, y, z)
        v['d_rear_outer'] = y - TUB['y0']                 # from rear wall outer face (y -110)
        v['d_front_face'] = FRONT_BULKHEAD['y1'] - y       # to front bulkhead front face (y 64)
        v['d_right_outer'] = TUB['x1'] - x                 # from right wall outer face (x 88)
        v['d_left_outer'] = x - TUB['x0']                  # from left wall outer face (x -88)
        v['d_floor_top'] = z - FLOOR_TOP_Z
        v['d_ground'] = z
        v['d_lid'] = LID_UNDERSIDE_Z - z
        v['d_centre_xy'] = math.hypot(*v['man'][:2])
    return out


def selfcheck():
    # front axle centre: CAD (0, 10, 32) -> manual (40, 0, 32)
    assert to_manual(0, 10, 32) == (40.0, 0, 32)
    assert abs(HALL_GAP - 2.5) < 1e-9
    assert abs((OVERALL['x1'] - OVERALL['x0']) - 206) < 1.0, OVERALL
    assert abs((OVERALL['y1'] - OVERALL['y0']) - 253) < 1.0, OVERALL
    # hall centre height = pulley axis + magnet radius, so the magnet passes the sensor centre
    assert centre(BOX['hall'])[2] == AXLE_Z + MAGNET_R_ON_PULLEY
    assert centre(BOX['hall'])[1] == MOTOR_Y
    return True


def route_len(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


# Recommended cable routes (CAD frame). Not in the CAD model: these are the manual's recommendation.
ROUTES = {
    # S2 hall: legs up from the TO-92, over the post top, over the right-front drive motor, to the ESP32
    'S2': [(7.25, 44, 41), (7.25, 44, 48), (14, 40, 51), (25, 28, 51), (25, -10, 51), (25, -13.7, 50)],
    # S1 IMU: straight up from the board through the free column behind the battery, over the ESP32
    'S1': [(3, -48, 11.5), (3, -48, 51), (12, -30, 51), (12, -16, 50)],
    # S3/S4 DS18B20 bus: ESP32 front edge -> over the ESC top to its probe -> forward to the FL motor can
    'S3': [(10, -16, 50), (0, -16, 51), (-32.5, -24, 49), (-32.5, -27.5, 46.5)],
    'S4': [(-32.5, -24, 49), (-25, -8, 51), (-20, 2, 49), (-20, 10, 45)],
    # S5 divider tap: removable-link output (right wall) -> ESP32 rear-right corner
    'S5': [(66, -30, 25), (60, -30, 40), (55, -38, 44)],
}
# Power / motor power (schematic paths in the top view, recommendation)
POWER_ROUTES = {
    'batt_to_link': [(37.5, -20, 20), (66, -30, 25)],
    'link_to_esc':  [(66, -30, 30), (60, -12, 50), (0, -12, 52), (-5, -27, 46)],
    'link_to_drv':  [(66, -30, 20), (60, -30, 14)],
}
MOTOR_ROUTES = {
    'phase_D3536':  [(-45, 44, 40), (-50, 30, 50), (-50, -8, 50), (-45, -15, 44)],
    'drv_l_to_left_motors': [(39, -40, 10), (30, -75, 10), (-4, -75, 12), (-45, -75, 10),
                             (-45, -20, 10), (-45, 10, 12), (-4, 10, 20)],
    'drv_r_to_right_motors': [(39, -14, 10), (20, 2, 12), (4, 10, 20)],
    'drv_r_to_rear_right': [(50, -24, 10), (50, -70, 12), (4, -70, 20)],
}


if __name__ == '__main__':
    selfcheck()
    import json
    print(json.dumps(sensor_table(), indent=1))
    print('overall', OVERALL)
    for k, r in ROUTES.items():
        print(k, round(route_len(r)))
