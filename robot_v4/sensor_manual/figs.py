"""Figures for the V4 sensor manual. Each function returns an SVG string."""
import math

import draw as D
import geom as G
from draw import Fig, P, box, cyl, INK, HID, SENS, SIG, PWR, MOT, OK

XC, YC, ZC = '#c62828', '#2e7d32', '#1565c0'      # manual X (fwd) / Y (left) / Z (up) axis colours
S3_PT = (-32.5, -27.5, 46.0)     # ESC top-face centre (CAD envelope) - nominal only
S4_PT = (-20.0, 10.0, 44.5)      # top of the FL drive-motor can - nominal only
S5_PT = (52.0, -40.0, 48.0)      # at the ESP32 rear-right corner (GPIO34 side) - nominal only


def origin_page(view):
    return P(view, 0, G.ORIGIN_Y_CAD, 0)


def axes(f, view, L=40, size=4.4):
    o = origin_page(view)
    vecs = {'X': (0, 1, 0), 'Y': (-1, 0, 0), 'Z': (0, 0, 1)}
    cols = {'X': XC, 'Y': YC, 'Z': ZC}
    for k, (dx, dy, dz) in vecs.items():
        a = P(view, 0, G.ORIGIN_Y_CAD, 0)
        b = P(view, dx * L, G.ORIGIN_Y_CAD + dy * L, dz * L)
        if math.dist(a, b) < 1e-6:
            # axis points out of / into the page
            # out of the page: front view +X, top view +Z; right view looks along -x_cad, so +Y (= -x_cad) goes INTO the page
            into = (view == 'front' and k == 'X') or (view == 'top' and k == 'Z')
            f.circle(o, 3, color=cols[k], w=0.5, fill='#fff')
            if into:     # 'into' here = points at the viewer (out of the page): dot
                f.circle(o, 0.9, color=cols[k], fill=cols[k])
                f.text((o[0] + (4 if view == 'top' else -4), o[1] + 4), '%s+ (ออกจากกระดาษ)' % k, size=size - 0.6, color=cols[k], anchor='start' if view == 'top' else 'end')
            else:
                f.line((o[0] - 2, o[1] - 2), (o[0] + 2, o[1] + 2), color=cols[k], w=0.5)
                f.line((o[0] - 2, o[1] + 2), (o[0] + 2, o[1] - 2), color=cols[k], w=0.5)
                f.text((o[0] + (4 if view == 'top' else -4), o[1] + 4), '%s+ (เข้ากระดาษ)' % k, size=size - 0.6, color=cols[k], anchor='start' if view == 'top' else 'end')
            continue
        f.line(a, b, color=cols[k], w=0.8, arrow='end')
        f.text((b[0] + (3 if b[0] >= a[0] else -3), b[1] + 1.5), k + '+', size=size + 0.6, color=cols[k], weight='bold',
               anchor='start' if b[0] >= a[0] else 'end')
    f.circle(o, 1.4, color=INK, fill='#fff', w=0.5)
    f.text((o[0] - 3, o[1] - 6), 'O', size=size + 0.6, weight='bold', anchor='end')


# ------------------------------------------------------------ section 1
def coord_top(width_mm=120):
    f = Fig()
    D.robot(f, 'top', internals=False)
    axes(f, 'top', L=55)
    O = G.OVERALL
    f.dim((O['x0'], O['y1']), (O['x1'], O['y1']), 12, 'กว้างรวม %s (รวม skirt)' % G.fmt(O['x1'] - O['x0']))
    f.dim((-88, -110), (88, -110), -26, 'ตัวถัง 176')
    f.dim((O['x1'], O['y0']), (O['x1'], O['y1']), -14, 'ยาวรวม %s' % G.fmt(O['y1'] - O['y0']))
    f.dim((-118, 10), (-118, -70), 0, 'ฐานล้อ 80', ext_lines=False)
    f.line((-118, 10), (-78, 10), w=0.15)
    f.line((-118, -70), (-78, -70), w=0.15)
    f.dim((-70, -95), (70, -95), 0, 'ระยะล้อซ้าย-ขวา 140 (กึ่งกลางล้อ)', ext_lines=False)
    f.text((0, O['y1'] + 22), 'ด้านหน้า (หัวรถ)', size=4.6, weight='bold')
    f.text((0, O['y0'] - 40), 'ด้านหลัง', size=4.6, weight='bold')
    f.text((O['x0'] + 4, O['y1'] + 22), '← ซ้าย', size=4.2, weight='bold', anchor='start')
    f.text((O['x1'] - 4, O['y1'] + 22), 'ขวา →', size=4.2, weight='bold', anchor='end')
    f.scalebar((-100, O['y0'] - 52))
    return f.svg(width_mm=width_mm)


def coord_front(width_mm=110):
    f = Fig()
    D.robot(f, 'front', internals=False)
    axes(f, 'front', L=45)
    f.line((-120, 0), (120, 0), color=INK, w=0.6)
    for i in range(-12, 12):
        f.line((i * 10, 0), (i * 10 - 4, -4), color=HID, w=0.2)
    f.text((-118, -9), 'พื้น (Z = 0)', size=3.8, anchor='start')
    f.dim((112, 0), (112, 64), 6, 'สูงรวม 64 (ล้อ Ø64)')
    f.dim((-95, 4), (-95, 60), 0, 'ตัวถัง z 4–60', ext_lines=False)
    f.text((-110, 72), 'ขวารถ', size=4, anchor='start')
    f.text((110, 72), 'ซ้ายรถ', size=4, anchor='end')
    return f.svg(width_mm=width_mm)


def coord_side():
    f = Fig()
    D.robot(f, 'right', internals=False)
    axes(f, 'right', L=45)
    f.line((-135, 0), (140, 0), color=INK, w=0.6)
    for i in range(-13, 14):
        f.line((i * 10, 0), (i * 10 - 4, -4), color=HID, w=0.2)
    f.dim((10, 32), (10, 0), 0, '', ext_lines=False)
    f.text((14, 14), 'เพลาล้อสูง 32', size=3.6, anchor='start')
    f.dim((-110, 4), (-110, 0), 0, '', ext_lines=False)
    f.text((-112, 7.5), 'ใต้ท้อง 4', size=3.4, anchor='end')
    f.text((-10, -14), 'หลัง ← มองด้านขวาของรถ → หน้า', size=4)
    f.dim((-70, 66), (10, 66), 4, 'ฐานล้อ 80')
    return f.svg(width_mm=150)


def internal_layout():
    f = Fig()
    D.robot(f, 'top', internals=True, light=True)
    labels = [
        ('battery', (0, -20), (-130, -5), 'แบต 3S 850 mAh (พื้นถาด z 8–33)'),
        ('esc', (-45, -32), (-130, -45), 'ESC Skywalker 40A (ชั้นบน z 34–46)'),
        ('esp32', (45, -20), (130, -20), 'ESP32 DevKit (ชั้นบน z 34–48)'),
        ('drv_l', (50, -40), (130, -55), 'DRV8871 L (พื้น z 8–14)'),
        ('drv_r', (55, -8), (130, 5), 'DRV8871 R (พื้น z 8–14)'),
        ('switch', (73, -30), (130, -35), 'ตัวตัดไฟ XT30 link (ผนังขวา)'),
    ]
    for k, p, q, t in labels:
        box(f, 'top', G.BOX[k], color=INK, w=0.35, fill='#fff', opacity=0.0)
        f.leader(p, q, t, size=3.6)
    cyl(f, 'top', G.CYL['weapon_motor'], color=INK, w=0.35)
    f.leader((-25, 50), (-130, 40), 'มอเตอร์อาวุธ D3536 (outrunner)', size=3.6)
    cyl(f, 'top', G.CYL['pulley'], color=INK, w=0.35)
    f.leader((0, 52), (-130, 60), 'รอกมอเตอร์ Ø26 (มีแม่เหล็ก)', size=3.6)
    for k in ('motor_f_l', 'motor_f_r', 'motor_r_l', 'motor_r_r'):
        cyl(f, 'top', G.CYL[k], color=INK, w=0.3)
    f.leader((30, 10), (130, 30), 'มอเตอร์ขับ JGA25-370 ×4', size=3.6)
    f.leader((30, -70), (130, -75), '', size=3.6)
    D.sensors(f, 'top')
    f.leader((0, -48), (-130, -90), 'S1 IMU (พื้นถาด)', color=SENS, size=3.6)
    f.leader((7.25, 44), (130, 60), 'S2 Hall + เสา PETG', color=SENS, size=3.6)
    f.text((0, 140), 'ด้านหน้า ↑', size=4.4, weight='bold')
    f.scalebar((-100, -150))
    return f.svg(width_mm=165)


# ------------------------------------------------------------ section 2 layout
def _sensor_marks(f, view):
    D.sensors(f, view)
    pts = {
        'S1': G.centre(G.BOX['imu']), 'S2': G.centre(G.BOX['hall']), 'S3': S3_PT, 'S4': S4_PT, 'S5': S5_PT,
    }
    for k in ('S3', 'S4'):
        p = P(view, *pts[k])
        f.circle(p, 2.2, color=SENS, w=0.4, dash='1,0.6')
    p5 = P(view, *pts['S5'])
    f.rect(p5[0] - 2.5, p5[0] + 2.5, p5[1] - 1.5, p5[1] + 1.5, color=SENS, w=0.4, dash='1,0.6')
    return {k: P(view, *v) for k, v in pts.items()}


def _dir_arrows(f, view, pp):
    # S1: Z axis up (+ yaw arrow in top view)
    if view == 'top':
        c = pp['S1']
        f.arc_path(c, 9, 20, 300, color=SENS, w=0.5)
        f.line((c[0] + 9 * math.cos(math.radians(300)), c[1] + 9 * math.sin(math.radians(300))),
               (c[0] + 9 * math.cos(math.radians(305)) + 0.5, c[1] + 9 * math.sin(math.radians(305)) + 0.4),
               color=SENS, w=0.5, arrow='end')
        f.line(c, (c[0], c[1] + 16), color=SENS, w=0.5, arrow='end')
    else:
        c = pp['S1']
        f.line(c, (c[0], c[1] + 16), color=SENS, w=0.6, arrow='end')
    # S2: sensing axis points to -x (toward the pulley)
    a = P(view, 6.5, 44, 39)
    b = P(view, 6.5 - 10, 44, 39)
    if math.dist(a, b) > 1:
        f.line(a, b, color=SENS, w=0.6, arrow='end')
    else:
        f.circle(a, 2.6, color=SENS, w=0.4)
    # S3, S4: contact probes, arrow into the surface (down)
    for k in ('S3', 'S4'):
        a = pp[k]
        if view == 'top':
            f.circle(a, 0.8, color=SENS, fill=SENS)
        else:
            f.line((a[0], a[1] + 10), (a[0], a[1] + 1), color=SENS, w=0.5, arrow='end')


def layout(view, width_mm=150):
    f = Fig()
    D.robot(f, view, internals=(view == 'top'), light=True)
    pp = _sensor_marks(f, view)
    _dir_arrows(f, view, pp)
    # badges placed outside the body
    place = {
        'top':   {'S1': (-60, -95), 'S2': (60, 90), 'S3': (-125, -10), 'S4': (-125, 30), 'S5': (125, -60)},
        'front': {'S1': (100, -14), 'S2': (-60, 90), 'S3': (70, 90), 'S4': (100, 80), 'S5': (-110, 85)},
        'right': {'S1': (-70, -18), 'S2': (60, 90), 'S3': (-40, 90), 'S4': (10, 90), 'S5': (-100, 85)},
        'rear':  {'S1': (-100, -14), 'S2': (60, 90), 'S3': (-70, 90), 'S4': (-100, 80), 'S5': (110, 85)},
    }[view]
    for k, q in place.items():
        f.leader(pp[k], q, '', color=SENS, badge=k)
    names = {'top': 'ด้านหน้า ↑', 'front': 'มองจากด้านหน้า', 'right': 'มองด้านขวา (หน้า → ขวามือ)', 'rear': 'มองจากด้านหลัง'}
    f.text((0, f.vmin - 8) if view == 'top' else (0, -24), names[view], size=4.2, weight='bold')
    return f.svg(width_mm=width_mm)


# ------------------------------------------------------------ section 3: S1
def s1_location():
    f = Fig()
    # rear half of the tub only
    f.rect(-88, 88, -110, 0, color=INK, w=0.45)
    f.rect(-80, 80, -104, 0, color=INK, w=0.25)
    for k in ('motor_r_l', 'motor_r_r', 'wheel_r_l', 'wheel_r_r'):
        cyl(f, 'top', G.CYL[k], color=HID, w=0.3, dash='2,1')
    box(f, 'top', G.BOX['battery'], color=HID, w=0.3, dash='2,1')
    f.text((0, -22), 'แบต (y −38…−3)', size=3.4, color=HID)
    box(f, 'top', G.BOX['imu'], color=SENS, w=0.6, fill=SENS, opacity=0.18)
    f.text((0, -49.3), 'S1 GY-521', size=3.4, color=SENS, weight='bold')
    f.line((0, -112), (0, 4), color=INK, w=0.2, dash='6,1.5,1,1.5')
    f.text((2, 2), 'เส้นกึ่งกลางรถ (Y = 0)', size=3.2, anchor='start')
    f.dim((10.5, -110), (10.5, -56), -26, 'ผนังหลังด้านนอก → ขอบหลังบอร์ด 54')
    f.dim((-10.5, -110), (-10.5, -48), 36, 'ถึงกึ่งกลางบอร์ด 62')
    f.dim((-10.5, -56), (10.5, -56), -10, '21')
    f.dim((10.5, -56), (10.5, -40), -8, '16')
    f.dim((-88, -48), (-10.5, -48), 0, 'ผนังซ้ายนอก → ขอบบอร์ด 77.5', ext_lines=False)
    f.dim((10.5, -36), (88, -36), 0, 'ขอบบอร์ด → ผนังขวานอก 77.5', ext_lines=False)
    # board X arrow (CAD intent: board X -> robot right)
    f.line((-6, -44), (6, -44), color=SENS, w=0.4, arrow='end')
    f.text((8, -45.2), 'X บอร์ด', size=2.8, color=SENS, anchor='start')
    f.text((0, 10), 'ด้านหน้า ↑', size=4, weight='bold')
    f.scalebar((40, -125), 20)
    return f.svg(width_mm=125)


def s1_side():
    f = Fig()
    f.line((-70, 0), (-26, 0), w=0.5)
    for i in range(-7, -2):
        f.line((i * 10 + 4, 0), (i * 10 + 1, -3), color=HID, w=0.2)
    f.rect(-70, -26, 4, 7, color=INK, w=0.35, fill='url(#hatch)')
    f.rect(-56, -40, 7, 8, color=INK, w=0.25, fill='#ffe082')
    f.rect(-56, -40, 8, 9.6, color=SENS, w=0.35, fill=SENS, opacity=0.3)
    f.rect(-50, -46, 9.6, 11.5, color=INK, w=0.25, fill='#555')
    f.leader((-66, 5.5), (-80, 12), 'พื้นถาด TPU 3 มม. (z 4–7)', size=3)
    f.leader((-41, 7.5), (-20, 4), 'เทปโฟม 2 หน้า 1 มม.', size=3)
    f.leader((-42, 9), (-20, 10), 'PCB GY-521 (ด้านชิ้นส่วนขึ้น)', size=3)
    f.leader((-47, 11.2), (-20, 16), 'ชิป MPU6050', size=3)
    f.dim((-60, 0), (-60, 8), 9, 'สูง 8')
    c = (-48, 11.5)
    f.line(c, (c[0], c[1] + 12), color=ZC, w=0.5, arrow='end')
    f.text((c[0] - 1.5, c[1] + 10), 'Z+ (ขึ้น)', size=3, color=ZC, anchor='end')
    f.text((-48, -7), 'Pitch = 0°, Roll = 0° (ขนานพื้นถาด)  ·  มองด้านขวา: ←หลัง  หน้า→', size=3)
    return f.svg(width_mm=165)


# ------------------------------------------------------------ section 3: S2
def s2_location():
    f = Fig()
    f.rect(-45, 30, 62, 64, color=INK, w=0.4, fill='url(#hatch)')
    f.rect(-6, 6, 61, 65, color=INK, w=0.3, fill='#fff')
    f.text((-25, 65.5), 'ผนังกั้นหน้า y 62–64', size=3)
    cyl(f, 'top', G.CYL['weapon_motor'], color=HID, w=0.3, dash='2,1')
    f.text((-23, 44), 'D3536', size=3, color=HID)
    f.text((-23, 39.5), '(กระป๋องหมุน)', size=2.6, color=HID)
    box(f, 'top', G.BOX['motor_mount'], color=HID, w=0.3)
    cyl(f, 'top', G.CYL['pulley'], color=INK, w=0.4, fill='#ddd')
    box(f, 'top', G.BOX['hall_post'], color=SENS, w=0.35, fill='#fff3e0')
    box(f, 'top', G.BOX['hall'], color=SENS, w=0.4, fill=SENS, opacity=0.6)
    f.line((0, 22), (0, 68), color=INK, w=0.2, dash='6,1.5,1,1.5')
    f.text((0, 69.5), 'กึ่งกลางรถ', size=2.6)
    f.dim((4, 46), (6.5, 46), 10, '2.5')
    f.dim((0, 31), (7.25, 31), -6, '7.25')
    f.dim((12, 44), (12, 64), -14, '20 (ถึงหน้าผนังกั้น)')
    f.dim((8, 48), (12, 48), 3, '4')
    f.dim((12, 40), (12, 48), -4, '8')
    f.line((6.5, 44), (1, 44), color=SENS, w=0.5, arrow='end')
    f.leader((7.5, 43), (32, 36), 'S2 A3144: หน้าแบนหันหารอก', color=SENS, size=3)
    f.leader((10, 41), (32, 29), 'เสา PETG 4×8×36', color=SENS, size=3)
    f.text((-8, 18), 'ด้านหน้า ↑ (มองจากบน, กำลังขยายเห็นชัด)', size=3, weight='bold')
    return f.svg(width_mm=130)


def s2_section():
    """Section at y = 44 seen from the front. Page u = -x (robot's left to the page right)."""
    f = Fig()
    V = 'front'
    f.line(P(V, 34, 0, 0), P(V, -52, 0, 0), w=0.5)
    box(f, V, (-50, 34, 0, 0, 4, 7), color=INK, w=0.35, fill='url(#hatch)')
    cyl(f, V, G.CYL['weapon_motor'], color=HID, w=0.3, dash='2,1')
    box(f, V, G.BOX['motor_mount'], color=HID, w=0.3)
    cyl(f, V, G.CYL['pulley'], color=INK, w=0.4, fill='#ddd')
    box(f, V, G.BOX['hall_post'], color=SENS, w=0.35, fill='#fff3e0')
    box(f, V, G.BOX['hall'], color=SENS, w=0.4, fill=SENS, opacity=0.6)
    box(f, V, (2, 4, 0, 0, 37.5, 40.5), color=INK, w=0.3, fill='#455a64')
    box(f, V, (2, 4, 0, 0, 23.5, 26.5), color=INK, w=0.3, fill='#90a4ae')
    f.line(P(V, 26, 0, 32), P(V, -46, 0, 32), color=INK, w=0.2, dash='6,1.5,1,1.5')
    f.text(P(V, -22, 0, 33), 'แกนรอก z 32', size=2.8)
    f.dim(P(V, 4, 0, 46), P(V, 6.5, 0, 46), -6, '2.5')
    f.dim(P(V, 30, 0, 0), P(V, 30, 0, 39), 0, 'กลาง sensor สูงจากพื้น 39')
    f.dim(P(V, 18, 0, 7), P(V, 18, 0, 43), 0, 'เสา 36', tpos=0.5)
    f.dim(P(V, -48, 0, 32), P(V, -48, 0, 39), 3, 'r 7')
    f.leader(P(V, 3, 0, 39.5), P(V, -12, 0, 60), 'แม่เหล็ก 1: ขั้ว S หันออกหา sensor', size=2.8)
    f.leader(P(V, 3, 0, 24), P(V, -12, 0, 12), 'แม่เหล็ก 2 (ตรงข้าม 180°): ขั้ว N หันออก', size=2.8)
    f.line(P(V, 6.5, 0, 39), P(V, 2, 0, 39), color=SENS, w=0.5, arrow='end')
    f.text(P(V, -8, 0, 68), 'หน้าตัดที่ y = 44 มองจากหน้ารถ:  ขวารถ ←  → ซ้ายรถ', size=3, weight='bold')
    return f.svg(width_mm=140)


def s2_pulley_face():
    """Pulley +x face as seen from the right side (page u = y, v = z)."""
    f = Fig()
    c = (G.MOTOR_Y, G.AXLE_Z)
    f.circle(c, 13, w=0.5, fill='#eee')
    f.circle(c, 2.5, w=0.3)
    f.circle(c, 7, color=HID, w=0.2, dash='1,0.8')
    f.circle((c[0], c[1] + 7), 1.5, w=0.3, fill='#455a64')
    f.circle((c[0], c[1] - 7), 1.5, w=0.3, fill='#90a4ae')
    f.text((c[0] + 3, c[1] + 6.2), 'S', size=2.6, anchor='start', weight='bold')
    f.text((c[0] + 3, c[1] - 7.8), 'N', size=2.6, anchor='start', weight='bold')
    f.rect(42, 46, 37, 41, color=SENS, w=0.45, dash='1,0.5')
    f.leader((46, 40.5), (62, 50), 'หน้าต่าง sensor (y 42–46, z 37–41)', color=SENS, size=2.8)
    f.dim((c[0], c[1]), (c[0], c[1] + 7), -9, 'r 7')
    f.arc_path(c, 17, 200, 330, color=INK, w=0.3)
    f.text((c[0], c[1] - 22), 'หมุนทางใดก็ได้: 1 พัลส์/รอบ', size=2.8)
    f.text((c[0], c[1] + 20), 'มองหน้ารอกจากด้านขวารถ', size=3, weight='bold')
    return f.svg(width_mm=95)
