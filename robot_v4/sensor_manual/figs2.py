"""Figures part 2: S3-S5, sensing map, hall timing, mechanical details."""
import math

import draw as D
import geom as G
from draw import Fig, P, box, cyl, INK, HID, SENS, SIG, PWR, MOT, OK
from figs import XC, YC, ZC


def s3_esc():
    f = Fig()
    f.rect(0, 55, 0, 25, w=0.5, fill='#f5f5f5')
    f.text((27.5, 26.5), 'ESC Skywalker 40A (envelope 55×25×12 จาก CAD)', size=3)
    for i in range(3):
        f.line((55, 6 + i * 6.5), (66, 6 + i * 6.5), color=MOT, w=0.8)
    f.text((67, 12), 'ไปมอเตอร์อาวุธ', size=2.8, anchor='start', color=MOT)
    f.line((0, 8), (-10, 8), color=PWR, w=0.9)
    f.line((0, 17), (-10, 17), color=INK, w=0.9)
    f.text((-11, 11), 'ไฟเข้า', size=2.8, anchor='end')
    f.rect(26, 34, 9, 16, color=SENS, w=0.5, fill=SENS, opacity=0.25)
    f.text((30, 11.2), 'S3', size=2.8, color=SENS, weight='bold')
    f.rect(21, 39, 7, 18, color=SENS, w=0.3, dash='1,0.6')
    f.leader((39, 18), (50, 34), 'เทป Kapton คลุมทับ', size=2.8)
    f.dim((0, 0), (26, 0), -6, 'a = ____ mm (วัดจริง)')
    f.dim((26, 0), (26, 9), -18, 'b = ____', tpos=0.5)
    f.text((27.5, -18), 'จุดติด = ด้านที่มี MOSFET / แผ่นระบายความร้อน (ดูจาก ESC จริง)', size=2.8, color=SENS)
    # side detail
    f.rect(70, 110, -30, -24, w=0.4, fill='#f5f5f5')
    f.text((90, -28.2), 'ESC', size=2.8)
    f.rect(85, 93, -24, -21.5, color=SENS, w=0.4, fill=SENS, opacity=0.4)
    f.poly([(80, -24), (80, -20.6), (98, -20.6), (98, -24)], closed=False, color='#e0a800', w=0.5)
    f.leader((89, -21), (104, -14), 'หน้าแบน DS18B20 แนบผิว + ซิลิโคนนำความร้อนบางๆ', size=2.6)
    f.text((90, -35), 'หน้าตัดด้านข้าง', size=2.8)
    return f.svg(width_mm=150)


def s4_motor():
    f = Fig()
    V = 'front'   # looking from the front: u = -x
    box(f, V, (-60, -56, 0, 0, 7, 58), w=0.4, fill='url(#hatch)')
    f.text(P(V, -58, 0, -6), 'ผนังยึดมอเตอร์', size=2.8)
    box(f, V, (-56, -4, 0, 0, 19.5, 44.5), w=0.5, fill='#f5f5f5')
    f.line(P(V, -34, 0, 19.5), P(V, -34, 0, 44.5), color=HID, w=0.3, dash='1.5,1')
    f.text(P(V, -45, 0, 30), 'เกียร์', size=3, color=HID)
    f.text(P(V, -19, 0, 30), 'กระป๋องมอเตอร์ 370', size=3, color=HID)
    f.text(P(V, -30, 0, 13), 'รอยต่อเกียร์/กระป๋อง = ____ mm จากผนัง (วัดจริง)', size=2.6, color=HID)
    box(f, V, (-23, -17, 0, 0, 44.5, 47.0), color=SENS, w=0.5, fill=SENS, opacity=0.35)
    f.text(P(V, -14, 0, 50), 'S4', size=3, color=SENS, weight='bold', anchor='end')
    f.line(P(V, -20, 0, 56), P(V, -20, 0, 47.5), color=SENS, w=0.5, arrow='end')
    f.circle(P(V, -36, 0, 32), 0.8, fill=INK)
    f.line(P(V, -62, 0, 32), P(V, 0, 0, 32), color=INK, w=0.2, dash='6,1.5,1,1.5')
    f.dim(P(V, -56, 0, 62), P(V, -20, 0, 62), 0, 'c = ____ mm (วัดจริง)')
    f.dim(P(V, 4, 0, 0), P(V, 4, 0, 44.5), 0, 'z 44.5 (CAD)')
    f.line(P(V, -62, 0, 0), P(V, 2, 0, 0), w=0.6)
    f.text(P(V, -30, 0, -7), 'มอเตอร์ขับหน้าซ้าย มองจากหน้ารถ (ซ้ายรถอยู่ขวามือ)', size=3)
    f.text(P(V, -30, 0, -12), 'ห้ามติดบนมอเตอร์อาวุธ D3536: กระป๋องหมุน', size=3, color=PWR, weight='bold')
    return f.svg(width_mm=110)


def s5_divider():
    f = Fig()

    def res(x, y0, y1, label):
        f.line((x, y0), (x, y0 - 3))
        f.rect(x - 2, x + 2, y0 - 3, y1 + 3, w=0.45, fill='#fff')
        f.line((x, y1 + 3), (x, y1))
        f.text((x + 4, (y0 + y1) / 2 - 1.2), label, size=3.2, anchor='start')
    f.text((0, 62), 'VBAT+ (หลัง removable link + ฟิวส์ 30 A)', size=3.2, anchor='start')
    f.line((0, 58), (20, 58), color=PWR, w=0.6)
    f.line((20, 58), (20, 52), color=PWR, w=0.6)
    res(20, 52, 34, '100 kΩ 1%')
    f.line((20, 34), (20, 30))
    f.circle((20, 30), 0.8, fill=INK)
    f.line((20, 30), (55, 30), color=SIG, w=0.5)
    f.text((57, 28.8), 'GPIO 34 (ADC1)', size=3.2, anchor='start', color=SIG)
    res(20, 30, 12, '22 kΩ 1%')
    f.line((20, 12), (20, 6))
    for i, w in enumerate((6, 4, 2)):
        f.line((20 - w, 6 - i * 1.4), (20 + w, 6 - i * 1.4), w=0.5)
    f.text((28, 3), 'GND ESP32', size=3, anchor='start')
    f.circle((40, 30), 0.6, fill=HID, color=HID)
    f.line((40, 30), (40, 24), color=HID, dash='1,0.6')
    f.text((42, 22), 'C 100 nF ลง GND (แนะนำ, ไม่อยู่ในแบบ)', size=2.6, color=HID, anchor='start')
    f.text((60, 50), '12.6 V → 2.27 V ที่ขา', size=3, anchor='start')
    f.text((60, 45), '10.9 V → 1.97 V (หายเตือน)', size=3, anchor='start')
    f.text((60, 40), '10.5 V → 1.89 V (เตือน > 2 s)', size=3, anchor='start')
    f.text((60, 35), 'สูงสุดที่ ADC แม่น ≈ 2.45 V', size=3, anchor='start')
    return f.svg(width_mm=120)


# ------------------------------------------------------------ section 5
def sensing_map():
    f = Fig()
    D.robot(f, 'top', internals=False, light=True)
    cyl(f, 'top', G.CYL['weapon_motor'], color=HID, w=0.3, dash='2,1')
    cyl(f, 'top', G.CYL['pulley'], color=INK, w=0.4)
    for k in ('esc',):
        box(f, 'top', G.BOX[k], color=HID, w=0.3, dash='2,1')
    cyl(f, 'top', G.CYL['motor_f_l'], color=HID, w=0.3, dash='2,1')
    D.sensors(f, 'top')
    # S1 axes
    c = (0, -48)
    f.line(c, (0, -48 + 30), color=XC, w=0.7, arrow='end')
    f.text((2, -22), 'X (หน้า)', size=3.2, color=XC, anchor='start')
    f.line(c, (-30, -48), color=YC, w=0.7, arrow='end')
    f.text((-31, -46.5), 'Y (ซ้าย)', size=3.2, color=YC, anchor='end')
    f.arc_path(c, 12, 200, 340, color=ZC, w=0.5)
    f.leader((0, -48), (-125, -95), 'S1 IMU: ทั้งตัวรถ (az, gz)', color=SENS, size=3.2)
    # S2 zone
    f.rect(-1, 8, 41, 47, color=SENS, w=0.3, fill=SENS, opacity=0.12)
    f.leader((4, 47), (125, 85), 'S2: แม่เหล็กผ่านหน้าต่าง', color=SENS, size=3.2)
    # S3/S4 contact zones
    f.rect(-40, -25, -34, -21, color=SENS, w=0.3, fill=SENS, opacity=0.15)
    f.leader((-32.5, -27.5), (-125, -20), 'S3: ผิว ESC', color=SENS, size=3.2)
    f.rect(-24, -16, 6, 14, color=SENS, w=0.3, fill=SENS, opacity=0.15)
    f.leader((-20, 10), (-125, 25), 'S4: มอเตอร์หน้าซ้าย', color=SENS, size=3.2)
    f.leader((52, -40), (125, -60), 'S5: แรงดันแบต (ไฟฟ้า)', color=SENS, size=3.2)
    f.text((0, 152), 'ด้านหน้า ↑', size=3.6, weight='bold')
    return f.svg(width_mm=165)


def hall_timing():
    f = Fig()
    T = 60.0 / 12400 * 1000      # ms per rev at operating rpm
    sx = 12.0                    # page units per ms
    y0 = 0
    f.line((0, y0), (sx * 16, y0), color=HID, w=0.2)
    pts = [(0, 10)]
    t = 1.2
    while t < 15.5:
        pts += [(t * sx, 10), (t * sx, 0), ((t + 0.35) * sx, 0), ((t + 0.35) * sx, 10)]
        t += T
    pts.append((16 * sx, 10))
    f.poly(pts, closed=False, color=SIG, w=0.6)
    f.text((-3, 8), 'GPIO32', size=3, anchor='end', color=SIG)
    f.text((-3, 2), '(pull-up 3.3 V)', size=2.4, anchor='end', color=SIG)
    f.dim((1.2 * sx, 12), ((1.2 + T) * sx, 12), 4, 'คาบ %.2f ms @ 12,400 rpm' % T)
    f.text((sx * 8, -6), 'ขอบขาลง (FALLING) = แม่เหล็กขั้ว S ผ่าน 1 ครั้ง/รอบ → rpm = 60,000,000 / คาบ(µs)', size=3)
    f.text((sx * 8, -11), 'ตัดพัลส์ที่ถี่กว่า 2 ms (>30,000 rpm = noise) · ไม่มีพัลส์ 0.2 s → rpm = 0 · เขียว ≥ 10,000 rpm (คาบ ≤ 6 ms)', size=2.8)
    return f.svg(width_mm=165)


# ------------------------------------------------------------ section 6
def hall_post_drawing():
    f = Fig()
    # front view (looking -y): 4 wide (x), 36 tall
    f.rect(0, 4, 0, 36, w=0.5, fill='#fff3e0')
    f.rect(-1.5, 0, 30, 34, color=SENS, w=0.4, fill=SENS, opacity=0.5)
    f.dim((0, 0), (4, 0), -5, '4')
    f.dim((4, 0), (4, 36), -6, '36')
    f.text((2, 40), 'มุมมองหน้า', size=3)
    f.text((-2, 32.5), 'A3144', size=2.4, anchor='end', color=SENS)
    f.line((2, -2), (2, 0.2), color=HID, w=0.3, dash='1,0.6')
    f.rect(1.2, 2.8, 0, 6, color=HID, w=0.3, dash='1,0.6')
    f.leader((2, 3), (-12, 8), 'รูนำ Ø1.6 ลึก 6 (แนะนำ)', size=2.6, color=HID)
    # side view: 8 wide (y), 36 tall
    f.rect(25, 33, 0, 36, w=0.5, fill='#fff3e0')
    f.dim((25, 0), (33, 0), -5, '8')
    f.text((29, 40), 'มุมมองข้าง', size=3)
    f.rect(27, 31, 30, 34, color=SENS, w=0.4, fill=SENS, opacity=0.5)
    f.text((29, 26.5), 'ตำแหน่ง TO-92', size=2.4, color=SENS)
    f.dim((33, 30), (33, 34), -4, '4')
    f.dim((33, 34), (33, 36), -9, '2')
    # print orientation
    f.rect(45, 81, 0, 4, w=0.5, fill='#fff3e0')
    f.line((43, -1), (84, -1), w=0.6)
    f.text((63, -5.5), 'ฐานพิมพ์', size=2.8)
    f.text((63, 8), 'วางนอนบนหน้า 8×36 (ชั้นพิมพ์วิ่งตามความยาว)', size=2.8)
    f.text((63, 13), 'PETG · infill 100% · ชั้น 0.2 มม. · ผนัง ≥ 3 รอบ', size=2.8)
    f.text((63, 40), 'ทิศพิมพ์แนะนำ', size=3)
    return f.svg(width_mm=150)


def imu_stack(width_mm=140):
    f = Fig()
    layers = [(0, 3, 'พื้นถาด TPU 95A (z 4–7)', 'url(#hatch)'),
              (3, 4, 'เทปโฟม 2 หน้า 1 มม. (21×16)', '#ffe082'),
              (4, 5.6, 'PCB GY-521 1.6 มม.', '#ef9a9a')]
    for z0, z1, t, fill in layers:
        f.rect(0, 60, z0 * 3, z1 * 3, w=0.4, fill=fill)
        f.text((63, (z0 + z1) * 1.5 - 1.2), t, size=3, anchor='start')
    f.rect(24, 36, 16.8, 20, w=0.3, fill='#424242')
    f.text((22, 18), 'MPU6050', size=2.6, anchor='end')
    for x in (6, 54):
        f.rect(x - 1.5, x + 1.5, 16.8, 18.5, color=HID, w=0.3, fill='#e0e0e0')
    f.leader((54, 18.5), (63, 26), 'กาวซิลิโคน/ฮอตกลู 2 มุม (แนะนำ สำรองกันหลุด)', size=2.6, color=HID)
    f.line((30, 20), (30, 34), color=ZC, w=0.6, arrow='end')
    f.text((32, 31), 'Z+', size=3, color=ZC, anchor='start')
    f.text((30, -6), 'ภาพขยายแนวตั้ง ×3 (ไม่ตามสเกล)', size=2.6, color=HID)
    return f.svg(width_mm=width_mm)


def ds_attach(width_mm=130):
    f = Fig()
    f.rect(0, 70, 0, 6, w=0.4, fill='#cfd8dc')
    f.text((35, 1.6), 'ผิวชิ้นส่วน (ESC / กระป๋องมอเตอร์)', size=2.8)
    f.rect(25, 30, 6, 6.6, color='#9e9e9e', w=0.2, fill='#bdbdbd')
    f.poly([(25, 6.6), (30, 6.6), (30, 10.5), (27.5, 11.5), (25, 10.5)], w=0.4, fill='#424242')
    f.text((27.5, 13), 'DS18B20', size=2.6)
    f.poly([(15, 6), (15, 7), (24, 7), (24, 12.2), (31, 12.2), (31, 7), (45, 7), (45, 6)], closed=False, color='#e0a800', w=0.6)
    f.leader((40, 7), (52, 14), 'Kapton กว้าง 10–15 มม. 2 รอบ', size=2.6)
    f.leader((27, 6.3), (8, 14), 'ซิลิโคนนำความร้อนบางๆ', size=2.6)
    for x in (26.2, 27.5, 28.8):
        f.line((x, 10.5), (x, 22), w=0.35)
    f.rect(25.6, 29.4, 16, 22, color=INK, w=0.3, fill='#fff', opacity=0.6)
    f.text((31, 19), 'ท่อหด 3 เส้นแยก + ท่อหดรวม', size=2.6, anchor='start')
    f.text((35, -5), 'หน้าแบน (มีตัวอักษร) แนบผิว · มอเตอร์: รัดเพิ่มด้วยเคเบิลไทร์ 1 เส้น', size=2.8)
    return f.svg(width_mm=width_mm)
