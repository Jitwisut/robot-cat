"""Figures part 4: installation steps, calibration, tilt test, safety."""
import math

import draw as D
import geom as G
from draw import Fig, P, box, cyl, INK, HID, SENS, SIG, PWR, MOT, OK
from figs import XC, YC, ZC


def _tub_top(f):
    f.rect(-88, 88, -110, 64, w=0.45)
    f.rect(-80, 80, -104, 62, w=0.2, color=HID)
    f.line((0, -114), (0, 68), w=0.2, dash='6,1.5,1,1.5')


def _cross(f, p, col=SENS, r=4):
    f.line((p[0] - r, p[1]), (p[0] + r, p[1]), color=col, w=0.5)
    f.line((p[0], p[1] - r), (p[0], p[1] + r), color=col, w=0.5)
    f.circle(p, r * 0.6, color=col, w=0.4)


def step1(w=None):
    f = Fig()
    _tub_top(f)
    _cross(f, (0, -48))
    _cross(f, (10, 44))
    f.text((0, -60), 'S1', size=4, color=SENS, weight='bold')
    f.text((22, 44), 'S2 (เสา)', size=4, color=SENS, weight='bold', anchor='start')
    f.text((0, 72), 'ทำเครื่องหมายบนพื้นถาด (มองจากบน)', size=4)
    return f.svg(width_mm=w or 70)


def step2(w=None):
    f = Fig()
    _tub_top(f)
    _cross(f, (0, -48), r=3)
    _cross(f, (10, 44), r=3)
    f.dim((40, -110), (40, -48), -4, '62')
    f.dim((10, -110), (10, 44), -40, '154')
    f.dim((10, 44), (88, 44), 6, '78 (ถึงกลางเสา)')
    f.text((0, 72), 'วัดจากผนังหลัง/ผนังขวาด้านนอก + เส้นกึ่งกลาง', size=4)
    return f.svg(width_mm=w or 70)


def step3(w=None):
    f = Fig()
    f.rect(-20, 20, 0, 3, w=0.4, fill='url(#hatch)')
    f.rect(-1.1, 1.1, 0, 3, color=SENS, w=0.4, fill='#fff')
    f.rect(-2, 2, 3, 30, w=0.4, fill='#fff3e0')
    f.rect(-0.8, 0.8, 3, 9, color=SENS, w=0.3, dash='1,0.5')
    f.line((0, -12), (0, -1), color=SENS, w=0.6, arrow='end')
    f.text((0, -16), 'สกรูเกลียวปล่อย M2×10 จากใต้ท้อง', size=3, color=SENS)
    f.text((4, 6), 'รูนำ Ø1.6 ลึก 6', size=2.8, anchor='start')
    f.text((4, 1), 'รูพื้นถาด Ø2.2', size=2.8, anchor='start')
    f.text((0, 34), 'ทางเลือกแนะนำ (ไม่อยู่ในแบบ)', size=3, color=HID)
    return f.svg(width_mm=w or 60)


def step4(w=None):
    f = Fig()
    cyl(f, 'top', G.CYL['pulley'], w=0.45, fill='#ddd')
    box(f, 'top', G.BOX['hall_post'], color=SENS, w=0.5, fill='#fff3e0')
    f.dim((4, 40), (8, 40), -6, '4.0 (หน้ารอก→เสา)')
    f.dim((12, 40), (12, 48), -4, '8')
    f.line((0, 30), (0, 58), w=0.2, dash='6,1.5,1,1.5')
    f.text((5, 62), 'ติดเสาให้หน้าเสาห่างหน้ารอก 4.0 มม.', size=3)
    return f.svg(width_mm=w or 60)


def step5(w=None):
    f = Fig()
    f.rect(0, 30, 0, 3, w=0.4, fill='url(#hatch)')
    f.rect(4, 25, 3, 4, w=0.3, fill='#ffe082')
    f.rect(4, 25, 4, 5.6, color=SENS, w=0.4, fill=SENS, opacity=0.3)
    f.text((14.5, 9), 'S1 บนโฟม', size=3, color=SENS)
    f.rect(45, 49, 0, 36, w=0.4, fill='#fff3e0')
    f.rect(43.5, 45, 30, 34, color=SENS, w=0.4, fill=SENS, opacity=0.6)
    for x in (43.8, 44.25, 44.7):
        f.line((x, 34), (x, 42), w=0.3)
    f.text((47, 45), 'S2 ขาชี้ขึ้น', size=3, color=SENS)
    return f.svg(width_mm=w or 70)


def step6(w=None):
    f = Fig()
    f.rect(-4, 0, 20, 46, w=0.45, fill='#ddd')
    f.text((-2, 16), 'รอก', size=2.8)
    f.rect(2.5, 4, 36, 42, color=SENS, w=0.4, fill=SENS, opacity=0.6)
    f.rect(4, 8, 7, 45, w=0.4, fill='#fff3e0')
    f.rect(0, 2.5, 30, 60, color='#607d8b', w=0.3, fill='#b0bec5', opacity=0.8)
    f.text((1.25, 63), 'ก้านดอกสว่าน Ø2.5 ใช้เป็นฟีลเลอร์', size=2.8)
    f.dim((0, 26), (2.5, 26), -3, '2.5')
    f.rect(25, 60, 20, 23, w=0.4, fill=SENS, opacity=0.3)
    f.line((20, 18), (65, 18), w=0.5)
    f.text((42.5, 27), 'IMU ขนานพื้น: az ≈ +1.00 g', size=2.8)
    return f.svg(width_mm=w or 75)


def step8(w=None):
    f = Fig()
    f.rect(0, 51.5, 0, 28.3, w=0.5, fill='#e3f2fd', color=SIG)
    f.rect(51.5, 57, 10, 18, w=0.4, fill='#fff')
    f.text((60, 13), 'USB', size=2.8, anchor='start')
    front = ['3V3', 'GND', 'D15', 'D2', 'D4', 'RX2', 'TX2', 'D5', 'D18', 'D19', 'D21', 'RX0', 'TX0', 'D22', 'D23']
    rear = ['VIN', 'GND', 'D13', 'D12', 'D14', 'D27', 'D26', 'D25', 'D33', 'D32', 'D35', 'D34', 'VN', 'VP', 'EN']
    use = {'3V3', 'GND', 'D4', 'D21', 'D22', 'VIN', 'D13', 'D14', 'D27', 'D26', 'D25', 'D32', 'D34'}
    for row, y, va in ((front, 26, 30), (rear, 2.3, -4)):
        for i, p in enumerate(row):
            x = 48 - i * 3.2
            c = SENS if p in use else HID
            f.circle((x, y), 0.8, color=c, fill=c if p in use else '#fff', w=0.3)
            f.text((x, va), p, size=2.4, color=c)
    f.text((25.75, 13), 'มองจากบน · USB หันไปทางขวารถ', size=2.6)
    f.text((25.75, 36), 'แถวหน้า (ทางหัวรถ)', size=2.6, color=HID)
    f.text((25.75, -9), 'แถวหลัง', size=2.6, color=HID)
    f.text((25.75, -14), 'ตำแหน่งขาจากความจำ: ตรวจกับป้ายบนบอร์ดจริงก่อนบัดกรี', size=2.4, color=PWR)
    return f.svg(width_mm=w or 95)


def step9(w=None):
    f = Fig()
    steps = ['1 ถอดโรเตอร์', '2 เสียบ USB เท่านั้น', '3 อ่าน Serial', '4 ต่อแบตผ่าน link', '5 ตรวจ telemetry']
    for i, s in enumerate(steps):
        x = i * 44
        f.rect(x, x + 38, 0, 12, w=0.4, fill='#fff')
        f.text((x + 19, 4.6), s, size=3.0)
        if i:
            f.line((x - 6, 6), (x, 6), w=0.4, arrow='end')
    return f.svg(width_mm=w or 170)


def cal_setup(w=None):
    f = Fig()
    # IMU upright / inverted
    f.line((0, 0), (40, 0), w=0.5)
    f.rect(5, 35, 0.5, 12, w=0.4, fill='#f5f5f5')
    f.line((20, 6), (20, 18), color=ZC, w=0.6, arrow='end')
    f.text((20, -6), '(ก) ปกติ az ≈ +1 g', size=2.8)
    f.line((48, 0), (88, 0), w=0.5)
    f.rect(53, 83, 0.5, 12, w=0.4, fill='#f5f5f5')
    f.line((68, 8), (68, -4), color=ZC, w=0.6, arrow='end')
    f.text((68, -9), '(ข) คว่ำ az ≈ −1 g', size=2.8)
    # hall + tach
    f.circle((115, 8), 9, w=0.4, fill='#ddd')
    f.rect(113, 117, 15, 17, color='#fff', w=0.3, fill='#fff')
    f.text((115, -6), '(ค) แถบสะท้อน + เครื่องวัดรอบ', size=2.8)
    f.rect(130, 138, 2, 16, w=0.4, fill='#fff')
    f.line((130, 9), (124, 9), color=PWR, w=0.4, dash='1,0.6', arrow='end')
    # DS18B20 + reference
    f.rect(150, 175, 0, 8, w=0.4, fill='#cfd8dc')
    f.rect(158, 162, 8, 11, color=SENS, w=0.3, fill=SENS, opacity=0.5)
    f.rect(165, 168, 8, 14, w=0.3, fill='#fff')
    f.text((162.5, -6), '(ง) เทอร์โมมิเตอร์อ้างอิงแนบข้าง', size=2.8)
    # multimeter
    f.rect(190, 205, 0, 18, w=0.4, fill='#fff9c4')
    f.text((197.5, 11), 'V', size=4, weight='bold')
    f.line((205, 6), (215, 6), color=PWR, w=0.5)
    f.text((200, -6), '(จ) มัลติมิเตอร์ที่ XT30', size=2.8)
    return f.svg(width_mm=w or 170)


def tilt_polar():
    f = Fig()
    R = 40
    zones = [(-60, 60, OK, 'ปกติ (az > +0.5 g)'), (60, 120, '#9e9e9e', 'คงสถานะเดิม'),
             (120, 240, PWR, 'กลับหัว (az < −0.5 g)'), (240, 300, '#9e9e9e', '')]
    for a0, a1, c, t in zones:
        f.arc_path((0, 0), R, 90 - a1, 90 - a0, color=c, w=0.3, fill=c, opacity=0.18, wedge=True)
    for a in range(0, 360, 45):
        r = math.radians(90 - a)
        f.line((0, 0), (R * math.cos(r), R * math.sin(r)), color=HID, w=0.2)
        f.text(((R + 6) * math.cos(r), (R + 6) * math.sin(r) - 1.3), '%d°' % a, size=3.2)
    f.rect(-7, 7, -3, 3, w=0.4, fill='#fff')
    f.line((0, 3), (0, 12), color=ZC, w=0.6, arrow='end')
    f.text((60, 20), 'เขียว: ปกติ (0–60°, 300–360°)', size=3, anchor='start', color=OK)
    f.text((60, 14), 'เทา: คงสถานะเดิม (60–120°, 240–300°)', size=3, anchor='start')
    f.text((60, 8), 'แดง: กลับหัว (120–240°)', size=3, anchor='start', color=PWR)
    f.text((60, 0), 'az = cos(มุมเอียง)  · เกณฑ์ ±0.5 g', size=3, anchor='start')
    f.text((60, -6), 'เปลี่ยนสถานะเมื่อค้าง > 0.3 s หลังผ่านเกณฑ์', size=3, anchor='start')
    f.text((60, -12), 'พลิก 180° ทันที → สลับประมาณ 0.51 s', size=3, anchor='start')
    f.text((0, -52), 'มุมเอียงรอบแกน Roll หรือ Pitch (ผลเหมือนกัน)', size=3)
    return f.svg(width_mm=150)


def safety_bands():
    f = Fig()
    rows = [
        ('รอบใบ (S2)', 0, 14000, [(0, 3000, '#e0e0e0', '< 3,000: ไม่จำกัดเลี้ยว'), (3000, 10000, '#ffe082', 'จำกัดเลี้ยว 690°/s'),
                                   (10000, 14000, '#a5d6a7', '≥ 10,000: เขียว พร้อมตี')], 'rpm'),
        ('ESC อาวุธ (S3)', 20, 110, [(20, 80, '#a5d6a7', 'ปกติ'), (80, 110, '#ce93d8', '> 80 °C: ม่วง + คันเร่งอาวุธ ≤ 70%')], '°C'),
        ('มอเตอร์ขับ (S4)', 20, 110, [(20, 90, '#a5d6a7', 'ปกติ'), (90, 110, '#ce93d8', '> 90 °C: ม่วง + เตือน')], '°C'),
        ('แบต (S5)', 9.5, 12.8, [(9.5, 10.5, '#ef9a9a', '< 10.5 V นาน 2 s: แดง'), (10.5, 10.9, '#e0e0e0', 'hysteresis'),
                                  (10.9, 12.8, '#a5d6a7', '> 10.9 V: หายเตือน')], 'V'),
    ]
    W = 120
    for i, (name, lo, hi, bands, unit) in enumerate(rows):
        y = -i * 20
        f.text((-3, y + 2), name, size=3.2, anchor='end', weight='bold')
        for a, b, c, t in bands:
            x0 = (a - lo) / (hi - lo) * W
            x1 = (b - lo) / (hi - lo) * W
            f.rect(x0, x1, y, y + 7, w=0.3, fill=c)
            f.text(((x0 + x1) / 2, y + 2.2), t, size=2.4)
            f.text((x0, y - 4), ('%g' % a), size=2.4)
        f.text((W, y - 4), '%g %s' % (hi, unit), size=2.4)
    return f.svg(width_mm=165)


def safety_zone():
    f = Fig()
    D.robot(f, 'top', internals=False, light=True)
    c = (0, G.ROTOR_Y)
    f.arc_path(c, G.TIP_R + 1, 0, 359.9, color=PWR, w=0.5)
    f.arc_path((0, 64), 120, 30, 150, color=PWR, w=0.3, fill=PWR, opacity=0.08, wedge=True, dash='2,1')
    f.text((0, 160), 'เขตอันตรายด้านหน้า: ห้ามมือ/หน้าอยู่หน้าใบ', size=3.4, color=PWR, weight='bold')
    f.leader((25, 110), (110, 120), 'วงหมุนใบ r 29.5 (+1 มม. เผื่อ)', color=PWR, size=3)
    f.text((0, -150), 'นอกสนาม: หมุดล็อกใบต้องเสียบเสมอ · ทดสอบ sensor: ถอดโรเตอร์ · ทดสอบอาวุธ: ในกล่องปิด', size=3.2)
    return f.svg(width_mm=150)
