"""Figures part 3: wiring, routing."""
import math

import draw as D
import geom as G
from draw import Fig, P, box, cyl, INK, HID, SENS, SIG, PWR, MOT, OK

W5 = '#d32f2f'      # 5 V
W33 = '#f57c00'     # 3.3 V
WG = '#212121'      # GND
WS = '#1565c0'      # signal


def _blk(f, x, y, w, h, title, sub='', color=INK, fill='#fff'):
    f.rect(x, x + w, y, y + h, color=color, w=0.5, fill=fill)
    f.text((x + w / 2, y + h - 5), title, size=3.4, weight='bold', color=color)
    if sub:
        f.text((x + w / 2, y + h - 9.5), sub, size=2.6, color=HID)


def wiring_block():
    f = Fig()
    _blk(f, 0, 60, 38, 16, 'แบต 3S LiPo', '11.1 V, XT30')
    _blk(f, 50, 60, 38, 16, 'Removable link', '+ ฟิวส์ 30 A')
    _blk(f, 105, 78, 40, 16, 'ESC Skywalker 40A', 'มี BEC 5 V')
    _blk(f, 105, 52, 40, 16, 'DRV8871 ×2', 'มอเตอร์ขับ ×4')
    _blk(f, 160, 78, 34, 16, 'D3536', 'มอเตอร์อาวุธ')
    _blk(f, 160, 52, 34, 16, 'JGA25-370 ×4', '')
    _blk(f, 80, 0, 60, 34, 'ESP32 DevKit V1', 'MCU ตัวเดียวของรถ', color=SIG, fill='#e3f2fd')
    for i, (t, s) in enumerate([('S1 IMU', 'I2C'), ('S2 Hall', 'GPIO int'), ('S3/S4 DS18B20', '1-Wire'), ('S5 Divider', 'ADC')]):
        _blk(f, i * 42 - 10, -45, 36, 14, t, s, color=SENS)
        f.line((i * 42 + 8, -31), (95 + i * 10, 0), color=WS, w=0.5, arrow='end')
    f.line((38, 68), (50, 68), color=PWR, w=0.9, arrow='end')
    f.line((88, 68), (97, 68), color=PWR, w=0.9)
    f.line((97, 86), (97, 60), color=PWR, w=0.9)
    f.line((97, 86), (105, 86), color=PWR, w=0.9, arrow='end')
    f.line((97, 60), (105, 60), color=PWR, w=0.9, arrow='end')
    f.line((145, 86), (160, 86), color=MOT, w=0.9, arrow='end')
    f.line((145, 60), (160, 60), color=MOT, w=0.9, arrow='end')
    f.line((92, 68), (92, 34), color=PWR, w=0.4, dash='1.5,1', arrow='end')
    f.text((90, 46), 'VBAT → S5', size=2.6, anchor='end', color=PWR)
    f.line((125, 78), (125, 34), color=W5, w=0.5, arrow='end')
    f.text((127, 40), 'BEC 5 V → VIN', size=2.6, anchor='start', color=W5)
    f.line((115, 34), (115, 52), color=WS, w=0.5, arrow='end')
    f.text((113, 43), 'PWM IN1/IN2', size=2.6, anchor='end', color=WS)
    f.poly([(140, 30), (151, 30), (151, 82), (145, 82)], closed=False, color=WS, w=0.4, dash='1.5,1', arrow='end')
    f.text((153, 40), 'GPIO13 50 Hz', size=2.6, anchor='start', color=WS)
    _blk(f, 160, 10, 40, 16, 'จอย PS4 (Bluepad32)', 'บลูทูธ')
    f.line((140, 18), (160, 18), color=SIG, w=0.5, arrow='both', dash='2,1')
    _blk(f, 160, -18, 40, 16, 'โน้ตบุ๊ก (บนโต๊ะเท่านั้น)', 'USB Serial 115200')
    f.line((140, 6), (160, -10), color=SIG, w=0.5, arrow='both', dash='2,1')
    f.text((95, -55), 'Sensor → สาย/ขั้วต่อ → ESP32 (MCU) → ESC/DRV และจอย · V4 ไม่มี Main Computer แยก', size=3)
    return f.svg(width_mm=170)


def wiring_schematic():
    f = Fig()
    # ESP32 pin column
    pins = ['VIN', '3V3', 'GND', 'D21 SDA', 'D22 SCL', 'D32', 'D4', 'D34', 'D13', 'D25', 'D26', 'D27', 'D14']
    x0 = 110
    f.rect(x0, x0 + 34, -len(pins) * 7 - 3, 10, color=SIG, w=0.6, fill='#e3f2fd')
    f.text((x0 + 17, 3), 'ESP32 DevKit V1', size=3.4, weight='bold', color=SIG)
    py = {}
    for i, p in enumerate(pins):
        y = -i * 7 - 4
        py[p] = y
        f.circle((x0, y), 0.9, fill='#fff', w=0.4)
        f.text((x0 + 3, y - 1.2), p, size=3, anchor='start')
    right = {'D13': 'สัญญาณ ESC อาวุธ', 'D25': 'DRV8871 L IN1', 'D26': 'DRV8871 L IN2',
             'D27': 'DRV8871 R IN1', 'D14': 'DRV8871 R IN2', 'VIN': '← BEC 5 V จาก ESC'}
    for p, t in right.items():
        f.line((x0 + 34, py[p]), (x0 + 42, py[p]), color=HID, w=0.4)
        f.text((x0 + 44, py[p] - 1.2), t, size=2.8, anchor='start', color=HID)

    def wire(a, pin, col, label=None, bus_x=None):
        bx = bus_x
        f.line(a, (bx, a[1]), color=col, w=0.5)
        f.line((bx, a[1]), (bx, py[pin]), color=col, w=0.5)
        f.line((bx, py[pin]), (x0, py[pin]), color=col, w=0.5)
        f.circle((bx, a[1]), 0.6, color=col, fill=col)

    # S1 GY-521
    _blk(f, 0, -36, 34, 30, 'S1 GY-521', 'MPU6050, addr 0x68', color=SENS)
    for i, (n, pin, col) in enumerate([('VCC', '3V3', W33), ('GND', 'GND', WG), ('SCL', 'D22 SCL', WS), ('SDA', 'D21 SDA', WS)]):
        y = -i * 4 - 20
        f.text((33, y - 1), n, size=2.6, anchor='end')
        wire((34, y), pin, col, bus_x=60 + i * 3)
    f.text((17, -40), 'AD0, INT, XDA, XCL: ไม่ต่อ', size=2.4, color=HID)
    # S2 A3144
    _blk(f, 0, -72, 34, 26, 'S2 A3144', 'หน้าแบนหันหาตัว ขาซ้าย→ขวา', color=SENS)
    for i, (n, pin, col) in enumerate([('1 VCC', 'VIN', W5), ('2 GND', 'GND', WG), ('3 OUT', 'D32', WS)]):
        y = -60 - i * 4
        f.text((33, y - 1), n, size=2.6, anchor='end')
        wire((34, y), pin, col, bus_x=75 + i * 3)
    f.text((17, -76), '+ R 10k จาก D32 → 3V3 เท่านั้น (ห้าม 5 V)', size=2.4, color=PWR)
    # S3/S4 DS18B20 bus
    _blk(f, 0, -112, 34, 28, 'S3 + S4 DS18B20', 'หน้าแบนหันหาตัว · ต่อขนาน', color=SENS)
    for i, (n, pin, col) in enumerate([('3 VDD', '3V3', W33), ('1 GND', 'GND', WG), ('2 DQ', 'D4', WS)]):
        y = -100 - i * 4
        f.text((33, y - 1), n, size=2.6, anchor='end')
        wire((34, y), pin, col, bus_x=90 + i * 3)
    f.text((17, -116), '+ R 4.7k จาก DQ → 3V3 (ที่ฝั่ง ESP32)', size=2.4, color=PWR)
    # S5
    _blk(f, 0, -136, 34, 18, 'S5 Divider', '100k / 22k', color=SENS)
    f.text((33, -133), 'จุดกลาง', size=2.6, anchor='end')
    wire((34, -132), 'D34', WS, bus_x=103)
    f.text((17, -140), 'ปลาย 22k → GND, ปลาย 100k → VBAT+', size=2.4, color=HID)
    # legend
    for i, (c, t) in enumerate([(W5, '5 V'), (W33, '3.3 V'), (WG, 'GND'), (WS, 'สัญญาณ')]):
        f.line((i * 28, 22), (i * 28 + 8, 22), color=c, w=0.9)
        f.text((i * 28 + 10, 20.8), t, size=2.8, anchor='start')
    return f.svg(width_mm=170)


def _route(f, view, pts, col, w=0.7, dash=None):
    f.poly([P(view, *p) for p in pts], closed=False, color=col, w=w, dash=dash)


def routing(view='top', width_mm=165):
    f = Fig()
    D.robot(f, view, internals=True, light=True)
    D.sensors(f, view)
    if view == 'top':
        f.rect(-88, 0, -110, 62, color=MOT, w=0.0, fill=MOT, opacity=0.05)
        f.rect(0, 88, -110, 62, color=SIG, w=0.0, fill=SIG, opacity=0.05)
        f.text((-44, -100), 'โซนกำลังอาวุธ', size=3.2, color=MOT)
        f.text((44, -100), 'โซนลอจิก', size=3.2, color=SIG)
    for k, r in G.MOTOR_ROUTES.items():
        _route(f, view, r, MOT, w=0.9)
    for k, r in G.POWER_ROUTES.items():
        _route(f, view, r, PWR, w=0.9)
    for k, r in G.ROUTES.items():
        _route(f, view, r, SIG, w=0.6, dash='2,0.8')
    if view == 'top':
        labels = {'S2': ((25, 20), (120, 45)), 'S1': ((3, -48), (-125, -85)), 'S3': ((-20, -20), (-125, -30)),
                  'S4': ((-22, 0), (-125, 10)), 'S5': ((62, -32), (120, -60))}
        for k, (p, q) in labels.items():
            f.leader(p, q, 'สายสัญญาณ ' + k, color=SIG, size=3)
        f.leader((-50, 20), (-125, 60), 'สายเฟส D3536 (ชิดผนังซ้าย x ≈ −50)', color=MOT, size=3)
        f.leader((-45, -75), (-125, -120), 'สายมอเตอร์ขับ (ลอดใต้มอเตอร์ z ≈ 10)', color=MOT, size=3)
        f.leader((63, -15), (120, -10), 'สายไฟหลัง link', color=PWR, size=3)
        f.text((0, 150), 'ด้านหน้า ↑', size=4, weight='bold')
    else:
        f.line((-130, 58), (135, 58), color=HID, w=0.3, dash='3,1')
        f.text((-128, 59.5), 'ใต้ฝา z 58', size=3, anchor='start', color=HID)
        f.text((0, -10), 'สายสัญญาณวิ่งที่ z ≈ 46–51 เหนือมอเตอร์ขับ / ESP32 · สายมอเตอร์ขับวิ่งบนพื้น z ≈ 10', size=3.2)
    y0 = -150 if view == 'top' else -20
    for i, (c, t, d) in enumerate([(SIG, 'สัญญาณ sensor', '2,0.8'), (PWR, 'ไฟเลี้ยง (Power)', None), (MOT, 'ไฟมอเตอร์', None)]):
        f.line((-125 + i * 85, y0), (-113 + i * 85, y0), color=c, w=0.9, dash=d)
        f.text((-111 + i * 85, y0 - 1.2), t, size=3, anchor='start')
    return f.svg(width_mm=width_mm)
