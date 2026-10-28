"""Assembly guide for V4 R5-AL (Thai, A4 PDF): build order after the frame parts are cut, one page per step.

Images, part lists and the fastener lists all come from the CAD (importing build_r5_al rebuilds it), so the guide
always matches the drawing. The HTML is printed to PDF with headless Google Chrome (Thai shaping needs a browser).
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r5_al/guide/make_guide.py
Writes robot_v4/print_revision/r5_al/guide/V4_R5_AL_ASSEMBLY_GUIDE_TH.pdf (+ .html and img/).
"""
import collections
import html
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
R5 = HERE.parent
sys.path.insert(0, str(R5))
import build_r5_al as B  # noqa: E402  (rebuilds the CAD and its checks)

IMG = HERE / 'img'
IMG.mkdir(exist_ok=True)
REP = json.loads((R5 / 'output' / 'build_report.json').read_text())
CALC = json.loads((R5 / 'calc' / 'r5_al_calc.json').read_text()) if (R5 / 'calc' / 'r5_al_calc.json').exists() else None
BODIES = [b for b in B.bodies if b.material != 'keepout']

# ---------------------------------------------------------------- what each body is called in the guide (prefix -> Thai)
LABEL = [
    ('Frame_Rail_', 'คานข้าง ท่ออะลูมิเนียม 12.7×12.7×2.0'), ('Frame_Rear_Cross', 'คานขวางท้าย'),
    ('Frame_Bulkhead_Cross', 'คานขวางกลาง (หลังแผ่นตั้งอาวุธ)'), ('Frame_Bulkhead_Stub_', 'ท่อสั้นปลายคานขวางกลาง'),
    ('Frame_Motor_Cross', 'คานขวางใต้มอเตอร์ขับ'), ('Frame_Clamp_Bar', 'คานกดมอเตอร์ขับ'),
    ('Frame_Hinge_Beam_', 'คานบานพับ wedgelet'), ('Frame_Midwall_Stub_', 'ท่อสั้นค้ำกลางผนัง'),
    ('Cleat_', 'ฉาก cleat 20×20×2 ที่มุมโครง'), ('Tab_Wall_', 'แผ่นยึดผนัง (อะลูมิเนียม 2 มม.)'),
    ('Tab_Battery_Strap_', 'แผ่นร้อยสายรัดแบต'), ('Skin_PC2_Floor', 'พื้น PC 2 มม.'), ('Rear_Skid', 'skid ท้าย UHMW/HDPE 3 มม.'),
    ('Tab_Drive_Motor_', 'แผ่นยึดมอเตอร์ขับ'), ('JGA25_370', 'มอเตอร์ขับ ORTECH JB361 12V 400 rpm'),
    ('Pad_Drive_Motor', 'แผ่นยาง EPDM รองมอเตอร์ขับ'), ('Clamp_Drive_Motor_Post', 'เสายึดคานกดมอเตอร์'),
    ('Wheel_Hub_PETG', 'ดุมล้อพิมพ์ PETG 100%'), ('Wheel_ORing', 'โอริง NBR P52 (ยางล้อ)'),
    ('Rotor_Pipe', 'ท่อเหล็กดำ 3/4" ยาว 40 (ดุมอาวุธ)'), ('Beater_Disc', 'จานอาวุธเหล็ก 6 มม.'), ('Bearing_608', 'ลูกปืน 608ZZ'),
    ('Belt_Flange_PETG', 'ปีกกันสายพานหลุด PETG'), ('Upright_6061', 'แผ่นตั้ง 6061 หนา 6 มม. (ต๊าป M4)'),
    ('Weapon_Top_Brace', 'brace บน 6061 3 มม.'), ('Weapon_Bottom_Brace', 'brace ล่าง 6061 3 มม.'),
    ('Spacer_PETG', 'spacer เพลา PETG'), ('Dead_Shaft', 'สกรู M8×80 เกรด 12.9 (เพลาอาวุธ)'), ('Weapon_Axle', 'น็อตล็อก/แหวน M8'),
    ('Rung_Al_Bar', 'แท่งอะลูมิเนียมตัน 12.7 ยาว 13'), ('Upright_Riser_Angle', 'ฉากตั้ง 6061 3 มม.'),
    ('Upright_Foot_Angle', 'ฉากฐาน 6061 3 มม. (3 ขา)'), ('Nose_Skid', 'skid หน้า UHMW/HDPE 3 มม.'),
    ('Weapon_Motor_Mount', 'ฐานมอเตอร์อาวุธ 6061 3 มม.'), ('Motor_Mount_Angle', 'ฉากยึดฐานมอเตอร์อาวุธ'),
    ('D3536', 'มอเตอร์อาวุธ SURPASS C3536 1300KV (หรือ D3536)'), ('Motor_Pulley_PETG', 'พูลเลย์มอเตอร์ PETG/ASA'),
    ('Belt_', 'สายพานกลม PU 5 มม.'), ('Hall_Post', 'เสายึดเซนเซอร์ hall (PETG)'), ('Hall_A3144', 'เซนเซอร์ hall A3144'),
    ('Hinge_Base', 'แผ่นฐานบานพับ 5052 2 มม. (พับแผ่นบานพับขึ้น)'), ('Tab_Hinge_', 'แผ่นบานพับ (พับจากแผ่นฐาน)'),
    ('Wedgelet_DIY', 'wedgelet เหล็ก 1.6 มม.'), ('Hinge_Bolt', 'สกรูสเตนเลส M3×55 (หมุดบานพับ)'),
    ('Standoff_Nylon', 'เสาไนลอน M3'), ('Electronics_Plate', 'แผ่นวางบอร์ด PC 2 มม.'), ('ESP32', 'ESP32'),
    ('DRV8871', 'บอร์ดขับมอเตอร์ DRV8871'), ('Skywalker', 'ESC Skywalker 40A'), ('IMU_GY521', 'IMU GY-521'),
    ('IMU_Foam', 'ฟองน้ำรอง IMU'), ('3S_650mAh', 'แบต LiPo 3S 650 mAh (ขนาด 57×31×24)'), ('Power_Switch', 'สวิตช์ตัดไฟ (link เสียบ)'),
    ('Skin_PC2_Side_Wall', 'ผนังข้าง PC 2 มม.'), ('Skin_PC2_Rear_Wall', 'ผนังท้าย PC 2 มม.'),
    ('Skirt_Bracket', 'ขายึด skirt 5052 1.5 มม.'), ('Skirt_Plate', 'skirt อะลูมิเนียม 0.8 มม.'),
    ('Skin_PC2_Lid', 'ฝา PC 2 มม.'), ('Lid_Angle', 'ฉากยึดฝา 12×12×1.5'),
]


def label(name):
    for p, t in LABEL:
        if name.startswith(p):
            return t
    return name


# ---------------------------------------------------------------- the build order (prefixes of the bodies added in each step)
STEPS = [
    {'key': 'frame', 'title': 'ขั้น 1 — ประกอบโครงอะลูมิเนียม',
     'prefixes': ('Frame_', 'Cleat_', 'Tab_Wall_', 'Tab_Battery_Strap_'),
     'how': ['ย้ำ rivnut ทุกรูในท่อ<b>ก่อน</b>ประกอบ (รู Ø5 = rivnut M3, รู Ø6 = rivnut M4) ตำแหน่งดูตารางจุดยึดท้ายเล่ม',
             'วางคานข้างสองเส้นบนโต๊ะเรียบ ประกบคานขวางท้าย คานขวางกลาง และคานขวางใต้มอเตอร์ แล้วยึดฉาก cleat ที่มุมด้านใน ขันแค่พอติดก่อน',
             'ยึดท่อสั้นปลายคานขวางกลาง ท่อสั้นค้ำกลางผนัง และคานบานพับ ด้วยฉาก cleat ด้านนอก',
             'วัดเส้นทแยงมุมของโครงทั้งสองเส้นให้ยาวเท่ากัน (ต่างกันไม่เกิน 1 มม.) แล้วจึงขันน็อตทุกตัวให้แน่น',
             'ยึดแผ่นยึดผนัง 6 ชิ้น (ท้าย/หน้า/กลาง ข้างละ 3) และแผ่นร้อยสายรัดแบต 4 ชิ้น'],
     'check': ['โครงวางราบบนโต๊ะ ไม่โยก', 'เส้นทแยงมุมเท่ากัน', 'ฝั่งซ้ายคานขวางกลางไม่มีฉาก cleat ด้านหลัง (ที่ตรงนั้นเป็นของฐานมอเตอร์อาวุธ) ฉากฐานขั้น 4 จะยึดมุมนั้นแทน']},
    {'key': 'floor', 'title': 'ขั้น 2 — พื้น PC และ skid ท้าย',
     'prefixes': ('Skin_PC2_Floor', 'Rear_Skid'),
     'how': ['ทาบพื้น PC ใต้โครง ให้ช่องเว้าล้อตรงกับแนวเพลาหน้า (y 18)',
             'ขันสกรูหัวจม M3 สเตนเลสจากใต้พื้นเข้า rivnut ในคานข้าง คานขวางท้าย และคานขวางกลาง (10 จุด)',
             'ยึด skid ท้ายใต้พื้นด้านหลังด้วยสกรูหัวจม M3 2 ตัว'],
     'check': ['หัวสกรูจมเรียบกับผิวพื้น ไม่ยื่นลงพื้นสนาม', 'skid ท้ายสูงจากพื้นประมาณ 1 มม.']},
    {'key': 'drive', 'title': 'ขั้น 3 — ระบบขับ (มอเตอร์ขับ 2 ตัว + ล้อ)',
     'prefixes': ('Tab_Drive_Motor_', 'JGA25_370', 'Pad_Drive_Motor', 'Clamp_Drive_Motor_Post', 'Wheel_Hub_PETG', 'Wheel_ORing'),
     'how': ['ยึดแผ่นยึดมอเตอร์ขับบนคานข้างด้วย M3 เข้า rivnut (ข้างละ 2 ตัว)',
             'วางแผ่นยาง EPDM บนคานขวางใต้มอเตอร์ แล้วสอดมอเตอร์ JB361 ให้เพลาลอดรูแผ่นยึดมอเตอร์ ขัน M3 สองตัวเข้าหน้ามอเตอร์',
             'ยึดเสายึดคานกดมอเตอร์กับหน้าคานขวางใต้มอเตอร์ และหน้าคานกด (M3 4 ตัว) ให้คานกดกดแผ่นยางด้านบนของมอเตอร์ไว้',
             'สวมโอริง P52 สองวงลงร่องดุมล้อ PETG แล้วสวมดุมเข้าเพลารูป D ขันสกรูตัวหนอน M3'],
     'check': ['หมุนล้อด้วยมือ ไม่ติดพื้นหรือผนัง', 'ต่อแบตเข้ามอเตอร์ตรง ๆ ทีละตัว ล้อหมุนทั้งสองทิศ', 'ล้อทั้งสองหมุนไปข้างหน้าเมื่อเลื่อนคันโยกไปข้างหน้า (ถ้าไม่ใช่ ให้สลับสายมอเตอร์)']},
    {'key': 'rotor', 'title': 'ขั้น 4 — ทำดุมอาวุธ (งานเชื่อมจุดเดียวของรถ)', 'standalone': True,
     'prefixes': ('Rotor_Pipe', 'Beater_Disc', 'Bearing_608', 'Belt_Flange_PETG'),
     'how': ['ตัดท่อเหล็กดำ 3/4" ยาว 40.0 มม. ปลายตั้งฉาก (ใช้ฉากเหล็กเช็ก) <b>วัดรูในก่อน: ต้องได้ 22.0–22.15 มม.</b>',
             'ร้อยแท่ง Ø8 ผ่านรูกลางจานทั้งสองใบและท่อ ให้ทุกชิ้นอยู่ในแนวเดียวกัน แล้วเชื่อมจุด (tack) 3 จุดต่อจาน สลับข้าง',
             'เชื่อมรอบท่อทั้งสองข้าง (ลวด 2.0 มม.) <b>ห้ามใส่ลูกปืนตอนเชื่อม</b> ปล่อยให้เย็นเองช้า ๆ',
             'วัดรูในท่ออีกครั้ง เคาะสะเก็ดออก แล้วอัดลูกปืน 608 เข้าปลายท่อทั้งสองข้างพร้อมน้ำยาล็อกลูกปืน (Loctite 648) ทิ้งไว้ 24 ชม.',
             'สวมปีกกันสายพาน PETG สองวงบนท่อ ห่างกัน 6 มม. ตรงกลาง (ใช้กาว CA ยึด)',
             'ถ่วงสมดุล: วางดุมบนแกน Ø8 ที่พาดบนใบมีดสองแผ่น ด้านที่หนักจะหมุนลงล่าง เจาะเนื้อออกที่ด้านนั้นจนดุมหยุดได้ทุกมุม'],
     'check': ['จานทั้งสองขนาน ไม่ส่าย (หมุนด้วยมือดูขอบฟัน)', 'ลูกปืนหมุนลื่น ไม่ฝืดหลังเชื่อม', 'ดุมถ่วงสมดุลแล้ว']},
    {'key': 'weapon', 'title': 'ขั้น 5 — โมดูลอาวุธ (แผ่นตั้ง + ดุม) ยึดเข้าโครง',
     'prefixes': ('Upright_6061', 'Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Spacer_PETG', 'Dead_Shaft', 'Weapon_Axle',
                  'Rung_Al_Bar', 'Upright_Riser_Angle', 'Upright_Foot_Angle', 'Nose_Skid',
                  'Rotor_Pipe', 'Beater_Disc', 'Bearing_608', 'Belt_Flange_PETG'),
     'how': ['ต๊าปเกลียว M4 ในแผ่นตั้ง 6061 ทั้งสองแผ่น (ดอกนำ 3.3 มม.) ข้างละ 3 รู ตามตำแหน่งในตารางจุดยึด',
             'ประกอบดุมอาวุธระหว่างแผ่นตั้ง: spacer PETG ด้านนอก → ลูกปืน → spacer ใน → ลูกปืน → spacer ด้านนอก ร้อยสกรู M8×80 เกรด 12.9',
             '<b>ขันน็อตล็อก M8 แค่พอแน่น</b> (spacer เป็น PETG ถ้าขันแน่นเกินจะยุบ) ดุมต้องหมุนลื่น',
             'ยึด brace บนและ brace ล่าง แล้ววาง skid หน้าใต้ brace ล่าง',
             'ยึดเข้าโครงข้างละ 3 × M4 เกรด 8.8: (1) ผ่านคานข้าง → แท่งตัน → แผ่นตั้ง (2) ฉากตั้ง → แผ่นตั้ง แล้วยึดขาฉากตั้งลงบนคานข้าง (M4 rivnut) (3) ฉากฐาน → แผ่นตั้ง',
             'ยึดขาอีกสองข้างของฉากฐาน: ขาหน้าคานขวางกลาง (M3 2 ตัว) และขาที่ติดคานข้าง (M3 1 ตัว)'],
     'check': ['หมุนดุมด้วยมือ ฟันห่างจากพื้น 2.5 มม. และไม่โดนโครงหรือ skid', 'ขยับแผ่นตั้งด้วยมือแรง ๆ ต้องไม่โยก',
               'ขันน็อต M4 เข้าเกลียวอะลูมิเนียมประมาณ 2 N·m (ขันแน่นมือ ห้ามใช้แรงเกิน เกลียวจะรูด)']},
    {'key': 'wmotor', 'title': 'ขั้น 6 — มอเตอร์อาวุธ สายพาน และเซนเซอร์รอบ',
     'prefixes': ('Weapon_Motor_Mount', 'Motor_Mount_Angle', 'D3536', 'Motor_Pulley_PETG', 'Belt_', 'Hall_Post', 'Hall_A3144'),
     'how': ['ยึดมอเตอร์อาวุธเข้าฐานมอเตอร์ 6061 ด้วย M3 4 ตัว (ระยะรู 19/25 มม. ตามรุ่น SURPASS)',
             'วางฐานมอเตอร์บนฉากยึดสองชิ้น ขันฉากลงบนคานข้าง (M3 เข้า rivnut) แล้วขันฐานมอเตอร์เข้าฉาก (M3 + น็อต)',
             'สวมพูลเลย์ PETG เข้าเพลามอเตอร์ ตะไบผิวเพลาให้เรียบตรงจุดที่สกรูตัวหนอน M3 กดลง',
             'คล้องสายพานกลม PU จากพูลเลย์ไปท่อดุมอาวุธ (ระหว่างปีกกันหลุดสองวง) สายพานควรตึงพอดี กดลงได้ประมาณ 3 มม.',
             'ยึดเสาเซนเซอร์ hall บนคานขวางกลาง ติดแม่เหล็กเล็กที่จาน แล้วติดเซนเซอร์ A3144 ห่างแม่เหล็กประมาณ 2 มม.'],
     'check': ['หมุนดุมด้วยมือ สายพานพามอเตอร์หมุนตาม ไม่หลุดร่อง', 'สายมอเตอร์อาวุธไม่พาดผ่านทางสายพานหรือจาน']},
    {'key': 'wedge', 'title': 'ขั้น 7 — แผ่นบานพับและ wedgelet',
     'prefixes': ('Hinge_Base', 'Tab_Hinge_', 'Wedgelet_DIY', 'Hinge_Bolt'),
     'how': ['พับแผ่นบานพับ 3 แผ่นขึ้น 90° จากแผ่นฐาน 5052 (ข้างละ 1 ชิ้น) แล้วเจาะรูหมุด Ø3.2 สามแผ่นพร้อมกัน ใช้สกรู M3 ยาวร้อยไว้ให้รูตรงกัน',
             'ขันแผ่นฐานขึ้นใต้คานบานพับด้วยสกรูหัวจม M3 3 ตัว เข้า rivnut',
             'สอด wedgelet ให้หูอยู่ระหว่างแผ่นบานพับ แล้วร้อยสกรูสเตนเลส M3×55 ปิดด้วยน็อตล็อก ขันแค่พอให้ wedgelet แกว่งได้'],
     'check': ['wedgelet แกว่งขึ้นได้ประมาณ 15° และลงแตะพื้นเองด้วยน้ำหนักตัว', 'ปลาย wedgelet แตะพื้น ไม่ค้างลอย']},
    {'key': 'elec', 'title': 'ขั้น 8 — แบต อิเล็กทรอนิกส์ และการเดินสาย',
     'prefixes': ('Standoff_Nylon', 'Electronics_Plate', 'ESP32', 'DRV8871', 'Skywalker', 'IMU_', '3S_650mAh', 'Power_Switch'),
     'how': ['วางแบตระหว่างคานข้างด้านหลัง ร้อยสายรัดผ่านแผ่นร้อยสายรัดแบต (ต้องดึงแบตออกได้โดยไม่ถอดอย่างอื่น)',
             'ตั้งเสาไนลอน 4 ต้นบนคานข้าง แล้ววางแผ่นวางบอร์ด PC ขันสกรูไนลอน M3',
             'ติด ESP32, DRV8871 สองตัว และ ESC บนแผ่นวางบอร์ดด้วยเทปโฟมสองหน้า รัดเสริมด้วยเคเบิ้ลไทร์ ติด IMU บนฟองน้ำ 1 มม.',
             'เดินสายตาม firmware/v4_controller (ขา ESP32 ดูในโค้ด) ใส่ฟิวส์และสวิตช์ตัดไฟระหว่างแบตกับวงจรทั้งหมด',
             'ใช้ขั้วต่อ XT30 ทุกจุดที่ต้องถอดได้ มัดสายให้ห่างจากล้อ สายพาน และจานอาวุธ'],
     'check': ['ถอดสวิตช์ตัดไฟแล้วต้องไม่มีไฟเข้าวงจรใดเลย', 'วัดว่าขั้วบวกกับลบไม่ลัดกันก่อนเสียบแบตครั้งแรก',
               'ESP32 กับ IMU ติดแน่น ไม่สั่นตอนเคาะโครง']},
    {'key': 'walls', 'title': 'ขั้น 9 — ผนังข้าง ผนังท้าย และ skirt',
     'prefixes': ('Skin_PC2_Side_Wall', 'Skin_PC2_Rear_Wall', 'Skirt_Bracket', 'Skirt_Plate'),
     'how': ['ขันผนังข้าง PC เข้าแผ่นยึดผนัง 3 จุดต่อข้าง (M3 สเตนเลส + น็อตล็อก) แผ่นยึดตรงกลางคือจุดที่แก้ปัญหาผนังแอ่น',
             'ขันผนังท้าย PC เข้าคานขวางท้าย (M3 3 ตัวเข้า rivnut)',
             'ยึดขายึด skirt 11 ชิ้นนอกผนัง แล้วยึด skirt อะลูมิเนียม 0.8 มม. ด้วย M2 ให้ขอบล่างสูงจากพื้นประมาณ 0.5 มม.'],
     'check': ['วาง skirt บนพื้นเรียบ ขอบล่างห่างพื้น 0–1 มม. ตลอดแนว', 'กดผนังข้างตรงกลางด้วยนิ้ว ต้องไม่ยุบมาก']},
    {'key': 'lid', 'title': 'ขั้น 10 — ฝา และตรวจก่อนลงสนาม',
     'prefixes': ('Skin_PC2_Lid', 'Lid_Angle'),
     'how': ['ยึดฉากยึดฝา 4 ชิ้นที่มุมด้านในของผนังข้าง (M3 + น็อตล็อก)',
             'วางฝา PC ให้ล้อโผล่ช่องบนฝา ขันสกรู M3 4 ตัวลงฉากยึดฝา',
             'ชั่งน้ำหนักทั้งคันพร้อมแบต <b>ต้องไม่เกิน 2,000 g</b> (CAD ประมาณ %(mass).0f g)'],
     'check': ['คว่ำรถแล้วขับได้ (ล้อโผล่ทั้งบนและล่าง)', 'น้ำหนักไม่เกิน 2 กก.', 'ทำการทดสอบในหัวข้อถัดไปครบทุกข้อก่อนลงสนาม']},
]


# ---------------------------------------------------------------- renders
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

COL = {'al_tube': '#9fb3c8', 'al_tab': '#5d7ea6', 'al_bar': '#5d7ea6', '6061': '#5d7ea6', 'petg_print': '#f39c12',
       'steel_pipe': '#444', 'steel': '#555', 'steel_wedgelet': '#8e6b3f', 'steel_bolt': '#333', 'PC': '#56b2b0',
       'UHMW': '#dddddd', 'EPDM': '#222', 'NBR': '#222', 'purchased': '#2e4a8b', 'battery_650': '#2e4a8b', 'belt': '#222',
       'al5052_bracket': '#7f8c8d', 'nylon_standoff': '#eee', 'TPU95A': '#56b2b0', 'foam': '#ccc'}
TESS = {}


def tris(b):
    if b.name not in TESS:
        v, t = b.shape.tessellate(0.6, 0.4)
        TESS[b.name] = (np.array([p.toTuple() for p in v]), np.array(t)) if t else None
    return TESS[b.name]


def step_of(name):
    for i, st in enumerate(STEPS):
        if st.get('standalone'):
            continue
        if name.startswith(st['prefixes']):
            return i
    return None


def render(path, show, highlight, view=(28, -55), lims=None):
    fig = plt.figure(figsize=(8, 5.2), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    for b in show:
        d = tris(b)
        if d is None:
            continue
        v, t = d
        hi = b.name in highlight
        ax.add_collection3d(Poly3DCollection(v[t], facecolors=COL.get(b.material, '#7a8ca6') if hi else '#d5d8dc',
                                             edgecolors='none', alpha=(0.35 if (b.material == 'PC' and not hi) else (0.95 if hi else 0.55))))
    lx, ly, lz = lims or ((-105, 105), (-125, 130), (0, 70))
    ax.set_xlim(*lx); ax.set_ylim(*ly); ax.set_zlim(*lz)
    ax.set_box_aspect((lx[1] - lx[0], ly[1] - ly[0], lz[1] - lz[0]))
    ax.view_init(elev=view[0], azim=view[1]); ax.set_axis_off()
    fig.savefig(path, dpi=200); plt.close(fig)
    from PIL import Image, ImageChops                           # crop the wide empty margin mplot3d leaves around the model
    im = Image.open(path).convert('RGB')
    box = ImageChops.difference(im, Image.new('RGB', im.size, (255, 255, 255))).getbbox()
    if box:
        pad = 20
        im.crop((max(box[0] - pad, 0), max(box[1] - pad, 0), min(box[2] + pad, im.width), min(box[3] + pad, im.height))).save(path)


done = set()
for i, st in enumerate(STEPS):
    if st.get('standalone'):
        new = [b for b in BODIES if b.name.startswith(st['prefixes'])]
        st['parts'] = new
        render(IMG / ('%02d_%s.png' % (i + 1, st['key'])), new, {b.name for b in new}, view=(18, -60), lims=((-40, 40), (55, 135), (-8, 72)))
        continue
    new = [b for b in BODIES if step_of(b.name) == i]
    st['parts'] = new
    done |= {b.name for b in new}
    show = [b for b in BODIES if b.name in done]
    render(IMG / ('%02d_%s.png' % (i + 1, st['key'])), show, {b.name for b in new})
render(IMG / '00_cover.png', BODIES, {b.name for b in BODIES})
render(IMG / '00_cover_rear.png', BODIES, {b.name for b in BODIES}, view=(30, 125))

# ---------------------------------------------------------------- HTML
FIX = REP['fixings']
e = html.escape


WHAT_TH = {
    'corner cleat': 'ฉาก cleat ที่มุมโครง', 'wall tab -> tube top': 'แผ่นยึดผนัง → บนท่อ',
    'clamp post -> clamp bar front face': 'เสายึด → หน้าคานกดมอเตอร์', 'clamp post -> motor cross front face': 'เสายึด → หน้าคานขวางใต้มอเตอร์',
    'drive-motor tab -> rail top': 'แผ่นยึดมอเตอร์ขับ → บนคานข้าง', 'foot angle -> bulkhead cross front face': 'ฉากฐาน (ขา A) → หน้าคานขวางกลาง',
    'foot angle leg C -> rail inner face': 'ฉากฐาน (ขา C) → ด้านในคานข้าง', 'riser angle -> rail top': 'ฉากตั้ง (ขานอน) → บนคานข้าง',
    'upright -> foot angle': 'แผ่นตั้ง → ฉากฐาน (ขา B)', 'upright -> riser angle': 'แผ่นตั้ง → ฉากตั้ง',
    'upright -> rung bar -> rail': 'แผ่นตั้ง → แท่งตัน → คานข้าง', 'hinge base -> hinge beam bottom': 'แผ่นฐานบานพับ → ใต้คานบานพับ',
    'battery-strap tab -> rail inner face': 'แผ่นร้อยสายรัดแบต → ด้านในคานข้าง', 'electronics plate -> nylon standoff': 'แผ่นวางบอร์ด → เสาไนลอน',
    'PC floor -> bulkhead cross bottom': 'พื้น PC → ใต้คานขวางกลาง', 'PC floor -> rail bottom': 'พื้น PC → ใต้คานข้าง',
    'PC floor -> rear cross bottom': 'พื้น PC → ใต้คานขวางท้าย', 'PC rear wall -> rear cross': 'ผนังท้าย PC → คานขวางท้าย',
    'PC side wall -> wall tab': 'ผนังข้าง PC → แผ่นยึดผนัง', 'lid -> lid corner angle': 'ฝา → ฉากยึดฝา',
    'lid corner angle -> side wall': 'ฉากยึดฝา → ผนังข้าง'}
FAST_TH = [('2 x ', '2 × '), ('M3x', 'M3×'), ('M4x', 'M4×'), (' countersunk', ' หัวจม'), (' into an M3 rivnut', ' เข้า rivnut M3'),
           (' into an M4 rivnut', ' เข้า rivnut M4'), (' into M3 rivnuts', ' เข้า rivnut M3'), (' + nylock', ' + น็อตล็อก'),
           (' + washer (head outside the rail)', ' + แหวน (หัวอยู่นอกคานข้าง) ขันเข้าเกลียวแผ่นตั้ง'), (' + washer', ' + แหวน ขันเข้าเกลียวแผ่นตั้ง'),
           (' nylon', ' ไนลอน'), (' A2', ' สเตนเลส A2'), (' 8.8', ' เกรด 8.8')]


def what_th(w):
    return WHAT_TH['corner cleat'] if w.startswith('corner cleat') else WHAT_TH.get(w, w)


def fast_th(f):
    for a, b in FAST_TH:
        f = f.replace(a, b)
    return f


def fix_step(f):
    """Guide step key for a CAD fixing (CAD groups: 1 frame, 2 drive, 3 weapon, 4 hinge, 5 strap tabs, 6 plate, 7 skins, 8 lid)."""
    if f['step'] == 7:
        return 'floor' if 'floor' in f['what'] else 'walls'
    return {1: 'frame', 5: 'frame', 2: 'drive', 3: 'weapon', 4: 'wedge', 6: 'elec', 8: 'lid'}[f['step']]


def parts_table(parts):
    c = collections.Counter(label(b.name) for b in parts)
    rows = ''.join('<tr><td>%s</td><td class="n">%d</td></tr>' % (e(k), v) for k, v in c.items())
    return '<table><tr><th>ชิ้นส่วน</th><th>จำนวน</th></tr>%s</table>' % rows


def fix_table(key):
    fs = [f for f in FIX if fix_step(f) == key]
    if not fs:
        return '<p class="muted">ขั้นนี้ไม่มีจุดยึดจากรายการจุดยึด (ใช้วิธีในข้อ "วิธีทำ")</p>'
    c = collections.Counter((what_th(f['what']), fast_th(f['fastener'])) for f in fs)
    n = sum(2 if f['fastener'].startswith('2 x') else 1 for f in fs)
    rows = ''.join('<tr><td>%s</td><td>%s</td><td class="n">%d</td></tr>' % (e(w), e(fa), k) for (w, fa), k in c.items())
    return '<table><tr><th>จุดยึด</th><th>น็อต / สกรู</th><th>จุด</th></tr>%s</table><p class="muted">รวม %d ตัว (ตำแหน่งดูภาคผนวก)</p>' % (rows, n)


m = REP['mass']
CSS = '''
@page { size: A4; margin: 14mm 13mm 14mm 13mm; }
body { font-family: "Sukhumvit Set", "Thonburi", sans-serif; font-size: 10.5pt; color: #1d2733; line-height: 1.5; }
h1 { font-size: 22pt; margin: 0 0 4mm; } h2 { font-size: 15pt; margin: 0 0 3mm; border-bottom: 2px solid #f39c12; padding-bottom: 1mm; }
h3 { font-size: 11.5pt; margin: 4mm 0 1.5mm; }
.page { page-break-after: always; } .page:last-child { page-break-after: auto; }
img.step { display: block; width: 100%; height: 88mm; object-fit: contain; border: 1px solid #e1e4e8; border-radius: 4px; }
.cover img.step { height: 75mm; }
table { border-collapse: collapse; width: 100%; font-size: 8.8pt; margin: 1mm 0 2mm; }
th, td { border: 1px solid #d0d5db; padding: 0.8mm 1.6mm; text-align: left; vertical-align: top; } th { background: #f2f4f6; }
td.n { text-align: right; width: 12mm; }
.muted { color: #6b7785; font-size: 9pt; } .warn { background: #fff4e5; border-left: 4px solid #f39c12; padding: 2mm 3mm; margin: 2mm 0; }
.ok { background: #eef8f0; border-left: 4px solid #2e9e57; padding: 2mm 3mm; margin: 2mm 0; }
.two { display: flex; gap: 4mm; } .two > div { flex: 1; }
ol, ul { margin: 1mm 0 2mm 5mm; padding-left: 4mm; } li { margin-bottom: 1mm; }
.tag { display: inline-block; background: #1d2733; color: white; border-radius: 3px; padding: 0 2mm; font-size: 9pt; }
'''
pages = []
pages.append('''<div class="page cover"><h1>คู่มือประกอบ ROBOT V4 R5-AL</h1>
<p><span class="tag">CONCEPT CAD · DRAFT</span> &nbsp; โครงอะลูมิเนียมยึดน็อต · ไม่มีชิ้นกลึง · %s</p>
<div class="two"><div><img class="step" src="img/00_cover.png"></div><div><img class="step" src="img/00_cover_rear.png"></div></div>
<div class="two"><div><h3>ข้อมูลรถ</h3><ul>
<li>น้ำหนักจาก CAD <b>%.0f g</b> (กติกาไม่เกิน 2,000 g)</li><li>อาวุธ beater แนวตั้ง จานเหล็ก 2 ใบ เก็บพลังงานประมาณ 61 J</li>
<li>ขับ 2 ล้อหน้า ดันได้ประมาณ %.1f N (μ 0.6)</li><li>แกน Z ขึ้น หน้ารถ +Y หน่วย มม.</li></ul></div>
<div><h3>เอกสารนี้ใช้ทำอะไร</h3><p>บอกลำดับประกอบหลังจากตัดและเจาะชิ้นส่วนโครงเสร็จแล้ว ทุกภาพ รายการชิ้นส่วน และรายการน็อต สร้างจาก CAD
(<code>build_r5_al.py</code>) จึงตรงกับแบบเสมอ ชิ้นใหม่ของแต่ละขั้นเป็นสี ชิ้นที่ประกอบแล้วเป็นสีเทา</p></div></div>
<div class="warn"><b>สถานะ DRAFT:</b> แบบผ่านการตรวจระยะชนและความแข็งแรงแล้ว แต่อะไหล่ยังไม่ได้วัดของจริง ก่อนเจาะรูยึดมอเตอร์ แบต และบอร์ด ให้วัดของจริงก่อนทุกครั้ง
ท่ออะลูมิเนียมต้องเป็นเกรด <b>6063-T6 ขึ้นไป</b> (T5 ไม่ผ่านการตรวจแรง)</div></div>''' % (date.today().isoformat(), m['total_g'], REP['support_2wd']['push_N_mu0.6']))

# overview page: tools, safety, order
order_rows = ''.join('<tr><td>%s</td><td class="n">%d</td></tr>' % (e(st['title']), len(st['parts'])) for st in STEPS)
pages.append('''<div class="page"><h2>ก่อนเริ่ม</h2>
<div class="two"><div><h3>เครื่องมือ</h3><ul>
<li>สว่าน + ดอก 2.5 / 3.3 / 3.4 / 4.5 / 5.0 / 6.0 มม. (สว่านแท่นจะช่วยให้รูตรง)</li><li>ชุดต๊าป M4 (+ M3 ถ้าต้องการ)</li>
<li>คีมย้ำ rivnut (หัว M3 และ M4)</li><li>เลื่อยตัดเหล็ก / หินเจียตัด, ตะไบ, ฉากเหล็ก, เวอร์เนีย</li>
<li>ประแจหกเหลี่ยม 2 / 2.5 / 3 มม., ประแจ 5.5 / 7 / 13 มม.</li><li>ตู้เชื่อมไฟฟ้า + ลวด 2.0 มม. (ใช้เฉพาะขั้น 4)</li>
<li>หัวแร้ง, ที่ปอกสาย, ท่อหด, มัลติมิเตอร์</li><li>Loctite 648 (ล็อกลูกปืน), Loctite 243 (ล็อกเกลียว), กาว CA</li></ul></div>
<div><h3>ความปลอดภัย</h3><ul>
<li><b>ทดสอบอาวุธในกล่องกันหรือหลังแผ่นกั้นเสมอ</b> จานหมุนประมาณ 12,000 rpm</li>
<li>ถอดสวิตช์ตัดไฟทุกครั้งที่จับรถหรือทำงานกับรถ</li>
<li>แบต LiPo: ชาร์จในถุงกันไฟ ห้ามชาร์จทิ้งไว้ ถ้าแบตบวมห้ามใช้</li>
<li>ใส่แว่นตานิรภัยตอนเชื่อม เจียร และทดสอบหมุนอาวุธ</li>
<li>เฟิร์มแวร์ต้องกด arm ก่อนอาวุธจะหมุน และถ้าสัญญาณจอยหลุดต้องหยุดทุกอย่าง (ทดสอบในหัวข้อการทดสอบ)</li></ul></div></div>
<h3>ลำดับการประกอบ</h3><table><tr><th>ขั้น</th><th>ชิ้นใหม่</th></tr>%s</table>
<div class="ok"><b>ทำไมเรียงแบบนี้:</b> โครงต้องตรงก่อน (ขั้น 1–2) → ระบบขับเพื่อทดสอบว่ารถวิ่งได้ (3) → ดุมอาวุธเชื่อมแยกบนโต๊ะ (4) แล้วค่อยติดตั้ง (5–6)
→ wedgelet (7) → ไฟฟ้า (8) → ปิดผนังและฝาเป็นอย่างสุดท้าย (9–10) เพื่อให้เข้าถึงข้างในได้ตลอดตอนประกอบและทดสอบ</div></div>''' % order_rows)

for i, st in enumerate(STEPS):
    how = ''.join('<li>%s</li>' % (h % {'mass': m['total_g']} if '%(mass)' in h else h) for h in st['how'])
    chk = ''.join('<li>%s</li>' % c for c in st['check'])
    pages.append('''<div class="page"><h2>%s</h2><img class="step" src="img/%02d_%s.png">
<div class="two"><div><h3>ชิ้นส่วนที่ใช้</h3>%s</div><div><h3>น็อตและจุดยึด</h3>%s</div></div>
<h3>วิธีทำ</h3><ol>%s</ol><div class="ok"><b>จุดตรวจก่อนไปขั้นถัดไป</b><ul>%s</ul></div></div>'''
                 % (e(st['title']), i + 1, st['key'], parts_table(st['parts']), fix_table(st['key']), how, chk))

pages.append('''<div class="page"><h2>การทดสอบก่อนลงสนาม</h2><ol>
<li><b>ทดสอบจอย:</b> อัปโหลด <code>firmware/pad_test/pad_test.ino</code> เข้า ESP32 เปล่า จับคู่จอย ค่าต้องเปลี่ยนตามคันโยก และส่งข้อมูลต่อเนื่องตอนถือคันโยกนิ่ง</li>
<li><b>เฟิร์มแวร์หลัก:</b> อัปโหลด <code>firmware/v4_controller</code> แล้ว<b>ถอดสายพานออกก่อน</b></li>
<li><b>failsafe:</b> ปิดจอยระหว่างที่ล้อหมุน ล้อต้องหยุดภายใน 0.5 วินาที</li>
<li><b>ขับ:</b> ยกรถให้ล้อลอยแล้วทดสอบทุกทิศ จากนั้นวางบนพื้น ทดสอบเลี้ยว และทดสอบขับตอนคว่ำ (เมื่อ IMU ตรวจว่ารถคว่ำ ต้องกลับทิศการขับให้อัตโนมัติ)</li>
<li><b>มอเตอร์อาวุธ (ยังไม่ใส่สายพาน):</b> arm แล้วเร่งช้า ๆ ทิศหมุนต้องทำให้ฟันด้านหน้าวิ่ง<b>ขึ้น</b> ถ้าผิดให้สลับสายมอเตอร์ 2 เส้นใดก็ได้</li>
<li><b>อาวุธจริง (ใส่สายพาน, อยู่หลังแผ่นกั้น):</b> เร่ง 25% → 50% → 100% ทีละขั้น ฟังเสียงและดูการสั่น ถ้าสั่นมาก ให้หยุดแล้วถ่วงสมดุลดุมใหม่ อ่านรอบจากเซนเซอร์ hall ใน Serial</li>
<li><b>อุณหภูมิ:</b> หมุนอาวุธ 3 นาทีเหมือนเวลาแข่ง แล้ววัดอุณหภูมิมอเตอร์ ถ้าเกิน 60°C ให้เปลี่ยนพูลเลย์เป็น ASA</li>
<li><b>ตกกระแทก:</b> ปล่อยรถตกจากความสูงประมาณ 50 ซม. ลงพื้นแข็ง 3 ครั้ง แล้วตรวจว่าน็อตไม่คลาย (ทา Loctite 243 ที่ M4 ทุกตัวหลังผ่านการทดสอบ)</li>
<li><b>ชั่งน้ำหนัก:</b> ไม่เกิน 2,000 g พร้อมแบต</li></ol>
<div class="warn">ระหว่างแข่ง หลังแต่ละแมตช์ให้ตรวจ: น็อต M4 ของแผ่นตั้งอาวุธ, รอยเชื่อมดุม, สายพาน, ลูกปืน 608 (ถือเป็นของสิ้นเปลือง เตรียมสำรองไว้), ปลายฟันจาน (เจียแต่งได้)</div></div>''')

rows = ''.join('<tr><td class="n">%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>' % (
    [st['key'] for st in STEPS].index(fix_step(f)) + 1, e(what_th(f['what']) + (' (' + f['what'].split()[-1] + ')' if f['what'].startswith('corner cleat') else '')),
    e(fast_th(f['fastener'])), e(f['rivnut_in'] or '—'), ', '.join('%.1f' % v for v in f['xyz'])) for f in sorted(FIX, key=lambda f: [st['key'] for st in STEPS].index(fix_step(f))))
pages.append('<div class="page"><h2>ภาคผนวก: ตารางจุดยึดทั้งหมด (%d จุด)</h2><p class="muted">ตำแหน่ง x, y, z เป็นมม. จาก CAD (แกน Z ขึ้น หน้ารถ +Y, '
             'ศูนย์กลางรถ x 0 · เพลาล้อหน้า y 18) · คอลัมน์ "ขั้น" คือขั้นในคู่มือนี้</p>'
             '<table><tr><th>ขั้น</th><th>จุดยึด</th><th>น็อต</th><th>rivnut ใน</th><th>x, y, z</th></tr>%s</table></div>' % (len(FIX), rows))

doc = '<!doctype html><html lang="th"><head><meta charset="utf-8"><title>V4 R5-AL Assembly Guide</title><style>%s</style></head><body>%s</body></html>' % (CSS, ''.join(pages))
HTML = HERE / 'V4_R5_AL_ASSEMBLY_GUIDE_TH.html'
HTML.write_text(doc, encoding='utf-8')
PDF = HERE / 'V4_R5_AL_ASSEMBLY_GUIDE_TH.pdf'
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=%s' % PDF,
                HTML.as_uri()], check=True, capture_output=True, timeout=180)
print('wrote', PDF, PDF.stat().st_size, 'bytes;', len(pages), 'pages of content')
