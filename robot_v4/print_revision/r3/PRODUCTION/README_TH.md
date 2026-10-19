# V4 R3 — รายการสั่งผลิต (ปล่อยทั้งหมด 2026-10-19)

ผู้สั่งอนุมัติให้ผลิตทั้งหมดก่อนวัดอะไหล่จริง โดยรับความเสี่ยงตาม `RELEASE_DECISION.json`
แบบนี้ใช้มอเตอร์ขับ **ORTECH JB361 (12V 400 rpm) 4 ตัว** และมอเตอร์อาวุธ **SURPASS C3536-V2 1300KV**
ถ้าซื้อรุ่นอื่น แบบจะไม่พอดี

## 1. ร้านตัดเลเซอร์โลหะ
`V4_R3_METAL_RELEASED.zip` — 9 แบบ 12 ชิ้น พร้อมใบสั่งผลิต `WORK_ORDER_PRODUCTION_TH.pdf`

## 2. ร้าน CNC แผ่น PC
`../pc_body/output/PC_3mm/` (DXF/STEP 7 แผ่น) + `../pc_body/output/PRODUCTION/WORK_ORDER_PC_BODY_TH.pdf`
ความหนาแผ่นที่รับได้: 3 มม. = 2.9–3.1, 2 มม. = 1.9–2.1

## 3. ร้านกลึง
รอกมอเตอร์อาวุธ (รู Ø4), ดุมอาวุธ, spacer 3 ตัว, แกนบานพับ — ไฟล์ STEP ใน `../output_R3/DRAFT/metal/` ดู `../MACHINING.md`

## 4. พิมพ์ 3D
**พิมพ์ก่อน (ไม่ถึง 1 ชม.):** `Insert_Hole_D4.4_PETG.stl` และวงยาง 1 วงจาก `../output_R3/TRIAL_ONLY/`
กด insert ลงรูให้ได้ ไม่หมุน ไม่หลุด — ถ้าหลวมหรือคับ บอก Claude ให้เปลี่ยนขนาดรูแล้วสร้างไฟล์ใหม่

จาก `../output_R3/DRAFT/STL/`

| ไฟล์ | จำนวน | วัสดุ |
|---|---:|---|
| V4_R3_Wheel_Hub_PETG.stl (ซ้าย) | 2 | PETG |
| V4_R3_Wheel_Hub_PETG_variant1.stl (ขวา) | 2 | PETG |
| V4_R3_Wheel_Tyre_TPU95A.stl | 4 | TPU 95A |
| V4_R3_Wedgelet_Carrier_PETG_L1 / L2 / R1 / R2 | อย่างละ 1 | PETG |
| V4_R3_Weapon_Mount_Foot_PETG_1 / _2 | อย่างละ 1 | PETG |
| V4_R3_Battery_Tray_PETG.stl | 1 | PETG |
| V4_R3_Electronics_Platform_PETG.stl | 1 | PETG |
| V4_R3_Electronics_Standoff_PETG.stl | 4 | PETG |
| V4_R3_Disconnect_Collar_PETG.stl | 1 | PETG |
| V4_R3_Hall_Post_PETG.stl | 1 | PETG |

**ไม่พิมพ์** `V4_R3_Tub_TPU`, `V4_R3_Lid_TPU_2mm` (ใช้ตัวถัง PC แทน) และ `V4_R3_Wedgelet_Block_PETG_L/R` (ใช้รุ่น `_PCbody` แทน)

จาก `../pc_body/output/PETG_STL/` — พิมพ์ทุกไฟล์ (37 ไฟล์) ตามจำนวนและ infill ใน `WORK_ORDER_PC_BODY_TH.pdf`

ดุมล้อพิมพ์โดยวางหน้านอกลงฐาน จมูกดุม Ø9 ชี้ขึ้น

## ความเสี่ยงที่รับไว้ (สรุป)
- มอเตอร์ขับ: ใช้ค่าจากแบบ JB361 — เพลาต้องยื่นจากหน้ามอเตอร์ ≥ 11.2 มม. (แบบระบุ 11.5) ไม่งั้นดุมจับเพลาไม่พอ
- มอเตอร์อาวุธ: รูยึด 19×25 มม., บ่า Ø10 (เดา), เพลา Ø4 ยื่น 20 มม. ตามคู่มือ
- รู insert, รูในยาง, ระยะเผื่อต่างๆ เลือกโดยไม่ได้ทดสอบ
- น้ำหนักประมาณ 1,970.5 g เหลือก่อนลิมิต 2,000 g เพียง 29.5 g; ล้อหลังรับน้ำหนัก 10.8% (ขั้นต่ำ 10%)
- ยังไม่ได้ทดสอบแรงดึง insert และยังไม่ได้จำลองการชนด้วยมวลตัวถัง PC
