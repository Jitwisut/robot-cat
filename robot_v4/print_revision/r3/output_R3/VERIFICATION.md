# ตรวจ R3 ร่าง — 10 ต.ค. 2026

สถานะ DRAFT_NOT_RELEASED / ASSEMBLY_PENDING; build `5dfa96dcd76ec8505db7b12d26a1aad9cfd1e0a312cb58c4c2baec8a45bcc975` (18 ต.ค. 2026 สร้างใหม่หลังแก้ wedgelet 1.6 มม.ตั้งฉากผิว และรูต๊าปขอบ/ข้างแผ่นตั้ง; ดู r3/calc/wedgelet_tap_fix_calc.json). ใช้SURPASS1300KVและแบตGNBตามข้อมูลเว็บที่ยืนยันใน[WEB_CONFIRMED.md](../WEB_CONFIRMED.md); ไม่มีRELEASED ไม่อนุมัติผลิตชุดใหญ่ เก็บR1/R2เดิมไว้

| ตรวจ | ผล |
|---|---|
| CAD solids/BREP self-interference | passed=True; invalid/unexpected/sweep hitsเป็น0 |
| Rotor sweep | เผื่อ1mm ไม่พบhits |
| WedgeletL/R | −2..+14° ทีละ1°ไม่พบhitsที่posesตรวจ ไม่ใช่continuous proof ต้องทดลองหมุนจริง |
| บานพับ | ผนัง1.6mmคงเกณฑ์ |
| STL | 17แบบ/26ชิ้น closed/manifold/winding/duplicates/Z0ผ่าน |
| Trial STL | 41ไฟล์ผ่านmesh; tyreIDทีละ.1mm ไม่ใช่ผลสวมจริง |
| เพลาอาวุธ | SURPASSØ4ยื่น20mmในCAD สวมรอก13mm; screwx6อยู่บนเพลา |
| MuJoCo | latestSHAตรงกัน passed=True; mass/CGตรงใน0.0075mm |
| พัก/เร่ง/เลี้ยว/กลับหัว | rearloadจำลอง12.369%; ฟันต่ำสุด2.075mm; ไม่มีfloorcontacts;กลับหัว1.369m |
| ไฟฟ้าในmodel | weaponKV1300/R.038/I01.8จากคู่มือ; drive torqueยังnominal; predictedpeak28.87Aไม่ใช่กระแสวัดจริง |
| มวล/สมดุล | 1973.05g/rearstatic10.615%; hardlimitsผ่านเฉพาะค่าประมาณ ยังพลาด1900g/15% |
| Wedgelet / รูต๊าปแผ่นตั้ง | แผ่น1.6มม.ตั้งฉากผิว วัดใน STEP ได้1.600; เนื้อใต้countersink0.55≥0.5; รูต๊าปขอบØ2.5×8อยู่ในSTEP เนื้อกั้นถึงรูข้าง3.5≥2.0 (dimension check ใหม่ใน checks.py) |
| Tests | 46testsผ่าน; release/profile/evidence/budget/current/mesh/ย้อนหลัง |
| Release | --releaseปฏิเสธexit2เพราะยังขาดหลักฐานจริง; ไม่มีRELEASED |

Mesh auditใช้edge/winding/duplicate/component/volume/manifold3dร่วมCAD BREP self-interference ไม่ใช่layerpreviewของslicer ไม่มีprofile/3MFของร้าน ผลFEA/ชนจริงหรือเฟิร์มแวร์ในรายงาน

## งานที่ยังต้องปิด

43เหตุผลจากgate; หลายข้อเป็นรายละเอียดของกลุ่มงานเดียวกัน ชื่อUnconfirmed hardwareหมายถึงค่าชิ้นจริง ไม่ลบข้อมูลcatalogที่ยืนยันแล้ว:

- Unconfirmed hardware: drive_Front_L
- Unconfirmed hardware: drive_Front_R
- Unconfirmed hardware: drive_Rear_L
- Unconfirmed hardware: drive_Rear_R
- Unconfirmed hardware: weapon_motor
- Unconfirmed hardware: battery
- Unconfirmed hardware: esc
- Unconfirmed hardware: esp32
- Unconfirmed hardware: drv_left
- Unconfirmed hardware: drv_right
- Unconfirmed hardware: imu
- Unconfirmed hardware: disconnect
- Unconfirmed hardware: bearing_608
- Unconfirmed hardware: hinge_pin
- Unconfirmed hardware: insert_m3
- Unconfirmed hardware: nut_m3
- Unconfirmed hardware: hall_sensor
- Unconfirmed hardware: dead_shaft
- Unconfirmed hardware: clips
- Unconfirmed hardware: belt
- Unconfirmed hardware: skid
- Unconfirmed hardware: rotor_screws
- Unconfirmed hardware: baseline_fasteners
- Unconfirmed hardware: revision_fasteners
- Unconfirmed hardware: wiring
- Unconfirmed hardware: aux_sensors
- Unconfirmed hardware: straps
- Missing/current-build failed evidence: fit
- Missing/current-build failed evidence: preflight
- Calibrated fit dimensions missing
- Shop/printer/material profile unconfirmed
- Usable build volume/brim missing or invalid
- Slicer project/review/masses missing or stale
- Complete confirmed project cost missing
- Incomplete current-build evidence: fit
- Incomplete current-build evidence: preflight
- Incomplete current-build evidence: electrical
- Physical fit/slicer printer-material-profile evidence missing or changed
- Two-round fit evidence, all four shafts and five lid cycles required
- Required slicer review checks missing/failed
- Electrical ratings/current requirements incomplete
- Manufactured metal/foam part masses not physically confirmed: Upright_6061_6mm_L, Upright_6061_6mm_R, Weapon_Top_Brace_3mm, Weapon_Bottom_Brace_3mm, Beater_Disc_L, Beater_Disc_R, Beater_Hub_6061_D30, Spacer_Outer_L_ID8.2_OD11, Spacer_Inner_ID8.2_OD11, Spacer_Outer_R_ID8.2_OD11, Motor_Pulley_D30_1to1, Weapon_Motor_Mount_3mm, Wedgelet_L, Wedgelet_R, Skirt_Plate_08mm_L, Skirt_Plate_08mm_R, Skirt_Plate_08mm_Rear, IMU_Foam_Pad_1mm
- Mass/rear-load targets missed; current-build user decision required
