# ผลตรวจ V4 Print R2 — 8 ต.ค. 2026

Build SHA256: `f568a492e35cf395c6ddf2f1246d8a590da9da676421b64ebb28d5cba1669f32`

- CAD solid validity / exact-source self-interference: ผ่าน
- Interferenceที่ไม่ได้ตั้งใจ, rotor sweepเผื่อ1mm, wedgeletทั้งสองด้าน−2..+14°: ผ่าน
- STL17ชนิด และชิ้นทดลอง38ไฟล์: ปิดผิว ทิศผิว ขอบ/หน้าซ้ำ component/manifoldและZ0ผ่าน
- STEPอ่านกลับ:1root/1shape; DXF9ไฟล์ audit0errors หน่วยmm
- Unit tests20ข้อผ่าน รวมค่าเว็บไม่ทดแทนactualและmeasurementoverrideค่าเว็บ
- MuJoCoมวลตรงCAD:ผ่าน; พัก/เร่งใบ/เลี้ยว/กลับหัว:ผ่าน
- น้ำหนักประมาณ1969.91g เหลือ30.09gก่อน2kg; เป้า1900gยังไม่ผ่าน
- โหลดล้อหลังเชิงสถิต10.57%; MuJoCo12.38%; ผ่านขั้นต่ำ10% ยังไม่ถึงเป้า15%
- ฟันต่ำสุดขณะเร่งใบประมาณ2.08mm ไม่มีfloorcontactในการจำลอง
- --releaseออกexit2ตามที่ควร เพราะหลักฐานจริง/ร้าน/งบยังขาด และไม่มีRELEASED

## ข้อจำกัด

ขนาดเว็บเป็นข้อมูลแคตตาล็อก ยังมีnominalของมอเตอร์ แบต ความสูงบอร์ดและขั้วต่ออยู่ ผลนี้ไม่ยืนยันความพร้อมผลิต ความแข็งแรงหลังชน การใช้ไฟจริง เวลารัน หรือเฟิร์มแวร์ ต้องวัดและทดลองกับอะไหล่SKUที่ซื้อจริงก่อน release; CADbody identifiersบางชื่อคงมาจากR1 ให้ใช้ค่าdimensionsและรุ่นในinputs_R2/sourcesเป็นหลัก

## ข้อที่ยังไม่ผ่านเกณฑ์ปล่อยผลิต

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
- Missing/current-build failed evidence: assembly
- Calibrated fit dimensions missing
- Shop/printer/material profile unconfirmed
- Usable build volume/brim missing or invalid
- Slicer project/review/masses missing or stale
- Complete confirmed project cost missing
