# V4_PRINT_R3 — DRAFT / ยังไม่ปล่อยผลิต

ไฟล์ชุดนี้มีค่าขนาดต้นแบบที่ยังไม่วัดจริง ห้ามส่งพิมพ์ชุดใหญ่จากสถานะนี้

Build SHA256: `319857d1a9f0a57815e203bf037db3a52d2c40b1a51fe7ead140aa4becb78534`

CAD ผ่าน: True · mesh ผ่าน: True
มวลประมาณ: 1975.48 g · โหลดล้อหลังประมาณ: 10.5%
MuJoCo ผ่าน: True · โหลดล้อหลังจำลอง: 12.3% · ระยะฟันต่ำสุดตอนเร่งใบ: 2.08 mm


## งานที่ต้องยืนยันก่อนปล่อยชุดผลิต

- มวลประมาณยังเกินเป้าหมาย 1900 g; เหลือเผื่อถึง 2000 g เพียง 24.52 g ต้องแทนด้วยน้ำหนัก slicer และอะไหล่จริงก่อนตัดสินใจ
- โหลดล้อหลังเชิงสถิตผ่านขั้นต่ำ 10% แต่ยังไม่ถึงเป้าหมาย 15%
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

## ชิ้นพิมพ์ต้นแบบ

| ไฟล์ | วัสดุ | จำนวน | ขนาด XYZ มม. |
|---|---|---:|---|
| V4_R3_Tub_TPU.stl | TPU95A | 1 | 190.523 × 217.261 × 54.001 |
| V4_R3_Lid_TPU_2mm.stl | TPU95A | 1 | 176.0 × 174.004 × 5.005 |
| V4_R3_Wedgelet_Block_PETG_L.stl | PETG | 1 | 54.0 × 41.2 × 14.0 |
| V4_R3_Wedgelet_Carrier_PETG_L1.stl | PETG | 1 | 12.5 × 15.2 × 20.0 |
| V4_R3_Wedgelet_Carrier_PETG_L2.stl | PETG | 1 | 12.5 × 15.2 × 16.0 |
| V4_R3_Wedgelet_Block_PETG_R.stl | PETG | 1 | 54.0 × 41.2 × 14.0 |
| V4_R3_Wedgelet_Carrier_PETG_R1.stl | PETG | 1 | 12.5 × 15.2 × 20.0 |
| V4_R3_Wedgelet_Carrier_PETG_R2.stl | PETG | 1 | 12.5 × 15.2 × 16.0 |
| V4_R3_Wheel_Hub_PETG.stl | PETG | 4 | 44.003 × 44.003 × 16.003 |
| V4_R3_Wheel_Tyre_TPU95A.stl | TPU95A | 4 | 64.005 × 64.005 × 16.005 |
| V4_R3_Hall_Post_PETG.stl | PETG | 1 | 46.5 × 16.0 × 10.0 |
| V4_R3_Weapon_Mount_Foot_PETG_1.stl | PETG | 1 | 8.0 × 13.0 × 13.0 |
| V4_R3_Weapon_Mount_Foot_PETG_2.stl | PETG | 1 | 8.0 × 11.0 × 13.0 |
| V4_R3_Battery_Tray_PETG.stl | PETG | 1 | 71.001 × 33.0 × 5.001 |
| V4_R3_Electronics_Platform_PETG.stl | PETG | 1 | 122.0 × 54.5 × 4.0 |
| V4_R3_Electronics_Standoff_PETG.stl | PETG | 4 | 8.001 × 8.001 × 30.601 |
| V4_R3_Disconnect_Collar_PETG.stl | PETG | 1 | 3.0 × 16.0 × 16.0 |
