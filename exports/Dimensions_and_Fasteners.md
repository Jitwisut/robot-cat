# ขนาด Assembly รุ่นแข่ง — `robot2`

วัดจาก Fusion หลังถอดค้อนบนและใส่ตัวแทนสวิตช์ FingerTech แล้ว แกนโลก: X = ซ้าย/ขวา, Y = สูงขึ้น, Z = หน้า/หลัง; กริดพื้นอยู่ที่ Y=0.

ข้อมูลนี้แทนรายงานเก่าที่วัดก่อนหมุน Assembly; สำเนาเดิมอยู่ที่ `Dimensions_and_Fasteners_legacy.md`. ตัวเลขรูยึดที่ไม่ได้ระบุใหม่ด้านล่างต้องอ่านจาก CAD/DXF ปัจจุบันและตรวจอะไหล่จริงก่อนส่งผลิต.

## ขอบเขตชิ้นงาน (ไม่รวม analysis envelope)

| แกน | ต่ำสุด | สูงสุด | ช่วง |
|---|---:|---:|---:|
| X | -80.00 | 80.00 | 160.00 mm |
| Y | -0.00 | 85.00 | 85.00 mm |
| Z | -107.50 | 90.00 | 197.50 mm |

## ขนาดชิ้นส่วนในพิกัดโลก

| กลุ่ม | ชิ้นส่วน | X (mm) | Y (mm) | Z (mm) | วัสดุใน CAD |
|---|---|---:|---:|---:|---|
| 01_Chassis | `Base_Plate_Al` | -80.00…80.00 | 3.00…5.00 | -90.00…90.00 | Aluminum 6061 |
| 06_3D_Printed | `Batt_Post_1` | -36.50…-31.50 | 5.00…32.60 | 27.50…32.50 | Nylon 6 |
| 06_3D_Printed | `Batt_Post_2` | 31.50…36.50 | 5.00…32.60 | 27.50…32.50 | Nylon 6 |
| 06_3D_Printed | `Batt_Post_3` | -36.50…-31.50 | 5.00…32.60 | 43.50…48.50 | Nylon 6 |
| 06_3D_Printed | `Batt_Post_4` | 31.50…36.50 | 5.00…32.60 | 43.50…48.50 | Nylon 6 |
| 06_3D_Printed | `Battery_Holder` | -52.00…52.00 | 32.60…39.60 | 18.50…57.50 | Nylon 6 |
| 05_Electronics | `Battery_and_Tray` | -52.00…52.00 | 34.60…61.60 | 20.50…55.50 | ABS Plastic |
| 05_Electronics | `DRV8874_A` | -34.90…-17.10 | 7.00…10.00 | 66.40…81.60 | ABS Plastic |
| 05_Electronics | `DRV8874_B` | -11.60…3.60 | 7.00…10.00 | 66.10…83.90 | ABS Plastic |
| 06_3D_Printed | `Disconnect_Post_1` | 29.50…35.50 | 5.00…45.00 | 67.00…73.00 | Aluminum 6061 |
| 06_3D_Printed | `Disconnect_Post_2` | 48.50…54.50 | 5.00…45.00 | 67.00…73.00 | Aluminum 6061 |
| 03_Front_Spinner | `ESC_Tekko32` | 17.85…52.15 | 7.00…11.50 | -36.65…-19.35 | ABS Plastic |
| 05_Electronics | `ESP32_DevKit` | -25.75…25.75 | 25.00…26.60 | 23.85…52.15 | ABS Plastic |
| 05_Electronics | `ESP32_Module` | -25.75…-0.25 | 26.60…29.70 | 29.00…47.00 | ABS Plastic |
| 05_Electronics | `FingerTech_Mini_Switch_Envelope` | 35.65…48.35 | 49.00…55.35 | 63.65…76.35 | Nylon 6 |
| 06_3D_Printed | `FingerTech_Switch_Adapter` | 28.00…56.00 | 45.00…49.00 | 61.00…79.00 | Nylon 6 |
| 01_Chassis | `Front_Wedge_L` | -80.00…-35.00 | 5.00…83.00 | -90.00…-40.00 | Aluminum 6061 |
| 01_Chassis | `Front_Wedge_R` | 35.00…80.00 | 5.00…83.00 | -90.00…-40.00 | Aluminum 6061 |
| 06_3D_Printed | `Holder_DRV8874_A` | -36.70…-15.30 | 5.00…9.00 | 64.60…83.40 | Nylon 6 |
| 06_3D_Printed | `Holder_DRV8874_B` | -13.40…5.40 | 5.00…9.00 | 64.30…85.70 | Nylon 6 |
| 06_3D_Printed | `Holder_ESC_Tekko32` | 16.05…53.95 | 5.00…10.50 | -38.45…-17.55 | Nylon 6 |
| 06_3D_Printed | `Holder_ESP32` | -27.75…27.75 | 5.00…25.00 | 21.85…54.15 | Nylon 6 |
| 06_3D_Printed | `Holder_Rx_ER6` | -66.30…-37.70 | 5.00…21.00 | 39.70…86.30 | Nylon 6 |
| 06_3D_Printed | `Holder_UBEC_5A` | 55.70…76.30 | 5.00…16.00 | 24.20…77.80 | Nylon 6 |
| 01_Chassis | `Left_Side_Plate_Al` | -80.00…-78.00 | 5.00…83.00 | -40.00…90.00 | Aluminum 6061 |
| 03_Front_Spinner | `Left_Support` | -32.50…-22.50 | 32.00…45.00 | -81.50…-68.50 | Aluminum 6061 |
| 02_Drivetrain | `Left_Wheel` | -80.00…-61.00 | -0.00…42.00 | -21.00…21.00 | ABS Plastic |
| 03_Front_Spinner | `Motor_Pulley` | 16.00…22.00 | 28.50…48.50 | -35.75…-15.75 | Aluminum 6061 |
| 06_3D_Printed | `Rear_Panel` | -78.00…78.00 | 5.00…83.00 | 88.00…90.00 | Nylon 6 |
| 01_Chassis | `Right_Side_Plate_Al` | 78.00…80.00 | 5.00…83.00 | -40.00…90.00 | Aluminum 6061 |
| 03_Front_Spinner | `Right_Support` | 22.50…32.50 | 32.00…45.00 | -81.50…-68.50 | Aluminum 6061 |
| 02_Drivetrain | `Right_Wheel` | 61.00…80.00 | -0.00…42.00 | -21.00…21.00 | ABS Plastic |
| 05_Electronics | `Rx_ER6` | -64.50…-39.50 | 7.00…22.00 | 41.50…84.50 | ABS Plastic |
| 03_Front_Spinner | `Spinner_Bar` | -3.00…3.00 | 28.50…48.50 | -107.50…-42.50 | Aluminum 7075 |
| 03_Front_Spinner | `Spinner_Motor` | -15.00…15.00 | 24.75…52.25 | -39.50…-12.00 | Aluminum 6061 |
| 03_Front_Spinner | `Spinner_Motor_Cradle` | -15.00…15.00 | 5.00…30.50 | -41.00…-12.00 | Aluminum 6061 |
| 03_Front_Spinner | `Spinner_Shaft` | -32.50…32.50 | 36.50…40.50 | -77.00…-73.00 | Steel |
| 01_Chassis | `Standoff_1` | -78.00…-72.00 | 5.00…83.00 | -38.00…-32.00 | Aluminum 6061 |
| 01_Chassis | `Standoff_2` | 72.00…78.00 | 5.00…83.00 | 82.00…88.00 | Aluminum 6061 |
| 01_Chassis | `Standoff_3` | -78.00…-72.00 | 5.00…83.00 | 82.00…88.00 | Aluminum 6061 |
| 01_Chassis | `Standoff_4` | 72.00…78.00 | 5.00…83.00 | -38.00…-32.00 | Aluminum 6061 |
| 02_Drivetrain | `TT_Gearbox_L` | -61.00…-37.00 | 12.00…30.00 | -11.00…11.00 | Nylon 6 |
| 02_Drivetrain | `TT_Gearbox_R` | 37.00…61.00 | 12.00…30.00 | -11.00…11.00 | Nylon 6 |
| 02_Drivetrain | `TT_Motor_Can_L` | -28.70…-3.70 | 11.00…31.00 | -10.00…10.00 | Steel |
| 02_Drivetrain | `TT_Motor_Can_R` | 3.70…28.70 | 11.00…31.00 | -10.00…10.00 | Steel |
| 02_Drivetrain | `TT_Motor_Clamp_L` | -49.85…-39.85 | 30.00…33.00 | -16.50…16.50 | Aluminum 6061 |
| 02_Drivetrain | `TT_Motor_Clamp_R` | 39.85…49.85 | 30.00…33.00 | -16.50…16.50 | Aluminum 6061 |
| 02_Drivetrain | `TT_Motor_Mount_L` | -55.00…-34.70 | 5.00…28.50 | -16.50…16.50 | Aluminum 6061 |
| 02_Drivetrain | `TT_Motor_Mount_R` | 34.70…55.00 | 5.00…28.50 | -16.50…16.50 | Aluminum 6061 |
| 01_Chassis | `Top_Cover` | -80.00…80.00 | 83.00…85.00 | -90.00…90.00 | Nylon 6 |
| 05_Electronics | `UBEC_5A` | 57.50…74.50 | 7.00…17.00 | 26.00…76.00 | ABS Plastic |
| 03_Front_Spinner | `Weapon_Pulley` | 16.00…22.00 | 28.50…48.50 | -85.00…-65.00 | Aluminum 6061 |

## รูยึดที่แก้ล่าสุด

- `FingerTech_Mini_Switch_Envelope`: ตัวเรือน 12.7 × 12.7 × 6.35 mm; รู M2 สองรูห่าง 7.62 mm ตามสเปกผู้ผลิต. เป็นตัวแทนรูปทรง ยังไม่รวมขั้วทองแดงและสายไฟ.
- `FingerTech_Switch_Adapter`: 28 × 18 × 4 mm, Nylon 6; รูสวิตช์ Ø2.2 mm ศูนย์ที่พิกัดชิ้นส่วนเดิม X=38.19/45.81, Y=-70 mm; รูยึดเสา Ø3.4 mm X=32.5/51.5, Y=-70 mm. พิกัดนี้เป็นพิกัดภายในชิ้นส่วน ก่อนหมุน Assembly; ในแกนโลก X คงเดิมและ Z=70 mm.
- ช่องเปิดฝาบนเดิม 20 × 16 mm ถูกปิดและแทนด้วยรูเข้าประแจ Ø6 mm ที่ศูนย์พิกัดชิ้นส่วน X=42, Y=-70 mm; ต้องตรวจระยะเอื้อมประแจและตำแหน่งขั้วไฟกับชิ้นจริง.
- รูยึด ESP32 บน holder ยังอิงรูปถ่าย ±0.5 mm; วัดบอร์ดจริงก่อนพิมพ์.

**ยังไม่ใช่แบบผลิตขั้นสุดท้าย:** ขนาดอะไหล่ที่ซื้อ, รูเพลาอาวุธ, การทำปีกหน้าเปลือกกลวง และทางเดินสายต้องยืนยันก่อนส่งร้าน.
