# Dimensions & Fastener Schedule

Measured out of the live Fusion model on 2026-09-23T03:19:18.

> Coordinate note (2026-09-23 14:36): The complete assembly was rotated after this report
> so its base lies on Fusion's XZ grid with Y up. World-axis bounding boxes and
> world-coordinate hole positions below describe the earlier orientation. Part
> dimensions and component-local geometry have not changed. Use the aligned F3D or
> refreshed STEP for current world coordinates.

> Not a drawing. Dimensioned 2D drawings are a Fusion Drawing-workspace
> job that the API cannot do. For fabrication use the DXF files in
> `exports/DXF/` and the STEP master; this table is the reference that
> goes with them.

## Overall envelope

| Axis | Min | Max | Size |
|---|---:|---:|---:|
| X (width) | -179.0 | -41.0 | **138.0 mm** |
| Y (length) | -73.8 | 86.2 | **160.0 mm** |
| Z (height) | -32.2 | 170.2 | **202.5 mm** |

Centre of mass: x -79.2, y 5.9, z 61.5 mm (z measured from ground)

## User parameters

| Parameter | Value | Comment |
|---|---:|---|
| `body_L` | 180 mm | Body length |
| `body_W` | 160 mm | Body width incl. wheels |
| `body_H` | 80 mm | Chassis height - raised from 70mm to give spinner sweep safe clearance under top cover |
| `base_t` | 2 mm | DECIDED 2026-09-23: 2 mm aluminium plate (user approved) |
| `side_t` | 2 mm | DECIDED 2026-09-23: 2 mm aluminium plate (user approved) |
| `ground_clearance` | 3 mm | Ground clearance target |
| `wheel_OD` | 42 mm | CONFIRMED: 42mm TT-motor wheel |
| `wheel_W` | 19 mm | CONFIRMED (typical TT wheel width; verify against actual supplier spec) |
| `motor_OD` | 25 mm | Gear motor housing envelope |
| `pcb_L` | 60 mm | Control PCB placeholder length |
| `pcb_W` | 50 mm | Control PCB placeholder width |
| `pcb_keepout_H` | 22 mm | PCB top keepout height (placeholder) |
| `assembly_clearance` | 1 mm | Generic assembly clearance |
| `spinner_gap_W` | 70 mm | MOCK layout gap for front spinner - NOT_FOR_MANUFACTURING, confirm with real weapon assembly |
| `wedge_depth` | 50 mm | MOCK front wedge depth (front-to-back) - NOT_FOR_MANUFACTURING placeholder |
| `battery_mock_L` | 104 mm | CONFIRMED: 3S 2200mAh LiPo typical size (Turnigy/GNB-class, ~500-700 THB Thai retail) |
| `battery_mock_W` | 35 mm | MOCK 3S LiPo envelope width - NOT_FOR_MANUFACTURING |
| `battery_mock_H` | 27 mm | MOCK 3S LiPo envelope height - NOT_FOR_MANUFACTURING |
| `pcb_t` | 1.6 mm | Standard PCB thickness placeholder |
| `module_mock_L` | 25 mm | MOCK small electronics module envelope length (driver/BEC/receiver/disconnect) - NOT_FOR_MANUFACTURING |
| `module_mock_W` | 20 mm | MOCK small electronics module envelope width - NOT_FOR_MANUFACTURING |
| `module_mock_H` | 10 mm | MOCK small electronics module envelope height - NOT_FOR_MANUFACTURING |
| `tt_gearbox_L` | 24 mm | TT motor gearbox length along shaft axis - TYPICAL VALUE, confirm exact supplier datasheet before drilling mount holes |
| `tt_gearbox_W` | 22 mm | TT motor gearbox cross-section width - TYPICAL VALUE, confirm before mount holes |
| `tt_gearbox_H` | 18 mm | TT motor gearbox cross-section height - TYPICAL VALUE, confirm before mount holes |
| `tt_can_D` | 20 mm | TT motor can (motor body) diameter - TYPICAL VALUE |
| `tt_can_L` | 25 mm | TT motor can (motor body) length - TYPICAL VALUE |
| `tt_shaft_D` | 5 mm | TT motor output D-shaft diameter - TYPICAL VALUE, confirm before hub fit |
| `tt_shaft_L` | 10 mm | TT motor output shaft protrusion length - TYPICAL VALUE |
| `hammer_servo_L` | 40.5 mm | CONFIRMED: DS3218MG servo body length (incl. mount tabs), srituhobby.com listing |
| `hammer_servo_W` | 20 mm | CONFIRMED: DS3218MG servo body width |
| `hammer_servo_H` | 40.5 mm | CONFIRMED: DS3218MG servo body height |
| `hammer_servo_horn_D` | 25 mm | MOCK: typical servo horn/spline envelope diameter |
| `hammer_arm_L` | 80 mm | MOCK hammer arm length from pivot to head - NOT_FOR_MANUFACTURING, confirm reach vs chassis geometry |
| `hammer_head_L` | 30 mm | MOCK hammer head length - NOT_FOR_MANUFACTURING |
| `hammer_head_W` | 20 mm | MOCK hammer head width - NOT_FOR_MANUFACTURING |
| `hammer_head_H` | 15 mm | MOCK hammer head height - NOT_FOR_MANUFACTURING |
| `spinner_motor_D` | 27.5 mm | CONFIRMED: A2212 brushless motor can diameter (standard size) |
| `spinner_motor_L` | 30 mm | CONFIRMED: A2212 brushless motor can length (standard size) |
| `spinner_shaft_D` | 4 mm | DECIDED 2026-09-23: 4 mm weapon shaft (separate from the A2212 3.17 mm motor shaft; driven by 1:1 belt) to fit the 624 bearing |
| `spinner_bearing_OD` | 13 mm | DECIDED 2026-09-23: 624 bearing, 4x13x5 mm, bore matches spinner_shaft_D |
| `spinner_bar_L` | 65 mm | MOCK spinner weapon bar length (within spinner_gap_W envelope) - NOT_FOR_MANUFACTURING, needs structural review before fabrication |
| `spinner_bar_W` | 20 mm | MOCK spinner weapon bar width - NOT_FOR_MANUFACTURING |
| `spinner_bar_T` | 6 mm | MOCK spinner weapon bar thickness - NOT_FOR_MANUFACTURING |
| `esc_L` | 45 mm | MOCK typical 30A BLHeli ESC envelope length |
| `esc_W` | 26 mm | MOCK typical 30A ESC envelope width |
| `esc_H` | 8 mm | MOCK typical 30A ESC envelope height |
| `print_wall_t` | 2 mm | 3D printed wall thickness placeholder (PETG typical) |
| `m3_clear_d` | 3.4 mm | M3 clearance hole dia for CNC/laser-cut aluminum (tight fit) |
| `m3_clear_d_fdm` | 3.6 mm | M3 clearance hole dia for FDM-printed parts (extra 0.2mm for print tolerance) |
| `standoff_od` | 6 mm | M3 corner standoff outer diameter |
| `standoff_bore_d` | 3.2 mm | Standoff through-bore for M3 screw (tap or use nut at one end) |
| `tt_hole_spacing` | 17.5 mm | CONFIRMED: TT motor (all-metal gearbox variant) 2-hole M3 mount spacing, handsontec.com FAM1062 datasheet - verify against exact purchased unit |
| `a2212_hole_x` | 16 mm | CONFIRMED: A2212 brushless motor rear mount hole spacing (short side), widely documented standard |
| `a2212_hole_y` | 19 mm | CONFIRMED: A2212 brushless motor rear mount hole spacing (long side) |
| `servo_flange_spacing` | 49.5 mm | CONFIRMED: standard servo flange mounting hole spacing (DS3218/MG996R-class), servodatabase.com |
| `mount_plate_t` | 3 mm | Motor mounting bracket/plate thickness (aluminum) |
| `lip_h` | 5 mm | Battery holder side-lip height |
| `strap_slot_w` | 10 mm | Battery strap slot width |
| `strap_slot_l` | 4 mm | Battery strap slot length (thin slit) |
| `pcb_standoff_h` | 3 mm | PCB standoff height (raises board off floor) |
| `m2_5_clear_d_fdm` | 2.9 mm | M2.5 clearance hole for FDM-printed standoffs |
| `m2_clear_d` | 2.2 mm | M2 clearance hole for small electronics module mounting |
| `roll_pin_d` | 2 mm | Roll pin diameter through shaft+bar - shock-resistant weapon fastening, stronger than friction set-screw alone |

## Parts

| Part | X | Y | Z | Size (mm) | Material | Mass |
|---|---|---|---|---|---|---:|
| Base_Plate_Al | -46.0…-44.0 | -73.8…86.2 | -27.2…152.8 | 2.0 × 160.0 × 180.0 | Aluminum 6061 | 142.36 g |
| Batt_Post_1 | -73.6…-46.0 | 37.8…42.8 | 30.2…35.2 | 27.6 × 5.0 × 5.0 | Nylon 6 | 0.54 g |
| Batt_Post_2 | -73.6…-46.0 | -30.2…-25.2 | 30.2…35.2 | 27.6 × 5.0 × 5.0 | Nylon 6 | 0.54 g |
| Batt_Post_3 | -73.6…-46.0 | 37.8…42.8 | 14.2…19.2 | 27.6 × 5.0 × 5.0 | Nylon 6 | 0.54 g |
| Batt_Post_4 | -73.6…-46.0 | -30.2…-25.2 | 14.2…19.2 | 27.6 × 5.0 × 5.0 | Nylon 6 | 0.54 g |
| Battery_Holder | -80.6…-73.6 | -45.8…58.2 | 5.2…44.2 | 7.0 × 104.0 × 39.0 | Nylon 6 | 10.24 g |
| Battery_and_Tray | -102.6…-75.6 | -45.8…58.2 | 7.2…42.2 | 27.0 × 104.0 × 35.0 | ABS Plastic | 104.18 g |
| DRV8874_A | -51.0…-48.0 | 23.4…41.1 | -18.9…-3.6 | 3.0 × 17.8 × 15.2 | ABS Plastic | 0.86 g |
| DRV8874_B | -51.0…-48.0 | 2.6…17.9 | -21.1…-3.4 | 3.0 × 15.2 × 17.8 | ABS Plastic | 0.86 g |
| Disconnect_Post_1 | -86.0…-46.0 | -29.2…-23.2 | -10.2…-4.2 | 40.0 × 6.0 × 6.0 | Aluminum 6061 | 2.90 g |
| Disconnect_Post_2 | -86.0…-46.0 | -48.2…-42.2 | -10.2…-4.2 | 40.0 × 6.0 × 6.0 | Aluminum 6061 | 2.90 g |
| ESC_Tekko32 | -52.5…-48.0 | -45.9…-11.6 | 82.1…99.4 | 4.5 × 34.3 × 17.3 | ABS Plastic | 2.83 g |
| ESP32_DevKit | -67.6…-66.0 | -19.5…32.0 | 10.6…38.9 | 1.6 × 51.5 × 28.3 | ABS Plastic | 2.44 g |
| ESP32_Dupont_Keepout_A | -66.0…-49.5 | -8.6…27.0 | 10.8…13.3 | 16.5 × 35.6 × 2.5 | ABS Plastic | 1.58 g |
| ESP32_Dupont_Keepout_B | -66.0…-49.5 | -8.6…27.0 | 36.2…38.7 | 16.5 × 35.6 × 2.5 | ABS Plastic | 1.58 g |
| ESP32_Module | -70.7…-67.6 | 6.5…32.0 | 15.8…33.8 | 3.1 × 25.5 × 18.0 | ABS Plastic | 1.51 g |
| Front_Wedge_L | -124.0…-46.0 | 41.2…86.2 | 102.8…152.8 | 78.0 × 45.0 × 50.0 | Aluminum 6061 | 71.86 g |
| Front_Wedge_R | -124.0…-46.0 | -73.8…-28.8 | 102.8…152.8 | 78.0 × 45.0 × 50.0 | Aluminum 6061 | 71.86 g |
| Hammer_Arm | -176.5…-166.5 | -1.2…13.8 | -17.2…62.8 | 10.0 × 15.0 × 80.0 | Aluminum 6061 | 32.40 g |
| Hammer_Head | -179.0…-164.0 | -3.8…16.2 | -32.2…-2.2 | 15.0 × 20.0 × 30.0 | Aluminum 6061 | 18.22 g |
| Hammer_Servo | -166.5…-126.0 | -3.8…16.2 | 34.0…91.5 | 40.5 × 20.0 × 57.5 | ABS Plastic | 35.80 g |
| Holder_DRV8874_A | -50.0…-46.0 | 21.6…43.0 | -20.6…-1.9 | 4.0 × 21.4 × 18.8 | Nylon 6 | 1.02 g |
| Holder_DRV8874_B | -50.0…-46.0 | 0.8…19.6 | -22.9…-1.6 | 4.0 × 18.8 × 21.4 | Nylon 6 | 1.02 g |
| Holder_ESC_Tekko32 | -51.5…-46.0 | -47.7…-9.8 | 80.3…101.2 | 5.5 × 37.9 × 20.9 | Nylon 6 | 2.32 g |
| Holder_ESP32 | -66.0…-46.0 | -21.5…34.0 | 8.6…40.9 | 20.0 × 55.5 × 32.3 | Nylon 6 | 4.45 g |
| Holder_Rx_ER6 | -62.0…-46.0 | 44.0…72.5 | -23.6…23.1 | 16.0 × 28.6 × 46.6 | Nylon 6 | 6.40 g |
| Holder_UBEC_5A | -57.0…-46.0 | -70.0…-49.5 | -15.1…38.5 | 11.0 × 20.6 × 53.6 | Nylon 6 | 4.61 g |
| Left_Side_Plate_Al | -124.0…-46.0 | 84.2…86.2 | -27.2…102.8 | 78.0 × 2.0 × 130.0 | Aluminum 6061 | 42.86 g |
| Left_Support | -86.0…-73.0 | 28.8…38.8 | 131.2…144.2 | 13.0 × 10.0 × 13.0 | Aluminum 6061 | 3.24 g |
| Left_Wheel | -83.0…-41.0 | 67.2…86.2 | 41.8…83.8 | 42.0 × 19.0 × 42.0 | ABS Plastic | 27.90 g |
| Main_Disconnect | -96.0…-86.0 | -48.2…-23.2 | -17.2…2.8 | 10.0 × 25.0 × 20.0 | ABS Plastic | 5.22 g |
| Motor_Pulley | -89.5…-69.5 | -15.8…-9.8 | 78.5…98.5 | 20.0 × 6.0 × 20.0 | Aluminum 6061 | 4.96 g |
| Rear_Panel | -124.0…-46.0 | -71.8…84.2 | -27.2…-25.2 | 78.0 × 156.0 × 2.0 | Nylon 6 | 27.17 g |
| Right_Side_Plate_Al | -124.0…-46.0 | -73.8…-71.8 | -27.2…102.8 | 78.0 × 2.0 × 130.0 | Aluminum 6061 | 42.86 g |
| Right_Support | -86.0…-73.0 | -26.2…-16.2 | 131.2…144.2 | 13.0 × 10.0 × 13.0 | Aluminum 6061 | 3.24 g |
| Right_Wheel | -83.0…-41.0 | -73.8…-54.8 | 41.8…83.8 | 42.0 × 19.0 × 42.0 | ABS Plastic | 27.90 g |
| Rx_ER6 | -63.0…-48.0 | 45.8…70.8 | -21.8…21.2 | 15.0 × 25.0 × 43.0 | ABS Plastic | 17.09 g |
| Spinner_Bar | -89.5…-69.5 | 3.2…9.2 | 105.2…170.2 | 20.0 × 6.0 × 65.0 | Aluminum 7075 | 21.69 g |
| Spinner_Motor | -93.2…-65.8 | -8.8…21.2 | 74.8…102.2 | 27.5 × 30.0 × 27.5 | Aluminum 6061 | 48.11 g |
| Spinner_Motor_Cradle | -71.5…-46.0 | -8.8…21.2 | 74.8…103.8 | 25.5 × 30.0 × 29.0 | Aluminum 6061 | 35.81 g |
| Spinner_Shaft | -81.5…-77.5 | -26.2…38.8 | 135.8…139.8 | 4.0 × 65.0 × 4.0 | Steel | 6.41 g |
| Standoff_1 | -124.0…-46.0 | 78.2…84.2 | 94.8…100.8 | 78.0 × 6.0 × 6.0 | Aluminum 6061 | 5.61 g |
| Standoff_2 | -124.0…-46.0 | -71.8…-65.8 | -25.2…-19.2 | 78.0 × 6.0 × 6.0 | Aluminum 6061 | 5.46 g |
| Standoff_3 | -124.0…-46.0 | 78.2…84.2 | -25.2…-19.2 | 78.0 × 6.0 × 6.0 | Aluminum 6061 | 5.61 g |
| Standoff_4 | -124.0…-46.0 | -71.8…-65.8 | 94.8…100.8 | 78.0 × 6.0 × 6.0 | Aluminum 6061 | 5.61 g |
| TT_Gearbox_L | -71.0…-53.0 | 43.2…67.2 | 51.8…73.8 | 18.0 × 24.0 × 22.0 | Nylon 6 | 10.16 g |
| TT_Gearbox_R | -71.0…-53.0 | -54.8…-30.8 | 51.8…73.8 | 18.0 × 24.0 × 22.0 | Nylon 6 | 10.16 g |
| TT_Motor_Can_L | -72.0…-52.0 | 9.9…35.0 | 52.8…72.8 | 20.0 × 25.0 × 20.0 | Steel | 61.65 g |
| TT_Motor_Can_R | -72.0…-52.0 | -22.4…2.5 | 52.8…72.8 | 20.0 × 25.0 × 20.0 | Steel | 61.65 g |
| TT_Motor_Clamp_L | -74.0…-71.0 | 46.1…56.1 | 46.2…79.2 | 3.0 × 10.0 × 33.0 | Aluminum 6061 | 2.53 g |
| TT_Motor_Clamp_R | -74.0…-71.0 | -43.6…-33.6 | 46.2…79.2 | 3.0 × 10.0 × 33.0 | Aluminum 6061 | 2.53 g |
| TT_Motor_Mount_L | -69.5…-46.0 | 41.0…61.2 | 46.2…79.2 | 23.5 × 20.3 × 33.0 | Aluminum 6061 | 17.83 g |
| TT_Motor_Mount_R | -69.5…-46.0 | -48.8…-28.4 | 46.2…79.2 | 23.5 × 20.3 × 33.0 | Aluminum 6061 | 17.83 g |
| Top_Cover | -126.0…-124.0 | -73.8…86.2 | -27.2…152.8 | 2.0 × 160.0 × 180.0 | Nylon 6 | 63.67 g |
| UBEC_5A | -58.0…-48.0 | -68.2…-51.2 | -13.2…36.8 | 10.0 × 17.0 × 50.0 | ABS Plastic | 9.01 g |
| Weapon_Pulley | -89.5…-69.5 | -15.8…-9.8 | 127.8…147.8 | 20.0 × 6.0 × 20.0 | Aluminum 6061 | 4.89 g |

## Fastener schedule

Every cylindrical hole found in each solid, by diameter and position.

| Part | Ø | Axis | Position (x, y, z) |
|---|---:|---|---|
| Base_Plate_Al | 2.50 | X | (-42.0, -59.8, -9.2) |
| Base_Plate_Al | 2.50 | X | (-42.0, -59.8, 32.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, -41.9, 90.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, -15.6, 90.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, -8.8, 24.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, 10.2, -17.2) |
| Base_Plate_Al | 2.50 | X | (-42.0, 10.2, -7.2) |
| Base_Plate_Al | 2.50 | X | (-42.0, 21.2, 24.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, 27.2, -11.2) |
| Base_Plate_Al | 2.50 | X | (-42.0, 37.2, -11.2) |
| Base_Plate_Al | 2.50 | X | (-42.0, 58.2, -17.8) |
| Base_Plate_Al | 2.50 | X | (-42.0, 58.2, 17.3) |
| Base_Plate_Al | 2.90 | X | (-42.0, -45.2, -7.2) |
| Base_Plate_Al | 2.90 | X | (-42.0, -27.8, 16.8) |
| Base_Plate_Al | 2.90 | X | (-42.0, -27.8, 32.8) |
| Base_Plate_Al | 2.90 | X | (-42.0, -26.3, -7.2) |
| Base_Plate_Al | 2.90 | X | (-42.0, 40.2, 16.8) |
| Base_Plate_Al | 2.90 | X | (-42.0, 40.2, 32.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -68.8, -22.2) |
| Base_Plate_Al | 3.40 | X | (-41.0, -68.8, 97.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -66.7, 142.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -44.8, 48.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -44.8, 76.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -43.8, 112.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -32.5, 48.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, -32.5, 76.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 45.0, 48.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 45.0, 76.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 56.2, 112.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 57.2, 48.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 57.2, 76.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 79.2, 142.8) |
| Base_Plate_Al | 3.40 | X | (-41.0, 81.2, -22.2) |
| Base_Plate_Al | 3.40 | X | (-41.0, 81.2, 97.8) |
| Batt_Post_1 | 2.10 | X | (-73.6, 40.2, 32.8) |
| Batt_Post_1 | 2.10 | X | (-46.0, 40.2, 32.8) |
| Batt_Post_1 | 5.00 | X | (-46.0, 40.2, 32.8) |
| Batt_Post_2 | 2.10 | X | (-73.6, -27.8, 32.8) |
| Batt_Post_2 | 2.10 | X | (-46.0, -27.8, 32.8) |
| Batt_Post_2 | 5.00 | X | (-46.0, -27.8, 32.8) |
| Batt_Post_3 | 2.10 | X | (-73.6, 40.2, 16.8) |
| Batt_Post_3 | 2.10 | X | (-46.0, 40.2, 16.8) |
| Batt_Post_3 | 5.00 | X | (-46.0, 40.2, 16.8) |
| Batt_Post_4 | 2.10 | X | (-73.6, -27.8, 16.8) |
| Batt_Post_4 | 2.10 | X | (-46.0, -27.8, 16.8) |
| Batt_Post_4 | 5.00 | X | (-46.0, -27.8, 16.8) |
| Battery_Holder | 2.90 | X | (-72.6, -27.8, 16.8) |
| Battery_Holder | 2.90 | X | (-72.6, -27.8, 32.8) |
| Battery_Holder | 2.90 | X | (-72.6, 40.2, 16.8) |
| Battery_Holder | 2.90 | X | (-72.6, 40.2, 32.8) |
| Disconnect_Post_1 | 2.10 | X | (-86.0, -26.3, -7.2) |
| Disconnect_Post_1 | 2.10 | X | (-46.0, -26.3, -7.2) |
| Disconnect_Post_1 | 6.00 | X | (-46.0, -26.3, -7.2) |
| Disconnect_Post_2 | 2.10 | X | (-86.0, -45.2, -7.2) |
| Disconnect_Post_2 | 2.10 | X | (-46.0, -45.2, -7.2) |
| Disconnect_Post_2 | 6.00 | X | (-46.0, -45.2, -7.2) |
| ESP32_DevKit | 2.60 | X | (-65.0, -17.4, 13.0) |
| ESP32_DevKit | 2.60 | X | (-65.0, -17.4, 36.5) |
| ESP32_DevKit | 2.60 | X | (-65.0, 29.8, 13.0) |
| ESP32_DevKit | 2.60 | X | (-65.0, 29.9, 36.5) |
| Front_Wedge_L | 2.50 | X | (-46.0, 79.2, 142.8) |
| Front_Wedge_L | 3.40 | X | (-41.0, 56.2, 112.8) |
| Front_Wedge_L | 3.40 | X | (-41.0, 79.2, 142.8) |
| Front_Wedge_R | 2.50 | X | (-46.0, -66.7, 142.8) |
| Front_Wedge_R | 3.40 | X | (-41.0, -66.7, 142.8) |
| Front_Wedge_R | 3.40 | X | (-41.0, -43.8, 112.8) |
| Hammer_Servo | 3.40 | X | (-125.0, 6.2, 38.0) |
| Hammer_Servo | 3.40 | X | (-125.0, 6.3, 87.5) |
| Holder_DRV8874_A | 3.40 | X | (-45.0, 27.2, -11.2) |
| Holder_DRV8874_A | 3.40 | X | (-45.0, 37.2, -11.2) |
| Holder_DRV8874_B | 3.40 | X | (-45.0, 10.2, -17.2) |
| Holder_DRV8874_B | 3.40 | X | (-45.0, 10.2, -7.2) |
| Holder_ESC_Tekko32 | 3.40 | X | (-45.0, -41.9, 90.8) |
| Holder_ESC_Tekko32 | 3.40 | X | (-45.0, -15.6, 90.8) |
| Holder_ESP32 | 1.60 | X | (-66.0, -17.4, 13.0) |
| Holder_ESP32 | 1.60 | X | (-66.0, -17.4, 36.5) |
| Holder_ESP32 | 1.60 | X | (-66.0, 29.8, 13.0) |
| Holder_ESP32 | 1.60 | X | (-66.0, 29.9, 36.5) |
| Holder_ESP32 | 3.40 | X | (-45.0, -8.8, 24.8) |
| Holder_ESP32 | 3.40 | X | (-45.0, 21.2, 24.8) |
| Holder_ESP32 | 4.50 | X | (-48.0, -17.4, 13.0) |
| Holder_ESP32 | 4.50 | X | (-48.0, -17.4, 36.5) |
| Holder_ESP32 | 4.50 | X | (-48.0, 29.8, 13.0) |
| Holder_ESP32 | 4.50 | X | (-48.0, 29.9, 36.5) |
| Holder_Rx_ER6 | 3.40 | X | (-45.0, 58.2, -17.8) |
| Holder_Rx_ER6 | 3.40 | X | (-45.0, 58.2, 17.3) |
| Holder_UBEC_5A | 3.40 | X | (-45.0, -59.8, -9.2) |
| Holder_UBEC_5A | 3.40 | X | (-45.0, -59.8, 32.8) |
| Left_Side_Plate_Al | 3.40 | Y | (-85.0, 86.2, -22.2) |
| Left_Side_Plate_Al | 3.40 | Y | (-85.0, 86.2, 97.8) |
| Left_Support | 4.00 | Y | (-79.5, 6.3, 137.8) |
| Main_Disconnect | 2.20 | X | (-86.0, -45.2, -7.2) |
| Main_Disconnect | 2.20 | X | (-86.0, -26.3, -7.2) |
| Motor_Pulley | 3.17 | Y | (-79.5, 6.3, 88.5) |
| Rear_Panel | 3.40 | Z | (-96.0, -68.8, 62.8) |
| Rear_Panel | 3.40 | Z | (-96.0, 81.2, 62.8) |
| Rear_Panel | 3.40 | Z | (-61.0, -68.8, 62.8) |
| Rear_Panel | 3.40 | Z | (-61.0, 81.2, 62.8) |
| Right_Side_Plate_Al | 3.40 | Y | (-85.0, -73.8, -22.2) |
| Right_Side_Plate_Al | 3.40 | Y | (-85.0, -73.8, 97.8) |
| Right_Support | 4.00 | Y | (-79.5, 6.3, 137.8) |
| Spinner_Bar | 2.00 | X | (-41.0, 6.3, 137.8) |
| Spinner_Bar | 4.00 | Y | (-79.5, 6.3, 137.8) |
| Spinner_Motor_Cradle | 2.50 | X | (-46.0, -4.8, 78.8) |
| Spinner_Motor_Cradle | 2.50 | X | (-46.0, -4.7, 99.8) |
| Spinner_Motor_Cradle | 2.50 | X | (-46.0, 17.2, 78.8) |
| Spinner_Motor_Cradle | 2.50 | X | (-46.0, 17.3, 99.8) |
| Spinner_Shaft | 4.00 | Y | (-79.5, 38.8, 137.8) |
| Standoff_1 | 2.50 | X | (-124.0, 81.2, 97.8) |
| Standoff_1 | 2.50 | X | (-46.0, 81.2, 97.8) |
| Standoff_1 | 2.50 | Y | (-85.0, 6.3, 97.8) |
| Standoff_1 | 6.00 | X | (-46.0, 81.2, 97.8) |
| Standoff_2 | 2.50 | X | (-124.0, -68.8, -22.2) |
| Standoff_2 | 2.50 | X | (-46.0, -68.8, -22.2) |
| Standoff_2 | 2.50 | Y | (-85.0, 6.2, -22.2) |
| Standoff_2 | 2.50 | Z | (-96.0, -68.8, 62.8) |
| Standoff_2 | 2.50 | Z | (-61.0, -68.8, 62.8) |
| Standoff_2 | 6.00 | X | (-46.0, -68.8, -22.2) |
| Standoff_3 | 2.50 | X | (-124.0, 81.2, -22.2) |
| Standoff_3 | 2.50 | X | (-46.0, 81.2, -22.2) |
| Standoff_3 | 2.50 | Y | (-85.0, 6.2, -22.2) |
| Standoff_3 | 6.00 | X | (-46.0, 81.2, -22.2) |
| Standoff_4 | 2.50 | X | (-124.0, -68.8, 97.8) |
| Standoff_4 | 2.50 | X | (-46.0, -68.8, 97.8) |
| Standoff_4 | 2.50 | Y | (-85.0, 6.3, 97.8) |
| Standoff_4 | 6.00 | X | (-46.0, -68.8, 97.8) |
| TT_Gearbox_L | 3.40 | Y | (-62.0, 67.2, 54.0) |
| TT_Gearbox_L | 3.40 | Y | (-62.0, 67.2, 71.5) |
| TT_Gearbox_R | 3.40 | Y | (-62.0, -54.8, 54.0) |
| TT_Gearbox_R | 3.40 | Y | (-62.0, -54.8, 71.5) |
| TT_Motor_Clamp_L | 3.40 | X | (-41.0, 51.1, 48.8) |
| TT_Motor_Clamp_L | 3.40 | X | (-41.0, 51.1, 76.8) |
| TT_Motor_Clamp_R | 3.40 | X | (-41.0, -38.6, 48.8) |
| TT_Motor_Clamp_R | 3.40 | X | (-41.0, -38.6, 76.8) |
| TT_Motor_Mount_L | 2.50 | X | (-69.5, 51.1, 48.8) |
| TT_Motor_Mount_L | 2.50 | X | (-69.5, 51.1, 76.8) |
| TT_Motor_Mount_L | 2.50 | X | (-46.0, 45.0, 48.8) |
| TT_Motor_Mount_L | 2.50 | X | (-46.0, 45.0, 76.8) |
| TT_Motor_Mount_L | 2.50 | X | (-46.0, 57.2, 48.8) |
| TT_Motor_Mount_L | 2.50 | X | (-46.0, 57.2, 76.8) |
| TT_Motor_Mount_R | 2.50 | X | (-69.5, -38.6, 48.8) |
| TT_Motor_Mount_R | 2.50 | X | (-69.5, -38.6, 76.8) |
| TT_Motor_Mount_R | 2.50 | X | (-46.0, -44.8, 48.8) |
| TT_Motor_Mount_R | 2.50 | X | (-46.0, -44.8, 76.8) |
| TT_Motor_Mount_R | 2.50 | X | (-46.0, -32.5, 48.8) |
| TT_Motor_Mount_R | 2.50 | X | (-46.0, -32.5, 76.8) |
| Top_Cover | 3.40 | X | (-122.0, -68.8, -22.2) |
| Top_Cover | 3.40 | X | (-122.0, -68.8, 97.8) |
| Top_Cover | 3.40 | X | (-122.0, 6.2, 38.0) |
| Top_Cover | 3.40 | X | (-122.0, 6.3, 87.5) |
| Top_Cover | 3.40 | X | (-122.0, 81.2, -22.2) |
| Top_Cover | 3.40 | X | (-122.0, 81.2, 97.8) |
| Weapon_Pulley | 4.00 | Y | (-79.5, 6.3, 137.8) |

Total holes listed: **153**
