# Mass & Interference — generated from the live Fusion model

Written by `fusion_scripts/BattleBotFixes.py` on 2026-09-23T02:48:26.
These numbers come from the model as it stands right now and
supersede the CAD-computed table in `BOM_and_Mass_Report.md`.

Assembly orientation updated on 2026-09-23 14:36. Body masses remain the same;
the centre of mass below was remeasured in the new world axes (X, Y up, Z).

## Totals

| Figure | Value |
|---|---:|
| Real bodies (excludes analysis envelopes) | 1121.9 g |
| Analysis envelopes (not parts) | 24.3 g |
| Model total as Fusion reports it | 1146.2 g |
| Centre of mass x / y / z | 0.3 / 38.2 / 1.3 mm |
| Interference pairs | 2 |

> Purchased parts are modelled as solid blocks, so their CAD mass is
> not their real mass. Use the real masses in `BOM_and_Mass_Report.md`
> for those and the figures here for fabricated parts.

## By subsystem

| Subsystem | Mass |
|---|---:|
| 01_Chassis | 457.8 g |
| 02_Drivetrain | 240.1 g |
| 03_Front_Spinner | 152.3 g |
| 04_Top_Hammer | 86.4 g |
| 05_Electronics | 144.3 g |
| 06_3D_Printed | 65.2 g |

## By body

| Subsystem | Body | Material | Mass |
|---|---|---|---:|
| 01_Chassis | Base_Plate_Al | Aluminum 6061 | 142.36 g |
| 01_Chassis | Left_Side_Plate_Al | Aluminum 6061 | 42.86 g |
| 01_Chassis | Right_Side_Plate_Al | Aluminum 6061 | 42.86 g |
| 01_Chassis | Top_Cover | Nylon 6 | 63.67 g |
| 01_Chassis | Front_Wedge_L | Aluminum 6061 | 71.86 g |
| 01_Chassis | Front_Wedge_R | Aluminum 6061 | 71.86 g |
| 01_Chassis | Standoff_1 | Aluminum 6061 | 5.61 g |
| 01_Chassis | Standoff_2 | Aluminum 6061 | 5.46 g |
| 01_Chassis | Standoff_3 | Aluminum 6061 | 5.61 g |
| 01_Chassis | Standoff_4 | Aluminum 6061 | 5.61 g |
| 02_Drivetrain | Left_Wheel | ABS Plastic | 27.90 g |
| 02_Drivetrain | Right_Wheel | ABS Plastic | 27.90 g |
| 02_Drivetrain | TT_Gearbox_L | Nylon 6 | 10.16 g |
| 02_Drivetrain | TT_Motor_Can_L | Steel | 61.65 g |
| 02_Drivetrain | TT_Gearbox_R | Nylon 6 | 10.16 g |
| 02_Drivetrain | TT_Motor_Can_R | Steel | 61.65 g |
| 02_Drivetrain | TT_Motor_Mount_L | Aluminum 6061 | 17.83 g |
| 02_Drivetrain | TT_Motor_Clamp_L | Aluminum 6061 | 2.53 g |
| 02_Drivetrain | TT_Motor_Mount_R | Aluminum 6061 | 17.83 g |
| 02_Drivetrain | TT_Motor_Clamp_R | Aluminum 6061 | 2.53 g |
| 03_Front_Spinner | Spinner_Shaft | Steel | 6.41 g |
| 03_Front_Spinner | Left_Support | Aluminum 6061 | 3.24 g |
| 03_Front_Spinner | Right_Support | Aluminum 6061 | 3.24 g |
| 03_Front_Spinner | Spinner_Sweep_Envelope | ABS Plastic | 21.10 g |
| 03_Front_Spinner | Spinner_Motor | Aluminum 6061 | 48.11 g |
| 03_Front_Spinner | Spinner_Bar | Aluminum 7075 | 21.69 g |
| 03_Front_Spinner | Spinner_Motor_Cradle | Aluminum 6061 | 35.81 g |
| 03_Front_Spinner | Weapon_Pulley | Aluminum 6061 | 4.89 g |
| 03_Front_Spinner | Motor_Pulley | Aluminum 6061 | 4.96 g |
| 03_Front_Spinner | ESC_Tekko32 | ABS Plastic | 2.83 g |
| 04_Top_Hammer | Hammer_Servo | ABS Plastic | 35.80 g |
| 04_Top_Hammer | Hammer_Arm | Aluminum 6061 | 32.40 g |
| 04_Top_Hammer | Hammer_Head | Aluminum 6061 | 18.22 g |
| 05_Electronics | Main_Disconnect | ABS Plastic | 5.22 g |
| 05_Electronics | Battery_and_Tray | ABS Plastic | 104.18 g |
| 05_Electronics | Rx_ER6 | ABS Plastic | 17.09 g |
| 05_Electronics | UBEC_5A | ABS Plastic | 9.01 g |
| 05_Electronics | DRV8874_A | ABS Plastic | 0.86 g |
| 05_Electronics | DRV8874_B | ABS Plastic | 0.86 g |
| 05_Electronics | ESP32_DevKit | ABS Plastic | 2.44 g |
| 05_Electronics | ESP32_Module | ABS Plastic | 1.51 g |
| 05_Electronics | ESP32_Dupont_Keepout_A | ABS Plastic | 1.58 g |
| 05_Electronics | ESP32_Dupont_Keepout_B | ABS Plastic | 1.58 g |
| 06_3D_Printed | Rear_Panel | Nylon 6 | 27.17 g |
| 06_3D_Printed | Battery_Holder | Nylon 6 | 10.24 g |
| 06_3D_Printed | Batt_Post_1 | Nylon 6 | 0.54 g |
| 06_3D_Printed | Batt_Post_2 | Nylon 6 | 0.54 g |
| 06_3D_Printed | Batt_Post_3 | Nylon 6 | 0.54 g |
| 06_3D_Printed | Batt_Post_4 | Nylon 6 | 0.54 g |
| 06_3D_Printed | Disconnect_Post_1 | Aluminum 6061 | 2.90 g |
| 06_3D_Printed | Disconnect_Post_2 | Aluminum 6061 | 2.90 g |
| 06_3D_Printed | Holder_ESC_Tekko32 | Nylon 6 | 2.32 g |
| 06_3D_Printed | Holder_Rx_ER6 | Nylon 6 | 6.40 g |
| 06_3D_Printed | Holder_UBEC_5A | Nylon 6 | 4.61 g |
| 06_3D_Printed | Holder_DRV8874_A | Nylon 6 | 1.02 g |
| 06_3D_Printed | Holder_DRV8874_B | Nylon 6 | 1.02 g |
| 06_3D_Printed | Holder_ESP32 | Nylon 6 | 4.45 g |

## Interference pairs

| A | B | Volume |
|---|---|---:|
| Spinner_Shaft | Spinner_Sweep_Envelope | 75.398 mm³ |
| Spinner_Sweep_Envelope | Spinner_Bar | 7595.003 mm³ |

## Weapon energy (Spinner_Bar)

Model: thin bar about its centre, I = m*L^2/12, at 15540 RPM no-load.

| Material | Bar mass | Kinetic energy |
|---|---:|---:|
| Al 6061 (old mock) | 21.7 g | 10.1 J |
| Al 7075-T6 (applied) | 21.7 g | 10.1 J |
| Steel (NOT applied) | 63.1 g | 29.4 J |

no-load RPM, no gearing/belt ratio applied, bar treated as a uniform thin bar. Loaded RPM is lower. This is an order-of-magnitude figure for rule-checking, not a validated number.

**This is a screening figure, not a safety clearance.** A weapon at
this energy needs a rated containment box and a rules check against
the specific event before it is spun at any speed.
