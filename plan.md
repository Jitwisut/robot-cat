# PLAN — ESP32 Mini Combat Robot (Fusion 360)

> **ประวัติแผนเริ่มต้น:** รุ่นแข่งปัจจุบันใน `robot2` ถอดค้อนบนแล้ว,
> ใส่ตัวแทน FingerTech switch และใช้กรงจักรหน้าเป็นอาวุธหลัก.
> ดู `STATUS.md` และ `exports/Design_Review_Competition_2026-09-23.md`
> สำหรับสถานะและแนวทางล่าสุด. รายการชิ้นส่วน/checkbox ด้านล่างไม่ใช่รายการสั่งผลิตปัจจุบัน.

> สถานะ: Draft v1.0 — แผนสร้าง **แบบจำลอง 3D / Assembly / พื้นที่ติดตั้ง PCB** ใน Autodesk Fusion (Fusion 360) ไม่ใช่ไฟล์ CAD ที่สร้างเสร็จแล้ว  
> แนวทางภาพอ้างอิง: หุ่นตัวเตี้ยทรงลิ่ม โครงสีอลูมิเนียม + ชิ้นส่วน 3D print สีดำ/ส้ม, **กรงจักรอยู่ตรงกลางด้านหน้า**, **ค้อนอยู่ด้านบน**, ล้อขับสองข้าง  
> เป้าหมาย: ต้นแบบขับเคลื่อนได้ที่ปรับขนาดตามอะไหล่จริงและข้อกำหนดการแข่งขันได้

## 0. เป้าหมาย / ขอบเขต

### ต้องทำ
- สร้างแบบ **Parametric CAD** ที่แก้ความกว้าง ความยาว ระยะล้อ ตำแหน่งอุปกรณ์ และความหนาวัสดุจาก `Change Parameters` ได้
- ทำ Assembly แยกแต่ละระบบ: chassis, drivetrain, front spinner assembly, hammer assembly, electronics/PCB, enclosure
- ใช้ฐานและเพลตหลักเป็นอลูมิเนียม และชิ้นส่วนที่ไม่รับแรงกระแทกหลักเป็น PETG/TPU/Nylon ตามหน้าที่
- เตรียมช่องและรูยึดสำหรับ ESP32 control PCB ที่จะออกแบบใน Fusion Electronics ภายหลัง
- ตรวจสอบการชนกันของชิ้นส่วน **ทุกตำแหน่งการเคลื่อนที่ของค้อน**, การหมุนเชิงเรขาคณิตของกรงจักร และการถอดเปลี่ยนแบต
- สรุปน้ำหนักโดยประมาณ, BOM พร้อมขนาด/แหล่งอ้างอิง, แบบ 2D และไฟล์ผลิตชิ้นงาน

### ยังไม่ทำในรอบโมเดลแรก
- ไม่ผลิตหรือทดลองอาวุธหมุนโดยไม่มีชุดป้องกันและแผนทดสอบที่เหมาะสม
- ไม่วาด PCB วงจรกำลัง BLDC/ESC ใหม่เอง; ใช้ ESC แยกจาก control PCB
- ไม่เลือกเส้นผ่านศูนย์กลางเพลา, ลูกปืน, fastener เกรดรับแรง, หรือน้ำหนักอาวุธจากหน้าตาภาพอย่างเดียว: ต้องยืนยันตามอะไหล่/ข้อกำหนดก่อนผลิต
- ไม่ถือว่าภาพเรนเดอร์เป็นแบบวิศวกรรมหรือหลักฐานว่าโครงสร้างปลอดภัย

## 1. ข้อกำหนดต้นแบบที่ต้องล็อกก่อนเริ่ม CAD

| รายการ | ค่าเริ่มต้นสำหรับจัดวาง (ยังไม่ใช่ค่าผลิต) | การตัดสินใจที่ต้องยืนยัน |
|---|---|---|
| ความยาวตัวหุ่น | 180 mm | วัดกับ ESC, แบต, ตำแหน่งค้อนจริง |
| ความกว้างรวมรวมล้อ | 160 mm | วัดล้อและเพลาของมอเตอร์ที่ซื้อ |
| ความสูงตัวถัง (ไม่รวมค้อนยก) | 70 mm | เผื่อฝาครอบ สายไฟ และจุดยึด |
| มวลเป้าหมาย | 1–1.5 kg | ตรวจคลาสแข่งขันและชั่งอะไหล่ |
| ระบบขับเคลื่อน | 2WD, มอเตอร์เกียร์ 25D × 2 | ยืนยันรุ่น เพลา/รูยึด/กระแส stall |
| กรงจักรหน้า | ชุดหมุนแกนขวาง **อยู่กึ่งกลางด้านหน้า** | ใช้ขนาดจริงจากชุดอาวุธ/แบบที่ผ่านการตรวจ |
| ค้อนบน | แขนค้อนหมุนหน้า–หลัง, ใช้ servo สำหรับ mock-up | ยืนยัน envelope ของ servo, arm, pivot |
| Controller | ESP32-C3 SuperMini ช่วงทดลอง | PCB v1 ค่อยล็อกขนาดจริง |
| แบต | LiPo 3S | วัดขนาดแบตจริงพร้อมหัวต่อและสาย |
| วัสดุหลัก | Al 5052/6061 + ชิ้น print | ยืนยันชนิด/ความหนาตามภาระรับแรงและวิธีผลิต |

**เกณฑ์สั่งซื้อ:** ต้องมี datasheet หรือขนาดที่วัดเองของมอเตอร์ล้อ, wheel hub, แบต, ESC, servo, bearing, ESP32/PCB, หัวต่อ ก่อนทำรูยึดสุดท้าย

## 2. โครงสร้างไฟล์และ Components ของ Fusion

ชื่อโปรเจกต์: `ESP32_Mini_BattleBot`  
ชื่อ Assembly หลัก: `BattleBot_Master`

```text
BattleBot_Master
├── 00_References
│   ├── Reference_Image
│   ├── Purchased_Parts_Dimensions
│   └── Safety_and_Rules
├── 01_Chassis
│   ├── Base_Plate_Al
│   ├── Left_Side_Plate_Al
│   ├── Right_Side_Plate_Al
│   ├── Front_Wedges_Al
│   ├── Top_Cover
│   └── Standoffs_Fasteners
├── 02_Drivetrain
│   ├── Left_25D_Motor
│   ├── Right_25D_Motor
│   ├── Left_Wheel
│   ├── Right_Wheel
│   └── Motor_Mounts
├── 03_Front_Spinner
│   ├── Spinner_Envelope [ระยะกวาดจำลองก่อน]
│   ├── Spinner_Assembly [แบบ/อะไหล่ยืนยันแล้ว]
│   ├── Shaft_and_Bearings
│   ├── Left_Support
│   ├── Right_Support
│   ├── Guard
│   └── ESC_and_Mount
├── 04_Top_Hammer
│   ├── Hammer_Sweep_Envelope
│   ├── Servo_or_Actuator
│   ├── Pivot_and_Bushing
│   ├── Arm
│   ├── Head
│   └── Hammer_Lock
├── 05_Electronics
│   ├── Battery_and_Tray
│   ├── ESP32_Control_Board [placeholder → 3D PCB]
│   ├── Wheel_Motor_Drivers
│   ├── Power_Buck_BEC
│   ├── RC_Receiver
│   ├── Main_Disconnect
│   └── Cable_Routing_Keepouts
└── 06_3D_Printed
    ├── PCB_Mount
    ├── Battery_Holder
    ├── Wire_Clips
    ├── Non_Structural_Covers
    └── TPU_Bumpers
```

**กฎการทำงาน:** สร้าง `New Component` ก่อนสร้าง solid; ตั้งชื่อทุก component/body/sketch; `Pin/Ground` chassis หลักไว้; ใช้ `Rigid` กับของที่ยึดติด และ `Revolute Joint` กับล้อ เพลากรงจักร และค้อน

## 3. User Parameters ที่ตั้งก่อนสร้าง Sketch

เปิด `Design > Solid > Modify > Change Parameters` และเพิ่มรายการต่อไปนี้ โดยใช้หน่วย mm หรือหน่วยตามชนิด parameter

| Parameter | ค่าเริ่มต้น | ความหมาย |
|---|---:|---|
| `body_L` | 180 mm | ความยาวตัวหุ่น |
| `body_W` | 160 mm | ความกว้างรวมตามแนวคิด (รวมล้อ) |
| `body_H` | 70 mm | ความสูงตัวถัง |
| `base_t` | 2 mm | ความหนาฐานเริ่มต้น; ยังไม่อนุมัติผลิต |
| `side_t` | 2 mm | ความหนาแผงข้างเริ่มต้น |
| `ground_clearance` | 3 mm | ช่องว่างใต้ท้องเป้าหมาย |
| `wheel_OD` | 50 mm | เส้นผ่านศูนย์กลางล้อจำลอง |
| `wheel_W` | 20 mm | ความกว้างล้อจำลอง |
| `motor_OD` | 25 mm | envelope ตัวเรือนมอเตอร์เกียร์ |
| `pcb_L` | 60 mm | control PCB placeholder |
| `pcb_W` | 50 mm | control PCB placeholder |
| `pcb_keepout_H` | 22 mm | placeholder พื้นที่ด้านบน PCB; ปรับตามชิ้นจริง |
| `assembly_clearance` | 1 mm | ช่องเผื่อประกอบเบื้องต้น; แยกตามกระบวนการผลิตจริง |

**สำคัญ:** อย่าใส่ค่า `spinner_OD`, `shaft_D`, `bearing_OD`, `hammer_sweep`, `battery_L/W/H` แบบเดาสำหรับงานผลิต ให้เพิ่ม parameter จากสเปกและแบบที่ยืนยันแล้ว ระหว่างรอให้ใช้ placeholder/envelope ติดป้าย `NOT_FOR_MANUFACTURING`

## 4. ลำดับลงมือทำใน Fusion 360

### Phase 0 — เก็บข้อมูลอะไหล่และข้อจำกัด

- [ ] ระบุคลาสน้ำหนักและกติกาของรายการแข่งที่ตั้งใจลง
- [ ] ทำตาราง part number / รูปทรง / ขนาด / มวล / แรงดัน / ข้อจำกัดการติดตั้ง
- [ ] เลือกขนาดแบตและพื้นที่สำหรับเสียบถอดหัวต่อได้จริง
- [ ] ยืนยันมอเตอร์ล้อพร้อมระยะรูยึดและเพลา
- [ ] ยืนยันขนาด servo/actuator, ESC, bearing และชุดกรงจักร
- [ ] รวบรวมภาพ top/front/side ที่ต้องการเป็น reference แต่ไม่ trace เป็นมิติวิศวกรรม

**ส่งมอบ:** `parts_dimensions.csv`, `requirements.md`, รูปอ้างอิง, ข้อกำหนดการแข่งขัน

**ผ่านเมื่อ:** อุปกรณ์จำเป็นทั้งหมดมีขนาดอ้างอิงและสถานะ `confirmed` หรือ `placeholder`

### Phase 1 — Skeleton / พื้นที่ติดตั้ง (ทำก่อนรายละเอียดสวยงาม)

- [ ] ตั้ง `Units = mm` และใส่ User Parameters
- [ ] สร้าง master layout บนระนาบ XY: โครงตัวถัง, กึ่งกลางด้านหน้า, ตำแหน่งล้อ, จุดหมุนค้อน
- [ ] วาดกึ่งกลางกรงจักรตรงแกนสมมาตรของหุ่นหน้า (`x=0` ถ้ากำหนด x เป็นซ้าย–ขวา)
- [ ] สร้าง keep-out volumes ของกรงจักร, ค้อน, แบต, PCB, สายไฟและหัวต่อ
- [ ] กำหนดพื้นที่เข้าถึง main disconnect / weapon lock และฝาเซอร์วิส
- [ ] ทำแบบ top, front, side แบบหยาบและตรวจว่าสองอาวุธอยู่ร่วมกันได้

**ส่งมอบ:** `BattleBot_Master` v0.1 พร้อม skeleton และ envelopes

**ผ่านเมื่อ:** ทุกชิ้นหลักมีที่วาง, ค้อนกวาดผ่านได้โดยไม่ชนกรงจักรหรือกล่อง PCB และแบตถอดได้

### Phase 2 — Chassis / Wedge / Mounts

- [ ] สร้าง `Base_Plate_Al` จาก sketch parametric แล้ว Extrude
- [ ] ทำแผ่นด้านซ้าย–ขวาเป็น components แยกกัน
- [ ] ทำปีกหน้า wedge ซ้ายและขวา **โดยคงช่องกรงจักรตรงกลาง**
- [ ] ทำตำแหน่ง fastener และช่องมือเข้าถึงน็อตโดยอิงชิ้นส่วนจริง
- [ ] ทำฝาบนที่ถอดออกได้และไม่บังคับให้รื้อค้อนเพื่อเปลี่ยนแบต
- [ ] ใส่วัสดุเชิงกายภาพให้ทุก component แยกจาก appearance

**ส่งมอบ:** `Chassis` v0.2 + top/bottom views

**ผ่านเมื่อ:** โครงสมมาตร, ประกอบ/ถอดชิ้นส่วนได้, ไม่ตัดผ่าน keep-out ของกลไก

### Phase 3 — ล้อและมอเตอร์ขับ

- [ ] Insert มอเตอร์ 25D จาก CAD ของผู้ผลิตหรือ placeholder ที่วัดจริง
- [ ] วางศูนย์เพลาล้อซ้าย–ขวาให้สัมพันธ์กับ `ground_clearance`
- [ ] ทำ mount ที่มีพื้นที่สำหรับหัวน็อต สายมอเตอร์ และถอดซ่อมได้
- [ ] ใส่ล้อและ Wheel Hub ตามเพลาจริง
- [ ] ใช้ Revolute joints ทดสอบการหมุนล้อทั้งสอง
- [ ] ตรวจระยะล้อจากแผงข้าง/พื้น/สายไฟ

**ส่งมอบ:** `Drivetrain` v0.3

**ผ่านเมื่อ:** ทั้งสองล้อหมุนโดยไม่ชนตัวถัง และมอเตอร์ถอดได้โดยไม่รื้อทั้งหุ่น

### Phase 4 — กรงจักรตรงกลางด้านหน้า

- [ ] แยกกลุ่มกรงจักรหน้าเป็น subassembly ชัดเจน
- [ ] วางแนวแกนหมุนตามแบบ: ซ้าย–ขวาขวางหน้าหุ่น และให้ศูนย์อยู่ตรงกลาง
- [ ] สร้าง sweep/envelope แสดงบริเวณที่ชิ้นส่วนหมุนผ่านก่อนออกแบบ guard
- [ ] Import/สร้าง placeholder จากชุดอาวุธ/เพลา/ลูกปืนที่ยืนยันแล้ว
- [ ] วางตลับลูกปืน/แผ่น support/ตำแหน่ง ESC และทางเดินสายโดยไม่อยู่ในระยะกวาด
- [ ] ทำ guard และพื้นที่ล็อกอาวุธสำหรับการขนย้าย/เตรียมตัว
- [ ] ไม่ใช้โมเดลตำแหน่งอย่างเดียวอนุมัติการรับแรง ต้องตรวจสอบการออกแบบกับผู้มีประสบการณ์และกติกาการแข่งก่อนผลิต

**ส่งมอบ:** `Front_Spinner` v0.4 พร้อม motion envelope / protective guard

**ผ่านเมื่อ:** ศูนย์อาวุธตรงกลางหน้า, ไม่มีการชนเชิงเรขาคณิต, มีวิธีประกอบ/ล็อก/เข้าถึงชุดอาวุธ

### Phase 5 — ค้อนด้านบน

- [ ] วาง servo/actuator และจุดหมุนค้อนบนแนวกลางของหุ่น
- [ ] สร้าง swing envelope ตั้งแต่ตำแหน่งพักจนสุดช่วงใช้งาน
- [ ] แยก arm, head, pivot, bearing/bushing, motor mount เป็น components
- [ ] ใช้ Revolute joint และ joint limits จำลองช่วงมุม
- [ ] ตรวจการชนกับฝาบน, กรงจักร, สายไฟ, ล้อ และพื้น
- [ ] ทำ stop/lock และระบุชิ้นส่วนรับแรงที่ไม่ควรพิมพ์พลาสติกธรรมดา

**ส่งมอบ:** `Top_Hammer` v0.5 พร้อมภาพตำแหน่งพัก/ยก/กวาดสุด

**ผ่านเมื่อ:** ค้อนหมุนได้ตามช่วงที่กำหนด ไม่มีชิ้นส่วนตัดกัน และถอดซ่อม actuator ได้

### Phase 6 — Electronics / พื้นที่ติดตั้ง PCB

- [ ] สร้าง placeholder ESP32 control PCB ขนาดเริ่มต้น `60 × 50 mm`
- [ ] วาง battery tray + strap + ฝาที่ถอดได้
- [ ] วาง ESC แยกจาก control PCB โดยมีทางเดินไฟกำลังที่ไม่ผ่าน PCB ควบคุม
- [ ] วาง wheel motor driver, buck/BEC, RC receiver และ main disconnect
- [ ] เผื่อ antenna clearance ของ ESP32/receiver และ cable bend radius จากอุปกรณ์จริง
- [ ] ออกแบบตำแหน่งรู PCB/standoffs **หลัง footprint/ขอบบอร์ดถูกล็อก**
- [ ] เมื่อ schematic และ 3D PCB ใน Fusion Electronics เสร็จ ให้นำ 3D PCB เข้า Mechanical Assembly แทน placeholder

**ส่งมอบ:** `Electronics_Packaging` v0.6 + mounting-hole spec

**ผ่านเมื่อ:** เปิดฝา/ถอดแบต/ต่อสาย/เข้าถึงสวิตช์ได้ และไม่มีสายไฟกีดขวางชิ้นส่วนเคลื่อนที่

### Phase 7 — 3D Print + รายละเอียดการประกอบ

- [ ] ทำ battery holder, PCB mount, wire clips และฝาครอบที่ไม่รับแรงกระแทกหลัก
- [ ] เลือก PETG/Nylon/TPU ตามหน้าที่และสภาพการใช้งาน
- [ ] ออกแบบ print orientation, ผนัง, รูน็อต, inserts และ clearance ตามผลทดลองเครื่องพิมพ์จริง
- [ ] ใช้ aluminum สำหรับฐาน/แผ่นโครงรับแรงที่จำเป็น และทบทวนจุดยึดอาวุธก่อนผลิต
- [ ] ทำ exploded view หรือชุดภาพขั้นตอนประกอบ

**ส่งมอบ:** ชุดไฟล์ STL/3MF ของชิ้นส่วนที่พิมพ์และ assembly drawing

**ผ่านเมื่อ:** ไม่มีชิ้นที่ประกอบได้เฉพาะก่อนติดตั้งอุปกรณ์อีกชิ้นหนึ่งโดยถอดไม่ได้

### Phase 8 — ตรวจแบบ/น้ำหนัก/การผลิต

- [ ] `Inspect > Interference` ตรวจความซ้อนทับผิดปกติของ parts
- [ ] ใช้ `Section Analysis` ดูช่องว่างภายใน
- [ ] ใช้ `Inspect > Center of Mass` ร่วมกับ Physical Material/Mass จริงหรือค่าที่วัดได้
- [ ] ตรวจมวลรวมเทียบเป้าหมายและเผื่อมวล fasteners, สายไฟ, เทป, connector, guard
- [ ] ตรวจตำแหน่ง center of mass เมื่อค้อนพักและเมื่อยก
- [ ] เช็กว่ารู/ขอบ/ความหนาทุกจุดมีรายละเอียดผลิตได้จากเครื่องมือที่จะใช้จริง
- [ ] ทบทวนการรับแรง จุดหมุน และความปลอดภัยโดยผู้มีประสบการณ์ก่อนสร้างอาวุธจริง
- [ ] Freeze รายการ part numbers และชิ้นส่วนที่สั่งทำ

**ส่งมอบ:** Final CAD revision, drawing, BOM, mass report, assembly images, manufacturing exports

**ผ่านเมื่อ:** ไม่มี critical interference, น้ำหนักมีหลักฐาน, ซ่อม/ถอดแบตได้ และทุกชิ้นส่วนสั่งผลิตมีขนาดอ้างอิงครบ

## 5. น้ำหนัก (กรอบเป้าหมายสำหรับติดตาม ไม่ใช่ผลคำนวณ CAD)

| Subsystem | งบมวลเป้าหมาย |
|---|---:|
| โครง/เพลต/guard | 320 g |
| ชุดกรงจักรและเพลา | 180 g |
| มอเตอร์กรงจักร + ESC | 100 g |
| ค้อน/servo/กลไก | 180 g |
| มอเตอร์ล้อ + ล้อ + mounts | 300 g |
| แบตเตอรี่ | 110 g |
| PCB/driver/buck/receiver | 120 g |
| น็อตและสายไฟ | 100 g |
| เผื่อแก้แบบ | 90 g |
| **รวม** | **1,500 g** |

อัปเดตตารางนี้เมื่อซื้อ/ชั่งอุปกรณ์จริง; ถ้าลงแข่งคลาส 1 kg ต้องลดแบบและจัดงบมวลใหม่ก่อนผลิต

## 6. Definition of Done (DoD)

- [ ] Parametric Fusion Assembly เปิดและ regenerate ได้ ไม่มี lost reference สำคัญ
- [ ] กรงจักรอยู่ **กึ่งกลางด้านหน้า** อย่างเห็นได้ชัด; wedge อยู่ซ้าย/ขวาของช่องกลาง
- [ ] ค้อนอยู่ด้านบนและมีท่าพัก/ท่ายกที่ไม่ชนส่วนอื่น
- [ ] ล้อซ้าย/ขวาแยกกันและสัมพันธ์กับ ground clearance
- [ ] Chassis / drivetrain / spinner / hammer / electronics / printed parts แยก Components เป็นระบบ
- [ ] ไม่มี hard collision ที่ไม่ตั้งใจเมื่อใช้งาน joint/motion envelopes
- [ ] มีถาดแบต PCB และทางเดินสายไฟที่เข้าถึง/ซ่อมได้
- [ ] มี main disconnect / weapon locks / guard / safety notes ในแบบ
- [ ] BOM ระบุ `confirmed` / `placeholder`, source, quantity, mass, material, manufacturing method
- [ ] มี Top/Front/Side/Isometric + Exploded view และ drawing มิติผลิต
- [ ] Export ชิ้นส่วนตามกระบวนการผลิต (STEP/F3D master, STL/3MF สำหรับพิมพ์, DXF สำหรับแผ่นตามความเหมาะสม)

## 7. ลำดับลงมือทำครั้งแรก (60–90 นาทีแรกเป็นงานเตรียมแบบ ไม่ใช่เวลาเสร็จโปรเจกต์)

1. New Design → ตั้งชื่อ `BattleBot_Master` → ตั้งหน่วย mm
2. ตั้ง User Parameters จากหัวข้อ 3
3. สร้าง 6 component หลัก (`Chassis`, `Drivetrain`, `Front_Spinner`, `Top_Hammer`, `Electronics`, `3D_Printed`)
4. ทำ sketch top view สี่เหลี่ยมขนาด `body_L × body_W` + centerline
5. ทำวงล้อซ้ายขวาและกล่องขนาดอะไหล่แบบหยาบ
6. ทำตำแหน่ง **spinner กลางด้านหน้า** และ swing envelope ของค้อนบน
7. ใส่ battery และ PCB placeholders; ถ้ายัดไม่ลงให้ปรับ parameter ไม่ใช่สร้างรายละเอียดฝาครอบก่อน
8. Save revision `v0.1_layout` แล้วค่อยไป Phase 2

## 8. ลิงก์คู่มือ Autodesk สำหรับงานนี้

- Parameters: https://help.autodesk.com/view/fusion360/ENU/?contextId=SLD-MODIFY-PARAMETERS
- Joints / As-Built Joint: https://help.autodesk.com/cloudhelp/ENU/Fusion-Assemble/files/ASM-CREATE-AS-BUILT-JOINT.htm
- Inspect / Interference / Center of Mass: https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-INSPECT-TOOLS.htm
- นำ 3D PCB เข้า Mechanical Assembly: https://help.autodesk.com/cloudhelp/ENU/Fusion-ECAD/files/Fusion_ECAD_ecad_tutorials_mech_to_ecad_3_create_design_html.htm

---

**ข้อควรระวัง:** แบบจำลองนี้มีอาวุธเคลื่อนที่และแบตเตอรี่ LiPo การทดสอบอาวุธจริงต้องมีสนาม/กล่องป้องกันที่เหมาะสม ระบบหยุดเมื่อสัญญาณขาด สวิตช์ตัดไฟ และล็อกอาวุธ โดยต้องตรวจความปลอดภัยตามกติกาของสนามที่จะลงแข่งขัน
