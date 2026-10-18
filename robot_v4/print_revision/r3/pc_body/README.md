# V4 R3: ตัวถัง PC (ทางเลือกแทนถาด TPU) — rev 3 แบบผลิตที่ยังมีเงื่อนไข (BLOCKED)

แบบนี้ใช้รถ R3 ทั้งคัน เปลี่ยนแค่ถาดและฝาที่พิมพ์ TPU เป็นแผ่นโพลีคาร์บอเนตตัดด้วย CNC router:

- PC 3 มม. 5 แผ่น
- bulkhead PC 2 มม. 2 แผ่น

งานที่ถาด TPU เคยทำ ย้ายไปอยู่ในชิ้น PETG ใหม่ 35 ชิ้นที่ยึดด้วย heat-set insert ได้แก่ แท่นมอเตอร์ twin saddle lid bar บล็อกอาวุธ ตัวยึดมุม และขายึด skirt

- แนวแกนเหมือน R3 ตาม [AGENTS.md](../../../../AGENTS.md): Z ขึ้น พื้นที่ Z=0 หัวรถชี้ +Y หน่วยมม.
- ไฟล์ R3 ใน `../output_R3` ไม่ได้ถูกแก้โดยสคริปต์นี้

## ผลล่าสุด

build นี้ใช้ R3 ฉบับใหม่ในโฟลเดอร์ (ยังไม่ commit): wedgelet 1.6 มม. และรูต๊าปข้างแผ่นตั้ง y 75

| | R3 ถาด TPU | **PC rev 3 คาดว่าจะเกิดจริง** | PC rev 3 กฎ 80% |
|---|---:|---:|---:|
| มวลรวม | 1973.05 g | **1977.64 g** (เผื่อถึง 2000: **22.36 g**) | 1967.05 g |
| โหลดล้อหลัง | 10.615% | **10.913%** | 10.736% |

- **ความแข็งแรง:**
  - เป้าหมายคือรถตก 0.5 ม. (250 g) ทั้งตกหงายและตกคว่ำ และอาวุธตีด้วยแรง 2.6 kN ตาม CALC_R3
  - **SF ต่ำสุดของตัวถังคือ 1.82** จากเกณฑ์ ≥1.5
  - เพลามอเตอร์ขับ 4 มม. เป็นจุดที่ตั้งใจให้พังก่อน ตก 0.5 ม. แล้วเพลาจะงอ แต่ตัวถังไม่พัง
- **ตรวจ CAD:** ไม่มีชิ้นชนกัน สกรู 77 ตัวลงรูตรง เนื้อรอบ insert บางสุด 1.3 มม. STL ปิดผิวครบ DXF ไม่พบ error
- **ยังห้ามผลิตชุดจริง:** ต้องปิดเงื่อนไขในใบสั่งผลิตให้ครบก่อน ได้แก่ วัดมอเตอร์จริง วัดความหนาแผ่น ชั่งสกรู ทดสอบดึง insert และปล่อย R3 ฉบับใหม่

## ไฟล์

- [`CALC_PC_BODY.md`](CALC_PC_BODY.md): ใบคำนวณน้ำหนัก สมดุล ทางเลือกลดน้ำหนัก เป้าหมายแรงกระแทก และ SF ทุกเส้นทางแรง
- `output/PRODUCTION/WORK_ORDER_PC_BODY_TH.pdf`: ใบสั่งผลิตภาษาไทย พร้อมเงื่อนไขก่อนผลิต
- `output/PRODUCTION/DRAWINGS_PC_BODY.pdf`: แบบ 2D มีขนาดกำกับ 27 แผ่น (แยกไฟล์ต่อชิ้นอยู่ใน `output/PRODUCTION/drawings/`)
- [`SHOP_HANDOFF_PC_TH.md`](SHOP_HANDOFF_PC_TH.md): ใบขอราคาสำหรับร้าน CNC และร้านพิมพ์
- `output/PC_3mm/`: DXF/STEP ของแผ่น PC (`PC3_*` หนา 3 มม., `PC2_*` หนา 2 มม.) และไฟล์วางแผ่นรวม `PC3_NEST_ALL.dxf` (432 × 365) กับ `PC2_NEST_ALL.dxf` (174 × 72)
- `output/PETG_STL/`: ชิ้น PETG 37 ไฟล์ (ใหม่ 35 + wedgelet block ที่ออกแบบใหม่ 2)
- `output/TRIAL_ONLY/`: coupon รู insert M2.5 (Ø3.2–3.6)
- `output/ROBOT_V4_R3_PC_BODY.step`: ชุดประกอบทั้งคัน
- `output/assembly_preview.png`, `output/panels_preview.png`: ภาพจาก CAD
- `output/pc_body_report.json`: ตัวเลขทุกค่า

## สร้างใหม่

รันจาก root ของ repository (ใช้เวลาประมาณ 1–2 นาที เพราะต้องสร้าง PDF ด้วย Chrome):

```sh
.venv-v4-print/bin/python robot_v4/print_revision/r3/pc_body/build_pc_body.py
```

ค่าที่ยังเป็นค่าประมาณและต้องแทนด้วยค่าจริง:

- `FASTENER_G`: มวลสกรูและ insert
- `CAP`: แรงดึง insert และสกรู M2
- `T` / `T_BULK`: ความหนาแผ่น
- `PC_DENSITY`: ความหนาแน่น PC
- `PAD_T`: ความหนาแผ่น EPDM
- ขนาดรู insert: `fit.insert_hole_diameter_mm` ใน `inputs_R3.json`
- ขนาดมอเตอร์ขับ: ค่า `actual` ใน `inputs_R3.json`
