# Robot V4 Print R3 — DRAFT / ยังไม่พร้อมส่งพิมพ์ชุดเต็ม

**แนวโมเดลที่ผู้ใช้ล็อก:** Z ขึ้น, พื้น XY ที่ Z=0 ใต้ล้อ, ด้านหน้า +Y ห้ามหมุนหรือสลับแกน เปิด [`ROBOT_V4_PRINT_R3_FUSION_Z_UP.f3d`](output_R3/DRAFT/ROBOT_V4_PRINT_R3_FUSION_Z_UP.f3d) เพื่อใช้ Top/Home ที่ถูกต้อง ตรวจเปิดซ้ำแล้วกริดอยู่ใต้หุ่น ตำแหน่งชิ้นส่วนทั้ง 70 ชิ้นและขอบเขตโมเดลไม่เปลี่ยน ข้อกำหนดถาวรอยู่ใน [`AGENTS.md`](../../../AGENTS.md)

R3 ใช้ R2 เป็นฐาน เก็บ R1/R2 เดิมไว้ เริ่มรอบ10ต.ค.2026 ผู้ใช้ยังไม่ได้ซื้ออะไหล่ ข้อมูลเว็บ/nominal/actualแยกกัน ชุดนี้มี STEP/STL/DXF และชิ้นทดลองสำหรับตรวจและขอราคา ยังไม่ใช่ชุดผลิตที่รับรอง

การเปลี่ยนแบบ: เลือกSURPASSC3536V2 1300KVเป็นผู้สมัครหลักตามคู่มือ; body34.5mm/109g/เพลาØ4ยื่น20mm/รู19และ25M3 ปรับรอก ฐานยึด และค่าจำลองแล้ว; Øบ่า/ความลึกเกลียวยังnominal แบตผู้สมัครหลักเป็น GNB8503S80A ขนาดเว็บ63×30×24mm/80±3g ([แหล่งผู้ผลิต](https://www.gaoneng.shop/products/gaoneng-gnb-3s-11.1v-850mah-80c-xt30-lipo-battery)) ทำถาดและตำแหน่งตามพารามิเตอร์ใหม่; actualยังไม่วัด เพิ่มcouponยางทีละ0.1mmและสร้างcouponแยกหากเพลาทั้ง4ต่างกัน ระบบ4WD TPU95A/PETG ตัวถังชิ้นเดียวและเนื้อบานพับ>=1.6mmคงเกณฑ์ตามแผน

อ่าน [ข้อมูลเว็บที่ยืนยันและแก้ CAD เพิ่ม](WEB_CONFIRMED.md), [งานปิดและผู้รับผิดชอบ](CLOSEOUT.md), [มวลและสมดุล](MASS_BALANCE.md), [อะไหล่หลัก/สำรอง](PARTS_SELECTION.md), [ร้าน2+2และคำขอราคา](VENDORS.md), [งบ](PROCUREMENT.md), [สถานะ CAD/จำลอง](output_R3/STATUS.md), [ข้อมูลกลาง](inputs_R3.json), [ทดลองสองรอบ](FIT_TEST.md), [คู่มือประกอบ](ASSEMBLY.md) และ [งานโลหะ](MACHINING.md)

## ลำดับทำงาน

1. ผู้ใช้ส่ง RFQ/คำขอข้อมูล รับราคาครบและ SKU/stock/ไฟฟ้าเข้ากันได้ ก่อนซื้อ ภายใน6000รวมเงินเผื่อ300
2. ผู้ใช้ซื้อและส่งอะไหล่ให้ร้านวัด/ชั่ง/ถ่ายหลักฐานตามmeasurement_log.json แล้วนำactualเข้าinputs_R3 สร้างใหม่
3. ร้านล็อกเครื่องวัสดุ/profileแล้วพิมพ์couponรอบ1; นำค่าที่เลือกสร้างใหม่ ร้านทดลองสวมเต็มรอบ2และบันทึกfit_log.json; ต้องเพลาครบ4และฝา5รอบ
4. ตรวจpreflight CAD mesh interference/wedge/rotor; ร้านsliceทุกชิ้น น้ำหนักจริง+slicer/สมดุล/งบครบ แล้วจำลองล่าสุด หากพลาดเป้า1900g/15%ให้ผู้ใช้ตัดสินใจจากส่วนเผื่อก่อนปล่อย โดยขั้นต่ำ2000g/10%ข้ามไม่ได้
5. `--release` ผ่านจึงสร้างRELEASEDสถานะPRINT_READY ผู้ใช้ประกอบแล้วบันทึกassembly actual/หลักฐานทั้งหมดแยกเป็นASSEMBLED_CHECKED

## สร้าง/ตรวจ

รันจาก root repository:

```sh
.venv-v4-print/bin/python robot_v4/print_revision/build.py --inputs robot_v4/print_revision/r3/inputs_R3.json --output robot_v4/print_revision/r3/output_R3
.venv/bin/python robot_v4/print_revision/simulate.py --inputs robot_v4/print_revision/r3/inputs_R3.json --output robot_v4/print_revision/r3/output_R3
.venv-v4-print/bin/python robot_v4/print_revision/build.py --inputs robot_v4/print_revision/r3/inputs_R3.json --output robot_v4/print_revision/r3/output_R3
.venv-v4-print/bin/python robot_v4/print_revision/package.py --inputs robot_v4/print_revision/r3/inputs_R3.json --output robot_v4/print_revision/r3/output_R3 --docs robot_v4/print_revision/r3
```

เพิ่ม `--release` เมื่อหลักฐานครบเท่านั้น การรันตอนนี้ต้องปฏิเสธและไม่สร้างRELEASED คู่มือ/ไฟล์ผลิตและหลักฐานถูกเก็บพร้อมSHAในชุดปล่อย `fit`/`preflight`/`electrical`/`slicer`ต้องตรงbuildล่าสุด รวมทั้งprofile hashของไฟล์ตั้งค่าจริง ห้ามเปลี่ยนscale/รู/ท่าในslicerโดยไม่ทวนtrial

หลังประกอบ:

```sh
.venv-v4-print/bin/python robot_v4/print_revision/readiness.py --inputs robot_v4/print_revision/r3/inputs_R3.json --release-report robot_v4/print_revision/r3/output_R3/RELEASED/release_report.json
```

คำสั่งตรวจประกอบต้องมีผลหุ่นเต็มจริงและชั่งมวล/โหลดล้อหลัง; ไม่ใช้ผลcouponแทนสถานะประกอบ ไม่มีเฟิร์มแวร์หรือการรับรองแข่งในชุดนี้ สคริปต์+JSONเป็นแหล่งพารามิเตอร์ STEPไม่มีFusionfeaturetimeline

ยอดเก่า2268บาทเป็นข้อมูลบางรายการ8ต.ค. ปัจจุบันเพิ่มSURPASS840และGNB605ตรงvariantเป็นยอดที่ทราบ3,223บาท; กันเผื่อ300แล้วเหลือเพดาน2,477สำหรับ17หมวดรอราคา ยังไม่ใช่ราคาทั้งโครงการและยังไม่อนุมัติซื้อหรือผลิต
