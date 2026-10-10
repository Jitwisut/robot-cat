# V4 Print R2 — อ้างอิงข้อมูลร้านไทย / DRAFT

ปรับ R1 จากข้อมูลร้านที่อ่านได้วันที่8ต.ค.2026 เก็บ output ของ R1 เดิมไว้ทั้งหมด ไม่ใช่ชุดผลิตและยังไม่ยืนยันว่าอะไหล่ทุกตัวมีอยู่ในไทย

อ่าน [รายการจัดหาและงบ](PROCUREMENT.md), [ชุดขอข้อมูลและราคา](SUPPLIER_REQUESTS.md), [รายการร้านและข้อจำกัด](THAI_PARTS_RESEARCH.md), [สถานะ CAD](output_R2/STATUS.md), [ข้อมูลกลาง R2](inputs_R2.json), [คู่มือประกอบ](ASSEMBLY.md) และ [ผลทดลองที่ต้องทำ](FIT_TEST.md)

- ESCเป็นHobbywing Skywalker40AV2 PN80060140 ABCFlying68×25×8mm/39g อุปกรณ์ถูกจัดใหม่และขอบแท่นเปิดช่องสายสองปลาย
- ESP32Mikroelec51.5×28.5mm; ความสูง14mmเป็นnominalเดิม
- InsertFriendRobotM3×4×5 OD5/length4mm; coupons4.4–4.8mmทีละ0.1 ฝาใช้M3×6เป็นค่าเริ่มต้น
- JGAOD25,IMUXY21×16,6088×22×7ตรงค่าที่แบบเดิมใช้ ข้อมูลอื่นของชิ้นเหล่านี้ยังไม่ยืนยัน
- ไม่เปลี่ยนbattery850mAhเป็น650/720โดยเงียบๆ และไม่ใช้สเปกมอเตอร์longshaftแทนSKUที่ยังไม่เลือก
- catalog.valuesใช้ก่อนnominal; actualที่วัดจริงมีลำดับสูงสุด การปล่อยผลิตยังต้องactual/evidence/fit/shop/slicer/cost/assemblyครบ

## สร้างและตรวจซ้ำ

รันจากrootของrepository:

```sh
.venv-v4-print/bin/python robot_v4/print_revision/build.py --inputs robot_v4/print_revision/catalog/inputs_R2.json --output robot_v4/print_revision/catalog/output_R2
.venv/bin/python robot_v4/print_revision/simulate.py --inputs robot_v4/print_revision/catalog/inputs_R2.json --output robot_v4/print_revision/catalog/output_R2
.venv-v4-print/bin/python robot_v4/print_revision/build.py --inputs robot_v4/print_revision/catalog/inputs_R2.json --output robot_v4/print_revision/catalog/output_R2
.venv-v4-print/bin/python robot_v4/print_revision/preview.py --inputs robot_v4/print_revision/catalog/inputs_R2.json --output robot_v4/print_revision/catalog/output_R2
.venv-v4-print/bin/python robot_v4/print_revision/package.py --inputs robot_v4/print_revision/catalog/inputs_R2.json --output robot_v4/print_revision/catalog/output_R2 --docs robot_v4/print_revision/catalog
```

เพิ่ม `--release` ให้buildเฉพาะเมื่อหลักฐานครบ สถานะนี้ต้องปฏิเสธการปล่อยผลิต

## ข้อจำกัด

เว็บไม่ยืนยันขนาดรายตัว ความคลาดเคลื่อนหลังพิมพ์ การบัดกรี/โค้งสาย เวลารันจริง การรับแรงชน หรือเฟิร์มแวร์ เลย์เอาต์และมวลที่ผ่านในคอมพิวเตอร์ยังมีnominalของมอเตอร์ แบต และอุปกรณ์อื่นอยู่ ต้องปรับอีกครั้งเมื่อเลือกรุ่นได้ครบ

ชุดCAD/STEPนี้เปิดในFusionได้แต่ไม่มีFusionfeaturetimeline สคริปต์และJSONเป็นแหล่งพารามิเตอร์

## อัปเดตรอบจัดหา 8 ต.ค. 2026

แยกแบตFPVONLY SKU-08923 ได้ราคา490บาทและข้อมูลเว็บคงเหลือ9; ยังไม่มีdimension/massที่ใช้แก้CADได้ ไม่สร้างrevisionเรขาคณิตใหม่ในรอบนี้ ยอดบางรายการ2,268บาท เหลือเพดาน3,732บาท ยังขาดมอเตอร์อาวุธ จอย เครื่องชาร์จ งานกลึง งานพิมพ์และรายการย่อย จึงยังไม่ผ่านงบหรือปล่อยผลิต

ข้อมูลนี้แยกในprocurement_inputs/report ไม่เปลี่ยนactual/verified/costในinputs_R2; ราคาเว็บไม่ได้เป็นใบเสนอราคา ส่วนstockร้านMechashop DCM068แสดง2ตัวและต้องยืนยันครบ4
