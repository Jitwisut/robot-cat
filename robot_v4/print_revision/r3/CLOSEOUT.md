# ปิด Robot V4 R3 — งานที่ทำได้และงานรอของจริง

| ขั้น | ผู้ทำ | ผลส่งมอบ / ผ่านเมื่อ | สถานะ |
|---|---|---|---|
| อะไหล่หลัก/สำรอง | ผมค้น; ผู้ใช้ประสาน | SKUครบ exact stock/price/ไฟฟ้า/ขนาดและมวล; ไม่มีรายการไม่ทราบ | เว็บปิดSURPASSรุ่นย่อย/คู่มือ GNBราคาไทย และJB361/MDD3Aทางเลือกเพิ่มแล้ว; ยังล็อกทั้งระบบไม่ได้ |
| ราคาอย่างน้อยพิมพ์2+โลหะ2 | ผมเตรียม; ผู้ใช้ส่งร้าน | quoteบริการครบ ทุกหมวดรวม300<=6000 | RFQ4ฉบับพร้อมส่ง ยังไม่มีquote |
| วัดและแก้แบบ | ร้านวัด; ผมปรับ | ภาพฉลาก/caliper/scale/ขาสายและเครื่องมือ; actual27กลุ่ม | R3ปรับแบต+SURPASSตามข้อมูลเว็บเป็นDRAFT; actual27กลุ่มรอวัดชิ้นที่จะได้รับ |
| มวล/สมดุล | ผมคำนวณ; ร้านslice | target1900/15%; hard2000/10%; ผลใหม่ทุกbuild | ยังพลาดเป้าหมาย ห้ามถือbatterymoveแก้สำเร็จ |
| ทดลองรอบ1→2 | ร้าน; ผมสร้างตามค่าที่เลือก | 0.1mmcoupon→ดุมยาง/hinge/carrier/lidจริง,4shafts,5cycles,profile+ภาพ/วิดีโอ | ไม่มีผลจริง |
| CAD/STL/โลหะ/จำลอง | ผม | BREP/sweep+1mm/wedge−2..14° mesh closed/manifold Z0 L/R และstatic/spin/turn/inverted | ผลร่างในoutput_R3 ยังใช้nominal |
| slicer | ร้าน | ทุกpartจำนวนวัสดุ/profile/support/brim และน้ำหนัก;projectไฟล์ | รอร้าน/profile |
| ปล่อยPRINT_READY | ผม | หลักฐานทุกgateครบและราคายืนยัน ไม่ต้องมีหุ่นเต็มก่อนพิมพ์ | ระบบแยกแล้ว ยังไม่ผ่าน |
| ASSEMBLED_CHECKED | ผู้ใช้ประกอบ/ตรวจ | เครื่องมือเข้าถึง แบตแน่น ฉนวน สายปลอดส่วนหมุน โรเตอร์คล่อง ฝาเปิดได้ ไม่มีเจาะเพิ่มไม่ระบุ;มวล/loadที่ชั่ง | แยกตรวจหลังผลิต |

หากพบราคาหรือมวลเกิน: เทียบแหล่งซื้อ/รวมผลิต/ลดวัสดุที่ไม่รับแรงและไม่ป้องกันก่อน ทวนCAD/mesh/simและtrialจุดที่กระทบ ห้ามลดผนังบานพับ1.6mm mounts/bearing battery retention หรือcutoff ห้ามใช้ballastเพิ่มมวลเพื่อให้rearloadผ่านโดยไม่ตรวจทั้งระบบ

ยังไม่ซื้อ ไม่พิมพ์ชิ้นใหญ่ ไม่แจ้งว่าPRINT_READYจนขนาดจริง/fit/profile/ราคา/น้ำหนักผ่าน รายงานนี้จะปรับตามหลักฐานที่ร้านส่งกลับ คำตอบรอในmeasurement_log.json,fit_log.json,quote_comparison.json

## หมวดงบที่ต้องรวม

- `drive_drivers`: 2บอร์ดต่อมอเตอร์ขนาน2ตัวต่อฝั่ง
- `sensors`: IMU Hall temperature magnets resistors
- `disconnect_fuse`: link/collar interface fuse holder
- `wiring_connectors`: power/signals connectors insulation ties straps foam (แยกpadIMUที่ชั่งต่อbody)
- `fasteners_inserts`: BOMสกรูทุกจุด รวมM8axle/nuts/washers/insert14/สำรอง
- `measurement_fit_service`: caliper/scale/photos/trial fit/insert5cycles
- `trial_prints`: round1+round2+support/finish/retrial
- `full_prints`: STL26ชิ้น/17แบบ support/inserts/finish
- `shipping`: ภาษีที่ยังไม่รวมและส่งทุกแหล่ง อย่านับซ้ำ
- `contingency`: เงินเผื่อ300ภายในเพดาน6000
