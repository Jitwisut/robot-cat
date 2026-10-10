# ตรวจไฟฟ้าก่อนล็อกชุดอะไหล่ / R3

สถานะ UNVERIFIED; ยังไม่ได้ทดสอบกระแสของมอเตอร์ หรือจ่ายไฟให้ชิ้นจริง แผนนี้ไม่พัฒนาเฟิร์มแวร์

แบตคง3S>=850mAh ตรวจที่3Sเต็ม12.6Vด้วย ไม่ใช้ค่าCโฆษณาแทนการตรวจ battery/XT30/wire/fuse/holder path กระแสรับได้จริง ผลต้องอยู่ `electrical/specs` แยกจาก `electrical/nominal` ซึ่งใช้จำลองร่างเท่านั้น

| รายการ | หลักฐานที่ต้องขอ / ตรวจ |
|---|---|
| มอเตอร์ขับ | รุ่นย่อย4ตัว, no-load/rated rpm/voltage, torque-current, start/stallแต่ละตัว; รวมคู่ที่ต่อขนานต่อบอร์ด |
| บอร์ดขับ | รุ่นPCBจริงและขนาดรวมterminal, Rlim/ค่าตั้งlimit, continuous currentที่PCB/อุณหภูมิจริง; ระบุวิธีจำกัดกระแสด้วยวงจรจริง |
| อาวุธ+ESC | KVตรงรุ่น, winding resistance/no-load current, spin-up/current-duration/profileที่ตรวจจริง, 3Sและรอบ/ESC ratingตรงSKU ไม่สมมติESCมีcurrent limiter |
| แบต+ตัดไฟ | 3S/ความจุ, voltage sag, battery/connector/wire/fuse/loop worst-case current path และขนาดพื้นที่บริการ |

เกณฑ์ซอฟต์แวร์ R3: limitบอร์ดไม่เกินพิกัดcontinuousที่ยืนยัน; กระแสเริ่มของคู่มอเตอร์ไม่เกินlimitที่ผ่านจริง; spin-upไม่เกินESC continuous; ผลรวมspin-up+2×drive limitไม่เกินpower path; บันทึกstart/stallและหลักฐานทั้งหมด เมื่อไม่ผ่านให้เปลี่ยนบอร์ดหรือระบบจำกัดกระแสแล้วทวนlayout/มวล/ต้นทุน ไม่กรอก3.6A peakเป็นcontinuous

MuJoCo R3อ่านKV/resistance/I0/voltage/drive-rpm/torqueจากspecsที่ยืนยัน มิฉะนั้นใช้nominalและระบุว่าunverified แรงขับจำกัดตามcurrent-limit/คู่stallเมื่อข้อมูลครบ แบบจำลองแรงบิดและแรงสัมผัสไม่ทำนายความร้อน/การเสียหาย และผลผ่านการเคลื่อนที่ไม่ได้ยืนยันกระแสจริง R3ใช้KV1300/R=.038Ω/I0=1.8Aจากคู่มือSURPASSC35V2 (R3-08) เก็บแหล่งต่อparameter; แบต3S850จากGaoneng และESC40Aจากร้านตรงSKU ค่าstart/stall/spin-upจริงและpower pathยังว่าง ไม่ใช้max50Aของมอเตอร์แทนกระแสที่วัดได้หรือถือเป็นcurrentlimiter

MDD3Aเป็นทางเลือกมีพิกัด3Acontinuous/ช่อง และ5Apeak<5sตามdatasheet;2บอร์ด/4ช่องช่วยแยกมอเตอร์แต่ไม่ปิดการจำกัดกระแสหรือstallโดยอัตโนมัติ rail5V200mAไม่ใช้เลี้ยงESP32วิทยุโดยไม่ตรวจ ไม่มีการเปลี่ยนแท่นDRVในCADร่างนี้
