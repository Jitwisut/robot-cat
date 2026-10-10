**ข้อมูลเว็บR3อยู่ในcatalog.values; actualทั้ง27รายการยังว่าง**

อ่านTHAI_PARTS_RESEARCH.mdและsources.jsonเพื่อแยกค่าหน้าเว็บจากnominal/ค่าที่วัดจริง ค่าเว็บไม่ใช่หลักฐานverified ราคาเป็นรายการซื้อครบเพื่อวางงบ; สถานะของที่มีอยู่แล้วต้องยืนยันก่อนหักค่าใช้จ่าย

# ตารางวัดอะไหล่ — V4 Print R3

ทุกแถวด้านล่างยังรอวัดจริง ค่า nominal ใน JSON ใช้สร้างต้นแบบเท่านั้น กรอกค่ามม./กรัมใน actual และบันทึกไฟล์หลักฐานก่อนเปลี่ยน verified เป็น true

| รายการ | ต้องวัด/ยืนยัน | ค่า actual | หลักฐาน |
|---|---|---|---|
| drive_Front_L | length_mm, diameter_mm, mass_g, shaft_diameter_mm, shaft_flat_depth_mm, shaft_projection_mm, boss_diameter_mm, boss_projection_mm, mount_holes_yz_mm, mount_thread_diameter_mm, mount_thread_depth_mm | รอ | รอ |
| drive_Front_R | length_mm, diameter_mm, mass_g, shaft_diameter_mm, shaft_flat_depth_mm, shaft_projection_mm, boss_diameter_mm, boss_projection_mm, mount_holes_yz_mm, mount_thread_diameter_mm, mount_thread_depth_mm | รอ | รอ |
| drive_Rear_L | length_mm, diameter_mm, mass_g, shaft_diameter_mm, shaft_flat_depth_mm, shaft_projection_mm, boss_diameter_mm, boss_projection_mm, mount_holes_yz_mm, mount_thread_diameter_mm, mount_thread_depth_mm | รอ | รอ |
| drive_Rear_R | length_mm, diameter_mm, mass_g, shaft_diameter_mm, shaft_flat_depth_mm, shaft_projection_mm, boss_diameter_mm, boss_projection_mm, mount_holes_yz_mm, mount_thread_diameter_mm, mount_thread_depth_mm | รอ | รอ |
| weapon_motor | length_mm, diameter_mm, mass_g, shaft_diameter_mm, shaft_projection_mm, boss_diameter_mm, mount_holes_yz_mm, mount_thread_diameter_mm, mount_thread_depth_mm | รอ | รอ |
| battery | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| esc | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| esp32 | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| drv_left | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| drv_right | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| imu | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| disconnect | dimensions_mm, mass_g, connector_keepout_mm, connector_x_offset_mm, connector_side | รอ | รอ |
| bearing_608 | outer_diameter_mm, inner_diameter_mm, width_mm, mass_g_each | รอ | รอ |
| hinge_pin | diameter_mm, length_mm, mass_g_pair | รอ | รอ |
| insert_m3 | outer_diameter_mm, length_mm | รอ | รอ |
| nut_m3 | across_flats_mm, height_mm | รอ | รอ |
| hall_sensor | dimensions_mm, mass_g, magnet_diameter_mm, magnet_thickness_mm | รอ | รอ |
| dead_shaft | mass_g, diameter_mm, length_mm, head_across_flats_mm, head_thickness_mm, nut_across_flats_mm, nut_thickness_mm, washer_outer_diameter_mm, washer_inner_diameter_mm, washer_thickness_mm, thread_length_mm | รอ | รอ |
| clips | mass_g | รอ | รอ |
| belt | mass_g | รอ | รอ |
| skid | mass_g | รอ | รอ |
| rotor_screws | mass_g | รอ | รอ |
| baseline_fasteners | mass_g | รอ | รอ |
| revision_fasteners | mass_g | รอ | รอ |
| wiring | mass_g | รอ | รอ |
| aux_sensors | mass_g | รอ | รอ |
| straps | mass_g | รอ | รอ |

## วิธีเก็บข้อมูล

- มอเตอร์ขับ: วัดแยกทั้ง 4 ตัว ระยะรูใช้ศูนย์รู บันทึกพิกัด dy/dz จากศูนย์เพลา พร้อมเกลียว ความลึกเกลียว และความยาวเพลาที่ยื่นจากหน้ามอเตอร์
- มอเตอร์อาวุธ: วัดหน้าที่ประกบเพลตและปลายเพลา รวมพื้นที่หัวน็อต/สายไฟ ความยาวมอเตอร์ในแบบวัดระหว่างหน้าประกบและท้าย ไม่รวมเพลา
- บอร์ด/แบต/ESC: วัดรวมส่วนยื่น และทำ connector_keepout_mm เป็นกล่อง XYZ สำหรับหัวปลั๊กและช่วงสายที่ต้องดัด ระบุ connector_side front/rear และ connector_x_offset_mm จากกลางอุปกรณ์
- Insert: วัด OD ที่ปุ่มนูนสูงสุด ความยาว และชนิดเกลียว ยืนยันด้วยชิ้นทดลอง ไม่เลือกขนาดรูจาก OD เพียงอย่างเดียว
- น้ำหนัก ancillary: ชั่งชุดสกรู แหวน น็อต สาย และสายรัดจริงแยกหมวด ห้ามใส่ค่า allowances เป็น actual โดยไม่ชั่ง
- หลักฐานเก็บใน evidence/; ชื่อไฟล์ใน JSON อ้างจาก print_revision/r3/ เช่น evidence/drive_front_left_measurement.jpg
- ขนาดที่เปลี่ยนแล้วทำให้ layout ไม่ผ่านจะต้องปรับแบบและตรวจใหม่ โปรแกรมจะไม่ย่ออุปกรณ์ให้พอดีโดยอัตโนมัติ

## งบรวม

| หมวด | จำนวน | ราคารวมจริง บาท | หลักฐาน |
|---|---:|---:|---|
| drive_motors | 4 | รอ | รอ |
| weapon_motor | 1 | รอ | รอ |
| esc | 1 | รอ | รอ |
| battery | 1 | รอ | รอ |
| drive_drivers | 1 | รอ | รอ |
| sensors | 1 | รอ | รอ |
| controller | 1 | รอ | รอ |
| gamepad | 1 | รอ | รอ |
| measurement_fit_service | 1 | รอ | รอ |
| charger | 1 | รอ | รอ |
| disconnect_fuse | 1 | รอ | รอ |
| wiring_connectors | 1 | รอ | รอ |
| bearings | 1 | รอ | รอ |
| belt | 1 | รอ | รอ |
| metal_cutting | 1 | รอ | รอ |
| machining | 1 | รอ | รอ |
| fasteners_inserts | 1 | รอ | รอ |
| hinge_pins_clips | 1 | รอ | รอ |
| skid | 1 | รอ | รอ |
| trial_prints | 1 | รอ | รอ |
| full_prints | 1 | รอ | รอ |
| shipping | 1 | รอ | รอ |
| contingency | 1 | รอ | รอ |

ผู้ใช้ยืนยันว่าไม่ได้ซื้ออะไหล่เลย ต้องรวม ESP32 และเครื่องชาร์จในต้นทุนด้วย; กรอก 0 บาทได้เฉพาะรายการที่ภายหลังมีหลักฐานว่าเป็นของที่มีอยู่จริง ไม่ถือว่ารายการที่ยังไม่มีราคามีต้นทุนศูนย์ ราคาต้องรวมชิ้นทดลอง เก็บงาน ค่าส่ง และเผื่อรายการเล็ก เป้าหมายยอดไม่เกิน 6,000 บาท

R3 ต้องชั่งชิ้นโลหะ/foamที่แยกใน mass.parts แล้วลง `manufacturing.actual_mass_g_by_body` พร้อมหลักฐานและ verified; ค่าdensityCADไม่แทนมวลโลหะที่ยืนยัน ห้ามนับfoamซ้ำในstraps/foam/insulation
