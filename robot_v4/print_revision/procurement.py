"""Render a partial web-price budget; never populate production cost evidence."""
import argparse
import json
import math
from pathlib import Path


def evaluate(data, sources):
    records = {row['id']: row for row in sources['records']}
    rows = []
    for item in data['items']:
        row = dict(item)
        price = records.get(item.get('source_id'), {}).get('unit_price_thb')
        quantity = item['required_quantity']
        pack = item['units_per_pack']
        if quantity <= 0 or pack <= 0:
            raise ValueError('Positive quantities and pack sizes required')
        if price is not None and (isinstance(price, bool) or not isinstance(price, (int, float)) or not math.isfinite(price) or price < 0):
            raise ValueError('Invalid catalog price: ' + item['id'])
        packs = math.ceil(quantity / pack)
        row.update(purchase_packs=packs, unit_price_thb=price,
                   catalog_total_thb=None if price is None else packs * price)
        rows.append(row)
    known = sum(row['catalog_total_thb'] for row in rows if row['catalog_total_thb'] is not None)
    missing = [row['id'] for row in rows if row['catalog_total_thb'] is None]
    reserve=data.get('reserve_thb',0)
    if isinstance(reserve,bool) or not isinstance(reserve,(int,float)) or not math.isfinite(reserve) or reserve<0:raise ValueError('Invalid reserve')
    return dict(revision=data.get('revision','V4_PRINT_R2'),price_basis_date=data.get('price_basis_date',data['date']),planned_reserve_thb=reserve, date=data['date'], status='PARTIAL_WEB_BUDGET_NOT_QUOTE',
                purchased_hardware=False, approved_to_order=False,
                budget_thb=data['budget_thb'], known_catalog_subtotal_thb=known,
                remaining_budget_ceiling_thb=data['budget_thb'] - known - reserve,
                unpriced_items=missing, project_cost_complete=not missing,
                production_cost_verified=False, items=rows)


def render(report, sources):
    records = {row['id']: row for row in sources['records']}
    revision=report['revision'].rsplit('_',1)[-1]
    lines = ['# รายการจัดหาและงบบางส่วน — '+report['revision'], '',
             'รวบรวมวันที่ '+report['date']+'; ราคาที่ไม่ได้ตรวจใหม่อ้างอิงวันที่ '+report['price_basis_date'] + '; ผู้ใช้ยังไม่ได้ซื้ออะไหล่ ไม่มีคำสั่งซื้อหรือใบเสนอราคาที่ร้านยืนยัน', '',
             '**ยอดราคาที่พบ ' + f"{report['known_catalog_subtotal_thb']:,.0f}" +
             ' บาท; เพดานที่เหลือ ' + f"{report['remaining_budget_ceiling_thb']:,.0f}" +
             ' บาทหลังกันเงินเผื่อ '+str(report['planned_reserve_thb'])+' บาท** ยอดนี้ยังไม่ใช่ราคาทั้งโครงการ รายการรอราคาไม่ถูกนับเป็นศูนย์ และไม่ถือว่างบผ่าน', '',
             '| รายการ | จำนวนใช้ / หน่วยขาย | ยอดเว็บ (บาท) | แหล่ง / สิ่งต้องยืนยัน |',
             '|---|---:|---:|---|']
    for row in report['items']:
        source = records.get(row.get('source_id'))
        label = 'รอรุ่น/ราคา' if not source else '[' + source['id'] + '](' + source['url'] + ')'
        total = 'รอราคา' if row['catalog_total_thb'] is None else f"{row['catalog_total_thb']:,.0f}"
        quantity = f"{row['required_quantity']} / {row['units_per_pack']} ต่อหน่วยขาย"
        lines.append('| ' + row['label'] + ' | ' + quantity + ' | ' + total + ' | ' + label + '; ' + row['note'] + ' |')
    lines += ['', '## ลำดับปิดข้อมูล', '',
              '1. ตรวจข้อมูลที่ยังขาดของมอเตอร์ขับครบ 4 ตัว มอเตอร์อาวุธ แบต และบอร์ดขับก่อนซื้อ; R3 อ่าน WEB_CONFIRMED.md สำหรับข้อมูลเว็บที่ปิดแล้ว และใช้ SUPPLIER_REQUESTS.md เฉพาะส่วนที่เว็บไม่ระบุ',
              '2. ขอราคางานกลึง/ตัดโลหะ ชิ้นทดลอง และชุดพิมพ์เต็มจากไฟล์ DRAFT revision เดียวกันก่อนสั่งงาน รวม support เก็บงาน ภาษีและส่ง',
              '3. ล็อก SKU และรับของมาวัด กรอก actual/evidence ใน inputs_'+revision+'.json แล้วสร้าง revision ใหม่; ค่าจากเว็บไม่ใช่ actual',
              '4. พิมพ์เฉพาะชิ้นทดลองที่ได้รับอนุมัติและใช้ profile เดียวกับชุดเต็ม บันทึกผลตาม FIT_TEST.md; หาก layout เปลี่ยนใช้ชุดทดลองจาก build ใหม่',
              '5. ปิด slicer/มวล/โหลดล้อหลัง/ราคาและผลตรวจประกอบล่วงหน้าทุกข้อ; ประกอบจริงตรวจแยกหลังผลิตก่อนปล่อยชุดใหญ่', '',
              'ราคาสกรูใช้ BOM ใน ASSEMBLY.md และต้องรวมแหวน น็อตล็อก น็อตฝัง สำรอง และ insert 14 จุด (ซื้อ 2 ถุง ถุงละ 10 ตัว)',
              ('R3เลือก SURPASS C3536 V2 1300KV SKU02373 เป็นผู้สมัครหลัก ใช้ราคา840ตรงvariant และแก้รอกØ4/รูฐาน/มวล/โมเดลตามคู่มือ; ESC40Aยังต้องตรวจคู่มอเตอร์ที่ระบุmax50A' if revision=='R3' else 'ตัวเลือก SURPASS C3536 V2 1300KV จาก UDSHOBBY ยังเป็นทางเลือกแยก ไม่ใช้ราคา 840 บาทเติมยอดของ DYS 1250KV และไม่เปลี่ยนเพลา/รอก/ESC โดยไม่มีข้อมูลตรงรุ่น'),
              'มวล/สมดุลล่าสุดอ่าน output_'+revision+'/STATUS.md; ต้องถึงเป้า1900g/15% หรือมีการตัดสินใจจากส่วนเผื่อ โดยขั้นต่ำ2000g/10%ยังบังคับและต้องใช้slicer+มวลจริง',
              'จอยและเครื่องชาร์จเป็นต้นทุนโครงการ แม้ไม่อยู่บนตัวหุ่น ไม่สมมติว่าเป็นของที่มีแล้ว', '',
              'สร้างรายงานซ้ำ: `.venv-v4-print/bin/python robot_v4/print_revision/procurement.py --catalog robot_v4/print_revision/'+('r3' if revision=='R3' else 'catalog')+'` จาก root ของ repository', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--catalog', type=Path, default=Path(__file__).resolve().parent / 'catalog')
    args = parser.parse_args()
    folder = args.catalog
    sources = json.loads((folder / 'sources.json').read_text())
    data = json.loads((folder / 'procurement_inputs.json').read_text())
    report = evaluate(data, sources)
    (folder / 'procurement_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (folder / 'PROCUREMENT.md').write_text(render(report, sources))
    print(json.dumps({key: value for key, value in report.items() if key != 'items'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
