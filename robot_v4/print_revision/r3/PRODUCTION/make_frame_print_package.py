"""PETG frame print package for the PC body: the 29 PETG prints that join the PC panels and do not depend on motors.

User request 2026-10-21 (FRAME_PRINT_DECISION.json): production files for the frame now. Writes
PRODUCTION/RELEASE_<date>/05_PRINT_FRAME_PETG(.zip) next to the 2026-10-19 packages, which are not touched:
  STL/        cleats, skirt brackets, weapon side blocks, wedgelet blocks (pc_body/output/PETG_STL, copied unchanged, SHA-checked)
  COUPONS/    the insert and hinge-pin coupons the shop prints in the same job and checks BEFORE the frame parts
  ORDER_FRAME_PETG_TH.pdf with per-group print settings, the trial hole sizes and the hold point
Insert holes are still trial values (M2.5 D3.4, M3 D4.6, hinge pin + 0.2), so the order stops the job if the coupons reject them.
Run from the repository: .venv-v4-print/bin/python robot_v4/print_revision/r3/PRODUCTION/make_frame_print_package.py
"""
import json
import shutil
import zipfile
from pathlib import Path

import make_release_packages as rp
from make_release_packages import PC, PC_REPORT, R3_TRIAL, check_copy, e, mm, pdf

HERE = rp.HERE
DEC = json.loads((HERE / 'FRAME_PRINT_DECISION.json').read_text())
OUT = HERE / ('RELEASE_' + DEC['date'])
FOLDER = OUT / '05_PRINT_FRAME_PETG'
STAMP = 'ส่งผลิตตามคำขอผู้สั่ง — %s · มีจุดหยุดตรวจ coupon ก่อนพิมพ์' % DEC['date']
FRAME_PREFIXES = ('V4_R3PC_Cleat_', 'V4_R3PC_Skirt_Bracket_', 'V4_R3PC_Weapon_Side_Block_', 'V4_R3PC_Wedgelet_Block_')
HELD_PREFIXES = ('V4_R3PC_Motor_',)


def settings(group):
    # Wedgelet blocks carry the hinge bore: R3 SHOP_HANDOFF "บานพับ/ดุม" row, not the PC-body default.
    if group['title'].startswith('Wedgelet_Block'):
        return 'PETG · 4 walls · infill ≥80% · รูแกนบานพับวางแนวนอนตามไฟล์ · เนื้อรอบรูบานพับต้องไม่ต่ำกว่า 1.6 มม.'
    if 'เนื้อเต็ม' in group['infill']:
        return 'PETG · 4 walls · infill 100%'
    return 'PETG · 4 walls · infill 80% (ห้ามเกิน 80%)'


def main():
    if not mm.REPORT['build_sha256'].startswith(DEC['basis']['r3_build_sha256_prefix']):
        raise RuntimeError('R3 build changed since the frame print request')
    parts = {p['file']: p for p in PC_REPORT['petg_parts']}
    frame = sorted(f for f in parts if f.startswith(FRAME_PREFIXES))
    held = sorted(f for f in parts if f.startswith(HELD_PREFIXES))
    if len(frame) != DEC['frame_part_count'] or len(frame) + len(held) != len(parts):
        raise RuntimeError('PETG list changed: %d frame, %d held, %d total' % (len(frame), len(held), len(parts)))
    groups = [g for g in PC_REPORT['petg_groups'] if all(f in frame for f in g['files'])]
    if sorted(f for g in groups for f in g['files']) != frame:
        raise RuntimeError('frame groups do not cover the frame files')

    if FOLDER.exists():
        shutil.rmtree(FOLDER)
    FOLDER.with_suffix('.zip').unlink(missing_ok=True)
    files = []
    for f in frame:
        check_copy(PC / 'PETG_STL' / f, FOLDER / 'STL' / f, files, FOLDER)
        if files[-1]['sha256'] != parts[f]['sha256']:
            raise RuntimeError(f + ' differs from pc_body_report')
    coupons = sorted((PC / 'TRIAL_ONLY').glob('Insert_M2.5_Hole_*.stl')) + sorted(
        p for p in R3_TRIAL.glob('*_PETG.stl') if p.name.startswith(('Insert_Hole_', 'Insert_Horizontal_', 'Pin_Horizontal_')))
    for p in coupons:
        check_copy(p, FOLDER / 'COUPONS' / p.name, files, FOLDER)

    hole = DEC['trial_holes_mm']
    rows = ''.join('<tr><td>%s</td><td class="m">%s</td><td class="n">%d</td><td>%s</td><td>%s</td><td class="n">%.2f</td></tr>' % (
        g['id'], '<br>'.join(e(f) for f in g['files']), g['qty'], e(g['inserts'].replace('_L4_OD', ' L4 OD')), e(settings(g)), g['mass']) for g in groups)
    inserts = {}
    for g in groups:
        size, n = g['inserts'].split(' ×')
        inserts[size] = inserts.get(size, 0) + int(n) * g['qty']
    total_g = sum(g['mass'] * g['qty'] for g in groups)
    body = rp.header('ใบสั่งพิมพ์ชิ้น PETG โครงรถ (ตัวถัง PC) — ร้านพิมพ์ 3D').replace(e(rp.STAMP), e(STAMP)) + '''
<p>ชิ้น PETG ที่ยึดแผ่น PC เข้าด้วยกัน %(nf)d ไฟล์ ไฟล์ละ 1 ชิ้น ไฟล์วางท่าพิมพ์ไว้แล้ว (Z ขึ้น ฐานพิมพ์ Z=0) <b class="bad">ห้ามหมุน ห้าม mirror ห้ามย่อ/ขยาย</b>
ชิ้นซ้าย/ขวาเป็นคนละไฟล์อยู่แล้ว แผ่น PC สั่งแยกที่ร้าน CNC (<span class="m">02_CNC_PC.zip</span>)</p>
<h2>1. ขั้นตอนบังคับ — พิมพ์ coupon ก่อน แล้วหยุดตรวจ</h2><ol>
<li>พิมพ์ทุกไฟล์ในโฟลเดอร์ <span class="m">COUPONS/</span> (%(nc)d ไฟล์) ด้วย<b>เครื่อง เส้น PETG และ profile เดียวกับที่จะใช้พิมพ์ชิ้นจริง</b> (4 walls, infill 80%%, layer 0.2, หัวฉีด 0.4) บันทึกชื่อ profile</li>
<li>กด heat-set insert ตัวจริงลง coupon ด้วยหัวแร้ง:
<b>M2.5 L4 OD3.6</b> ลงรู <span class="m">Insert_M2.5_Hole_D3.2–3.6</span> และ <b>M3 L4 OD5</b> ลงรู <span class="m">Insert_Hole_D4.4–4.8</span> กับ <span class="m">Insert_Horizontal_D4.4–4.8</span></li>
<li>สอดแกน Ø3 (แกนบานพับจากร้านกลึง หรือเหล็กเพลา Ø3 ขัดเงา) ลง <span class="m">Pin_Horizontal_Clearance_000–030</span></li>
<li><b>ชิ้นจริงในชุดนี้ใช้รู: insert M2.5 Ø%(m25)g · insert M3 Ø%(m3)g · รูแกนบานพับ = Ø แกน + %(pin)g</b> (ตรงกับ coupon <span class="m">Insert_M2.5_Hole_D%(m25)g</span>,
<span class="m">Insert_Hole_D%(m3)g</span>, <span class="m">Insert_Horizontal_D%(m3)g</span>, <span class="m">Pin_Horizontal_Clearance_%(pinc)03d</span>)
— ผ่านเมื่อ insert ลงตรง ไม่หมุนเมื่อขันสกรู ไม่มีรอยร้าวรอบรู และแกนหมุนได้คล่องไม่คลอน</li>
<li><b class="bad">ถ้า coupon ขนาดดังกล่าวไม่ผ่าน ให้หยุด อย่าพิมพ์ชิ้นจริง</b> แจ้งผู้สั่งว่าขนาดไหนผ่าน ผู้สั่งจะสร้างไฟล์ใหม่ด้วยขนาดนั้น (ผนังรอบ insert ของตัวยึดมุมบางเพียง 1.3 มม. รูคับจะทำให้ชิ้นแตก)</li></ol>
<h2>2. รายการชิ้นจริง (%(nf)d ชิ้นต่อคัน)</h2>
<table><tr><th>ID</th><th>ไฟล์ STL</th><th>จำนวน</th><th>insert ต่อชิ้น</th><th>ตั้งค่าพิมพ์</th><th>g ตามแบบ/ชิ้น</th></tr>%(rows)s</table>
<ul><li>ขายึด skirt (P15–P19): วางหน้าตัดลงฐานตามไฟล์ · รู M2 Ø1.6 สำหรับสกรูเกลียวปล่อย ห้ามคว้านขยาย</li>
<li>บล็อกยึดแผ่นตั้งอาวุธ (P05–P06): insert M3 ×6 ต่อชิ้น (ลงพื้น 4 + ลง bulkhead 2 แนวนอน)</li>
<li>Wedgelet block (P20) ใช้แทน <span class="m">V4_R3_Wedgelet_Block_PETG_L/R</span> ในโฟลเดอร์ DRAFT เดิม ห้ามพิมพ์ไฟล์เดิม</li>
<li>เลือกตำแหน่ง seam ไม่ให้อยู่บนหน้าที่แนบแผ่น PC · ตรวจไม่ให้มี stringing ค้างในรู</li></ul>
<h2>3. Heat-set insert ที่ต้องใช้ (ผู้สั่งหรือร้านจัดหา)</h2><ul>%(ins)s<li>เผื่อสำรองอย่างน้อย 10%% และใช้ insert ล็อตเดียวกับที่กดลง coupon</li></ul>
<h2>4. ไม่อยู่ในใบนี้ (HOLD รอวัดมอเตอร์ขับจริง)</h2><ul><li class="m">%(held)s</li>
<li>Wedgelet carrier ของ R3 ยังไม่อยู่ในใบนี้ รอผล coupon แกนบานพับ</li></ul>
<h2>5. สิ่งที่ร้านต้องส่งกลับ</h2><ul>
<li>รูปและผลกด insert/สอดแกนของทุก coupon พร้อมชื่อ profile ที่ใช้</li>
<li><b>น้ำหนักจาก slicer และน้ำหนักชั่งจริงของทุกชิ้น (g)</b> — รวมตามแบบ %(tg).1f g รถเหลือเผื่อน้ำหนักน้อยมาก ถ้ารวมเกินแบบเกิน 5%% ให้แจ้งก่อน</li>
<li>ใบเสนอราคา แยกค่าพิมพ์ coupon ค่าพิมพ์ชิ้นจริง ค่ากด insert (ถ้ามี) VAT และค่าส่ง</li></ul>''' % dict(
        nf=len(frame), nc=len(coupons), rows=rows, m25=hole['M2.5_insert'], m3=hole['M3_insert'], pin=hole['hinge_pin_clearance'],
        pinc=round(hole['hinge_pin_clearance'] * 100), tg=total_g, held=', '.join(e(f) for f in held),
        ins=''.join('<li>%s: <b>%d ตัว</b></li>' % (e(k.replace('_L4_OD', ' L4 OD')), v) for k, v in sorted(inserts.items())))
    pdf(body, FOLDER / 'ORDER_FRAME_PETG_TH.pdf')
    files.append({'file': 'ORDER_FRAME_PETG_TH.pdf', 'source': 'generated', 'sha256': mm.sha(FOLDER / 'ORDER_FRAME_PETG_TH.pdf')})

    (FOLDER / 'SHA256SUMS.txt').write_text(''.join('%s  %s\n' % (f['sha256'], f['file']) for f in files))
    (FOLDER / 'manifest.json').write_text(json.dumps({
        'status': 'RELEASED_WITH_COUPON_HOLD_POINT', 'date': DEC['date'], 'decided_by': DEC['decided_by'],
        'r3_build_sha256': mm.REPORT['build_sha256'], 'shop': '3D print PETG (frame)', 'trial_holes_mm': hole,
        'frame_parts': frame, 'coupons': [p.name for p in coupons], 'held': held, 'inserts_needed': inserts,
        'design_mass_g': round(total_g, 2), 'files': files}, ensure_ascii=False, indent=2) + '\n')
    with zipfile.ZipFile(FOLDER.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as z:
        for path in sorted(FOLDER.rglob('*')):
            if path.is_file():
                z.write(path, Path(FOLDER.name) / path.relative_to(FOLDER))
    print(json.dumps({'zip': str(FOLDER.with_suffix('.zip').relative_to(rp.R3)), 'frame_parts': len(frame), 'coupons': len(coupons),
                      'held': len(held), 'inserts': inserts, 'design_mass_g': round(total_g, 2)}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
