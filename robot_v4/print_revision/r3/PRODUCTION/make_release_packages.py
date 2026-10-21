"""Per-shop RELEASE packages for the parts the user released on RELEASE_DECISION.json['date'].

Reads RELEASE_DECISION.json (what is released, what is held, accepted risks) and writes
PRODUCTION/RELEASE_<date>/ with one folder + one zip per shop:
  01_LASER_METAL   laser-cut metal plates (R3 output_R3/DRAFT/metal) + drawings/work order PDF
  02_CNC_PC        PC sheet panels (pc_body/output/PC_3mm) + RELEASED drawings + Thai release order
  03_LATHE         turned parts (hub, spacers, hinge pins) + Thai order
  04_PRINT_COUPONS insert / pin / D-shaft / tyre test coupons only + Thai instructions
and README_TH.md with the shop map and the HOLD list. Source files are copied unchanged and SHA-checked.
Run from the repository: .venv-v4-print/bin/python robot_v4/print_revision/r3/PRODUCTION/make_release_packages.py
"""
import collections
import html
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import cadquery as cq
import ezdxf

HERE = Path(__file__).resolve().parent
R3 = HERE.parent
sys.path.insert(0, str(R3 / 'RFQ'))
import make_metal_production_package as mm  # noqa: E402

DEC = json.loads((HERE / 'RELEASE_DECISION.json').read_text())
OUT = HERE / ('RELEASE_' + DEC['date'])
PC = R3 / 'pc_body' / 'output'
PC_REPORT = json.loads((PC / 'pc_body_report.json').read_text())
R3_TRIAL = R3 / 'output_R3' / 'TRIAL_ONLY'
CHROME = mm.CHROME
e = html.escape
STAMP = 'อนุมัติผลิต (RELEASED) — %s' % DEC['date']

CSS = '''@page{size:A4;margin:12mm}
body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:9.5pt;color:#111;line-height:1.5}
.stamp{border:2px solid #070;color:#070;font-weight:700;padding:3px 10px;display:inline-block;font-size:11pt}
h1{font-size:15pt;margin:6px 0}h2{font-size:11pt;margin:12px 0 4px;border-bottom:1px solid #999}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #aaa;padding:2px 5px;text-align:left;vertical-align:top}th{background:#eee}
.m{font-family:Menlo,monospace;font-size:8pt}.n{text-align:right;font-variant-numeric:tabular-nums}.bad{color:#b00;font-weight:700}
li{margin-bottom:2px}'''


def check_copy(src, dst, files, root):
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    if mm.sha(src) != mm.sha(dst):
        raise RuntimeError('copy changed ' + str(src))
    files.append({'file': str(dst.relative_to(root)), 'source': str(src.relative_to(R3)), 'sha256': mm.sha(dst)})


def pdf(body, path):
    tmp = path.with_suffix('.html')
    tmp.write_text('<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (CSS, body))
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(path), tmp.as_uri()],
                   check=True, capture_output=True, timeout=120)
    tmp.unlink()


def header(title):
    return ('<span class="stamp">%s</span><h1>%s</h1><p>Robot V4 R3 · R3 build <span class="m">%s</span> · หน่วย มม. · '
            'แนวแกน Z ขึ้น หัวรถ +Y · ใช้ไฟล์ DXF/STEP เป็นต้นฉบับ ห้ามวัดจากภาพ</p>') % (e(STAMP), e(title), mm.REPORT['build_sha256'][:12])


def finish(folder, files, manifest_extra):
    for f in files:
        if not (folder / f['file']).exists():
            raise RuntimeError('missing ' + f['file'])
    (folder / 'SHA256SUMS.txt').write_text(''.join('%s  %s\n' % (f['sha256'], f['file']) for f in files))
    (folder / 'manifest.json').write_text(json.dumps({'status': 'RELEASED', 'date': DEC['date'], 'r3_build_sha256': mm.REPORT['build_sha256'],
                                                      **manifest_extra, 'files': files}, ensure_ascii=False, indent=2) + '\n')
    with zipfile.ZipFile(folder.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as z:
        for path in sorted(folder.rglob('*')):
            if path.is_file():
                z.write(path, Path(folder.name) / path.relative_to(folder))


# ------------------------------------------------------------------ 01 laser metal
def laser():
    folder = OUT / '01_LASER_METAL'
    files, released, held = [], [], []
    accepted = DEC['accepted_risks']
    for part in mm.PARTS:
        blockers = []
        a = mm.analyse(part, blockers)
        remaining = [b for b in blockers if not any(key in b for key in accepted)]
        if part['id'] in DEC['released']['laser_metal']:
            if remaining:
                raise RuntimeError('%s is on the release list but still blocked: %s' % (part['id'], remaining))
            released.append(a)
        else:
            held.append({'id': part['id'], 'file': part['file'], 'blockers': blockers})
    total, rear = mm.REPORT['mass']['total_g'], mm.REPORT['mass']['rear_static_load_fraction']
    if total > mm.INPUTS['design']['mass_limit_g'] or rear < mm.INPUTS['design']['rear_load_min']:
        raise RuntimeError('R3 build fails mass/rear-load gates')
    if not mm.REPORT['build_sha256'].startswith(DEC['basis']['r3_build_sha256_prefix']):
        raise RuntimeError('R3 build changed since the release decision')
    for a in released:
        for ext, sub in (('.dxf', 'DXF'), ('.step', 'STEP')):
            check_copy(mm.SRC / (a['file'] + ext), folder / sub / (a['file'] + ext), files, folder)
    mass_rows = {r['part']: r for r in mm.REPORT['mass']['parts']}
    for src_name, dst_name in (('Wedgelet_L', 'Wedgelet_R'), ('Skirt_Plate_08mm_R', 'Skirt_Plate_08mm_L')):
        shape = cq.importers.importStep(str(mm.SRC / (src_name + '.step'))).val().mirror('YZ', (0, 0, 0))
        if max(abs(p - q) for p, q in zip(shape.Center().toTuple(), mass_rows[dst_name]['centroid_mm'])) > 0.01:
            raise RuntimeError(dst_name + ' mirror does not match the build')
        dst = folder / 'STEP' / ('DERIVED_%s_mirror_of_%s.step' % (dst_name, src_name))
        shape.exportStep(str(dst))
        files.append({'file': str(dst.relative_to(folder)), 'source': 'mirror of %s.step across x=0' % src_name, 'sha256': mm.sha(dst)})
    notes = list(accepted.values()) + ['ชิ้น %s ยังไม่อยู่ในชุดนี้ (HOLD รอวัดมอเตอร์)' % ', '.join(h['id'] + ' ' + h['file'] for h in held)]
    mm.write_pdf(folder, released, [], 'RELEASED', STAMP, files, notes=notes, pdf_name='WORK_ORDER_LASER_TH.pdf')
    files[-1]['source'] = 'generated'
    finish(folder, files, {'shop': 'laser metal', 'released': [a['id'] + ' ' + a['file'] for a in released], 'held': held})
    return released, held


# ------------------------------------------------------------------ 02 CNC PC panels
def csk_sizes(dxf_path):
    doc = ezdxf.readfile(dxf_path)
    return collections.Counter(round(2 * c.dxf.radius, 2) for c in doc.modelspace() if c.dxftype() == 'CIRCLE' and c.dxf.layer.startswith('CSK'))


def cnc_pc():
    folder = OUT / '02_CNC_PC'
    files = []
    wanted = DEC['released']['cnc_pc_panels']
    panels = [p for p in PC_REPORT['panels'] if p['file'][:-4] in wanted]
    if len(panels) != len(wanted):
        raise RuntimeError('PC panel list does not match the decision')
    drawings = {d['name']: d for d in PC_REPORT['production']['drawings']}
    for p in panels:
        name = p['file'][:-4]
        check_copy(PC / 'PC_3mm' / p['file'], folder / 'DXF' / p['file'], files, folder)
        check_copy(PC / 'PC_3mm' / p['step'], folder / 'STEP' / p['step'], files, folder)
        if mm.sha(folder / 'DXF' / p['file']) != p['sha256']:
            raise RuntimeError(p['file'] + ' differs from pc_body_report')
        d = drawings[name]
        check_copy(PC / d['file'], folder / 'DRAWINGS' / Path(d['file']).name, files, folder)
        p['csk'] = csk_sizes(PC / 'PC_3mm' / p['file'])
    for nest in ('PC3_NEST_ALL.dxf', 'PC2_NEST_ALL.dxf'):
        check_copy(PC / 'PC_3mm' / nest, folder / 'NEST' / nest, files, folder)
    acc = DEC['pc_sheet_thickness_acceptance_mm']
    rows = ''.join('<tr><td>%s</td><td class="m">%s / %s</td><td class="n">%.1f</td><td class="n">%s</td><td class="n">%d</td><td>%s</td><td>%s</td><td class="n">1</td><td class="n">%.1f</td></tr>' % (
        p['id'], e(p['file']), e(p['step']), p['thickness_mm'], ' × '.join('%g' % v for v in p['flat_size_mm']), p['holes'],
        ', '.join('Ø%g ×%d' % (d, n) for d, n in sorted(p['csk'].items())) or '–',
        'ร่องลึก 2.0 ±0.1 จากด้านล่าง' if p['pockets'] else '–', p['mass_g']) for p in panels)
    sheets = PC_REPORT['nest_sheet_mm']
    body = header('ใบสั่งผลิต — แผ่นตัวถังโพลีคาร์บอเนต (ร้าน CNC router)') + '''
<h2>1. รายการชิ้นงาน (ไฟล์ละ 1 ชิ้น รวม %(n)d ชิ้นต่อคัน ห้าม mirror ไฟล์ใด)</h2>
<table><tr><th>ID</th><th>ไฟล์ DXF / STEP</th><th>หนา</th><th>ขนาดแผ่น</th><th>รูทะลุ</th><th>Countersink 90° (ด้าน FACE UP)</th><th>ร่อง</th><th>จำนวน</th><th>g ตามแบบ</th></tr>%(rows)s</table>
<p>แบบมีขนาดของแต่ละแผ่นอยู่ในโฟลเดอร์ <span class="m">DRAWINGS/</span> (PC01–PC07) ไฟล์วางรวมบนแผ่นอยู่ใน <span class="m">NEST/</span>:
แผ่น 3 มม. ใช้พื้นที่ %(s3)s มม. · แผ่น 2 มม. ใช้พื้นที่ %(s2)s มม.</p>
<h2>2. วัสดุ</h2><ul>
<li>แผ่นโพลีคาร์บอเนตตัน (solid PC) เกรดทั่วไป เช่น Makrolon GP หรือ Lexan 9030 หรือเทียบเท่า สีใดก็ได้</li>
<li><b class="bad">ห้ามใช้อะคริลิค ห้ามใช้แผ่นลอนหรือแผ่นกลวง (twin-wall)</b></li>
<li><b>วัดความหนาแผ่นจริงด้วยคาลิปเปอร์ 4 จุดก่อนตัด:</b> แผ่น 3 มม. ต้องอยู่ใน %(a3)s มม. แผ่น 2 มม. ต้องอยู่ใน %(a2)s มม.
<b class="bad">ถ้าอยู่นอกช่วง ให้หยุดและแจ้งผู้สั่งก่อนตัด</b> (แผ่นหนาเกินทำให้รถหนักเกิน 2000 g)</li></ul>
<h2>3. วิธีตัดและเก็บงาน</h2><ul>
<li><b class="bad">CNC router เท่านั้น ห้ามตัดด้วยเลเซอร์</b> (ขอบ PC ไหม้ เหลือง เปราะ และมีควันพิษ) แนะนำดอก O-flute Ø3 มุมด้านในใน DXF เป็น R1.5 แล้ว</li>
<li>Layer ใน DXF: <span class="m">CUT_OUTER</span> ขอบนอก · <span class="m">CUT_INNER</span> รูและช่องทะลุ · <span class="m">CSK90_FACE_UP</span> วงนอกของ countersink 90° ที่ด้าน FACE UP ·
<span class="m">POCKET_2.0mm_UNDERSIDE</span> ร่องใต้ฝา ลึก 2.0 ±0.1 มม. วัดจากด้านล่าง (เหลือเนื้อ 1.0 มม.) · <span class="m">NOTES</span> ข้อความ ไม่ต้องตัด</li>
<li>ด้าน FACE UP ของแต่ละแผ่นระบุไว้ในแบบ (เช่น ผนังข้าง = ด้านในรถ, ฝา = ด้านบน)</li>
<li>พิกัด: ขอบนอก ±0.2 · ตำแหน่งรู ±0.15 · รู +0.1/−0 · ทั่วไป ISO 2768-m</li>
<li>ลบคมทุกขอบ คงฟิล์มกันรอยไว้ระหว่างตัด <b class="bad">ห้ามใช้ทินเนอร์ อะซีโตน น้ำยาล็อกเกลียว หรือลนไฟขัดขอบ</b> (PC แตกลายงา)</li></ul>
<h2>4. สิ่งที่ร้านต้องส่งกลับ</h2><ul>
<li>ยี่ห้อ/เกรดแผ่น และความหนาที่วัดได้ 4 จุดของแต่ละแผ่นวัสดุ</li>
<li><b>น้ำหนักจริงของทุกแผ่นหลังตัด (g)</b> พร้อมรูปถ่ายบนตาชั่ง</li>
<li>ใบเสนอราคา/ใบแจ้งหนี้ แยกค่าวัสดุ ค่าตัด countersink ร่อง ลบคม VAT และค่าส่ง</li></ul>
<p class="m">ชิ้น PETG ของตัวถัง PC ยังไม่อยู่ในใบนี้ (HOLD รอผล coupon insert และการวัดมอเตอร์)</p>''' % dict(
        n=len(panels), rows=rows, s3=' × '.join('%g' % v for v in sheets['3.0']), s2=' × '.join('%g' % v for v in sheets['2.0']),
        a3='–'.join('%g' % v for v in acc['3.0']), a2='–'.join('%g' % v for v in acc['2.0']))
    pdf(body, folder / 'ORDER_CNC_PC_TH.pdf')
    files.append({'file': 'ORDER_CNC_PC_TH.pdf', 'source': 'generated', 'sha256': mm.sha(folder / 'ORDER_CNC_PC_TH.pdf')})
    finish(folder, files, {'shop': 'CNC router PC', 'released': [p['id'] + ' ' + p['file'] for p in panels]})
    return panels


# ------------------------------------------------------------------ 03 lathe
LATHE_STEP = {'Beater_Hub_6061_D30': 1, 'Spacer_Inner_ID8.2_OD11': 1, 'Spacer_Outer_L_ID8.2_OD11': 1, 'Spacer_Outer_R_ID8.2_OD11': 1}


def lathe():
    folder = OUT / '03_LATHE'
    files = []
    for name in LATHE_STEP:
        if name not in DEC['released']['lathe']:
            raise RuntimeError(name + ' not released')
        check_copy(mm.SRC / (name + '.step'), folder / 'STEP' / (name + '.step'), files, folder)
    # Dimensions stated in the order are checked against the STEP.
    hub = cq.importers.importStep(str(mm.SRC / 'Beater_Hub_6061_D30.step')).val()
    hd = sorted({round(2 * f._geomAdaptor().Cylinder().Radius(), 2) for f in hub.Faces() if f.geomType() == 'CYLINDER'})
    if hd != [2.5, 22.0, 27.0, 30.0] or abs(hub.BoundingBox().xlen - 40) > 0.01:
        raise RuntimeError('hub STEP no longer matches the order text: %s' % hd)
    for name, length in (('Spacer_Inner_ID8.2_OD11', 26), ('Spacer_Outer_L_ID8.2_OD11', 7), ('Spacer_Outer_R_ID8.2_OD11', 7)):
        s = cq.importers.importStep(str(mm.SRC / (name + '.step'))).val()
        if abs(s.BoundingBox().xlen - length) > 0.01:
            raise RuntimeError(name + ' length changed')
    pin = mm.INPUTS['hardware']['hinge_pin']['nominal']
    body = header('ใบสั่งผลิต — ชิ้นกลึง (ร้านกลึง CNC)') + '''
<h2>1. ดุมใบอาวุธ — Beater_Hub_6061_D30.step (1 ชิ้น)</h2><ul>
<li>วัสดุอลูมิเนียม 6061-T6 · โตนอก Ø30 · ยาว 40</li>
<li>รูใน Ø22 ทะลุตลอด สำหรับลูกปืน 608 สองตัวอัดที่ปลายทั้งสองข้าง: เป้าหมาย N7 (Ø21.972–21.993)
<b>ผู้สั่งจะส่งลูกปืน 608 จริง 2 ตัวไปให้ ร้านต้องลองอัดกับลูกปืนตัวจริงแล้วยืนยันว่าแน่น ลูกปืนไม่หมุนฟรีในดุม</b></li>
<li>ร่องสายพานกลมตรงกลาง กว้าง 6 ลึก 1.5 (โคนร่อง Ø27)</li>
<li>รูต๊าป M3 ที่ปลายทั้งสองข้าง ข้างละ 4 รู วง PCD 26 ที่มุม 45°/135°/225°/315° เจาะ Ø2.5 ลึก 8 ต๊าป M3-6H ให้ลึกที่สุด</li>
<li>หน้าปลายทั้งสองขนานกัน ตั้งฉากกับรู · ลบคม · ห้ามชุบผิวในรูลูกปืน</li></ul>
<h2>2. แหวนรองแกน (spacer) — อลูมิเนียม 6061</h2>
<table><tr><th>ไฟล์</th><th>รูใน</th><th>โตนอก</th><th>ยาว</th><th>จำนวน</th></tr>
<tr><td class="m">Spacer_Outer_L_ID8.2_OD11.step</td><td class="n">Ø8.2</td><td class="n">Ø11</td><td class="n">7 ±0.05</td><td class="n">1</td></tr>
<tr><td class="m">Spacer_Outer_R_ID8.2_OD11.step</td><td class="n">Ø8.2</td><td class="n">Ø11</td><td class="n">7 ±0.05</td><td class="n">1</td></tr>
<tr><td class="m">Spacer_Inner_ID8.2_OD11.step</td><td class="n">Ø8.2</td><td class="n">Ø11</td><td class="n">26 ±0.05</td><td class="n">1</td></tr></table>
<p>หน้าทั้งสองของทุกตัวต้องขนานกัน (ชุดนี้บีบวงในลูกปืน: 7 + 7 + 26 + 7 + 7 = 54 ระหว่างแผ่นตั้ง)</p>
<h2>3. แกนบานพับ (hinge pin) — 2 ชิ้น</h2><ul>
<li>เหล็กเพลา Ø%(d)g ยาว %(l)g (ใช้เหล็กเพลาขัดเงาสำเร็จรูปได้)</li>
<li>ร่องแหวนล็อก E-clip ตาม DIN 6799 ขนาด 2.3: ร่อง Ø2.3 กว้าง 0.64 ห่างปลายด้านนอก 0.9 มม. (ใน CAD ร่องอยู่ที่ 55.5–56.1 มม. จากปลายด้านใน)</li>
<li>ลบคมปลายทั้งสองข้าง</li></ul>
<h2>4. ไม่อยู่ในใบนี้</h2><ul><li>รอกมอเตอร์ Motor_Pulley_D30_1to1 — HOLD รอวัดเพลามอเตอร์อาวุธจริง</li></ul>
<h2>5. สิ่งที่ร้านต้องส่งกลับ</h2><ul><li>ขนาดที่วัดได้ของรูดุม Ø22 ทั้งสองปลาย และผลลองอัดลูกปืน</li><li>น้ำหนักของทุกชิ้น (g)</li>
<li>ใบเสนอราคา แยกค่าวัสดุ ค่ากลึง ค่าต๊าป VAT และค่าส่ง</li></ul>''' % dict(d=pin['diameter_mm'], l=pin['length_mm'])
    pdf(body, folder / 'ORDER_LATHE_TH.pdf')
    files.append({'file': 'ORDER_LATHE_TH.pdf', 'source': 'generated', 'sha256': mm.sha(folder / 'ORDER_LATHE_TH.pdf')})
    finish(folder, files, {'shop': 'lathe', 'released': DEC['released']['lathe'], 'held': ['Motor_Pulley_D30_1to1']})


# ------------------------------------------------------------------ 04 print coupons
def coupons():
    folder = OUT / '04_PRINT_COUPONS'
    files = []
    picks = sorted(p for p in R3_TRIAL.glob('*.stl')
                   if p.name.startswith(('Insert_Hole_', 'Insert_Horizontal_', 'Pin_', 'D_Shaft_', 'Tyre_Ring_')) and 'TPU95A' not in p.name
                   or p.name.startswith('Tyre_Ring_'))
    for p in picks:
        check_copy(p, folder / 'STL' / p.name, files, folder)
    for p in sorted((PC / 'TRIAL_ONLY').glob('Insert_M2.5_*.stl')):
        check_copy(p, folder / 'STL' / p.name, files, folder)
    groups = collections.Counter(f['file'].split('/')[-1].rsplit('_', 1)[0].split('_D')[0].split('_ID')[0] for f in files)
    rows = ''.join('<tr><td class="m">%s</td></tr>' % e(f['file'].split('/')[-1]) for f in files)
    body = header('ใบสั่งพิมพ์ชิ้นทดลอง (coupon) — ร้านพิมพ์ 3D') + '''
<p>พิมพ์ไฟล์ละ 1 ชิ้น ไฟล์วางท่าพิมพ์ไว้แล้ว (Z ขึ้น ฐานพิมพ์ Z=0) <b>ใช้เครื่อง วัสดุ และ profile เดียวกับที่จะใช้พิมพ์ชิ้นจริง</b> แล้วบันทึกชื่อ profile ไว้</p>
<ul><li>ชื่อไฟล์ที่ลงท้าย <span class="m">_PETG</span> พิมพ์ PETG: 4 walls, infill 80%%, layer 0.2, หัวฉีด 0.4</li>
<li>ชื่อไฟล์ที่ลงท้าย <span class="m">_TPU95A</span> (Tyre_Ring) พิมพ์ TPU 95A: 4 walls, infill 60%% gyroid</li></ul>
<h2>ไฟล์ (%(n)d ไฟล์)</h2><table><tr><th>STL</th></tr>%(rows)s</table>
<h2>ใช้ทำอะไร</h2><ul>
<li><span class="m">Insert_Hole_D4.x / Insert_Horizontal_D4.x</span>: เลือกขนาดรู heat-set insert M3 (แนวตั้ง/แนวนอน)</li>
<li><span class="m">Insert_M2.5_Hole_D3.x</span>: เลือกขนาดรู heat-set insert M2.5 ของตัวถัง PC</li>
<li><span class="m">Pin_*_Clearance</span>: เลือกระยะเผื่อรูแกนบานพับ Ø3</li>
<li><span class="m">D_Shaft_Clearance</span>: เลือกระยะเผื่อรูเพลา D ของดุมล้อ (ต้องลองกับเพลามอเตอร์ขับจริง)</li>
<li><span class="m">Tyre_Ring_ID43.x</span>: เลือกรูในยางล้อ TPU ให้สวมดุมแน่น</li></ul>
<p><b>ส่งชิ้นทดลองทั้งหมดกลับให้ผู้สั่ง</b> ผู้สั่งจะกด insert ทดสอบ แล้วสร้างไฟล์ชิ้นจริงใหม่ด้วยขนาดที่เลือก ชิ้นจริงทุกชิ้นยังไม่อยู่ในใบนี้</p>''' % dict(n=len(files), rows=rows)
    pdf(body, folder / 'ORDER_COUPONS_TH.pdf')
    files.append({'file': 'ORDER_COUPONS_TH.pdf', 'source': 'generated', 'sha256': mm.sha(folder / 'ORDER_COUPONS_TH.pdf')})
    finish(folder, files, {'shop': '3D print (coupons only)', 'coupon_groups': dict(groups)})
    return files


def readme(released, held_metal, panels, coupon_files):
    held = DEC['held']
    lines = ['# ชุดไฟล์ส่งผลิต Robot V4 R3 — อนุมัติ %s' % DEC['date'], '',
             'ผู้ใช้เลือกปล่อยผลิตเฉพาะชิ้นที่ไม่ขึ้นกับมอเตอร์ที่ยังไม่ได้วัด ชิ้นที่เหลือเป็น HOLD ตามตารางด้านล่าง',
             'R3 build `%s` · ตัวถัง PC rev 3 · ไฟล์ทุกไฟล์คัดลอกจากต้นฉบับโดยไม่แก้ และมี SHA256SUMS.txt ในทุกโฟลเดอร์' % mm.REPORT['build_sha256'][:12], '',
             '## ส่งไฟล์ไหนให้ร้านไหน', '',
             '| ร้าน | ส่งไฟล์ | ข้างในมี |', '|---|---|---|',
             '| ร้านตัดเลเซอร์โลหะ | `01_LASER_METAL.zip` | %d แบบ %d ชิ้น: %s + ใบสั่งงานพร้อมแบบ `WORK_ORDER_LASER_TH.pdf` |' % (
                 len(released), sum(a['qty'] for a in released), ', '.join(a['id'] for a in released)),
             '| ร้าน CNC router (พลาสติก) | `02_CNC_PC.zip` | แผ่น PC %d แผ่น (PC01–PC07) + แบบ + `ORDER_CNC_PC_TH.pdf` |' % len(panels),
             '| ร้านกลึง | `03_LATHE.zip` | ดุมใบอาวุธ, spacer 3 ตัว, แกนบานพับ 2 ชิ้น + `ORDER_LATHE_TH.pdf` (ต้องส่งลูกปืน 608 จริง 2 ตัวไปด้วย) |',
             '| ร้านพิมพ์ 3D | `04_PRINT_COUPONS.zip` | ชิ้นทดลอง %d ไฟล์ + `ORDER_COUPONS_TH.pdf` (ยังไม่ใช่ชิ้นจริง) |' % (len(coupon_files) - 1), '',
             '## ความเสี่ยงที่ผู้ใช้ยอมรับ', ''] + ['- `%s`: %s' % (k, v) for k, v in DEC['accepted_risks'].items()] + [
             '', '## HOLD — ยังไม่ส่งผลิต', '', '| ชิ้น | ปลดล็อกเมื่อ |', '|---|---|'] + ['| %s | %s |' % (k, v) for k, v in held.items()] + [
             '', 'เมื่อวัดค่าแล้ว กรอกลงช่อง `actual` ใน `inputs_R3.json` สร้าง R3 และตัวถัง PC ใหม่ แล้วรันสคริปต์นี้อีกครั้งพร้อมแก้ `RELEASE_DECISION.json`', '',
             'สร้างใหม่: `.venv-v4-print/bin/python robot_v4/print_revision/r3/PRODUCTION/make_release_packages.py`']
    (OUT / 'README_TH.md').write_text('\n'.join(lines) + '\n')


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    for z in HERE.glob('RELEASE_%s/*.zip' % DEC['date']):
        z.unlink()
    OUT.mkdir(parents=True)
    released, held = laser()
    panels = cnc_pc()
    lathe()
    coupon_files = coupons()
    readme(released, held, panels, coupon_files)
    print(json.dumps({'out': str(OUT.relative_to(R3)), 'laser': [a['id'] for a in released], 'laser_held': [h['id'] for h in held],
                      'pc_panels': [p['id'] for p in panels], 'coupons': len(coupon_files) - 1,
                      'zips': sorted(str(z.relative_to(R3)) for z in OUT.glob('*.zip'))}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
