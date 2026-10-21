"""Dimensioned 2D drawings (PC panels + PETG part types) and the Thai production work order, as PDF.

Called at the end of build_pc_body.py with the in-memory CAD, so every number comes from the same geometry as the
DXF/STEP/STL files. HTML + SVG rendered to PDF with headless Chrome (same route as RFQ/make_metal_production_package.py).
"""
import hashlib
import re
import html
import math
import subprocess
from pathlib import Path

from cadquery.occ_impl.exporters.svg import getSVG
from OCP.BRepAdaptor import BRepAdaptor_Surface

CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
REV = 'PC-R3 rev 3'
DATE = '2026-10-18'
e = html.escape

CSS = '''@page{size:A4 landscape;margin:8mm}
body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:8.5pt;color:#111;margin:0}
.m{font-family:Menlo,monospace;font-size:7.5pt}.n{text-align:right;font-variant-numeric:tabular-nums}
table{border-collapse:collapse}td,th{border:1px solid #999;padding:1.5px 4px;text-align:left;vertical-align:top}th{background:#eee}
.stamp{border:2px solid #b00;color:#b00;font-weight:700;padding:2px 8px;display:inline-block;font-size:10.5pt}
.stamp.rel{border-color:#070;color:#070}.stamp.hold{border-color:#a60;color:#a60}
.sheet{page-break-before:always;height:192mm;display:grid;grid-template-columns:1.35fr 1fr;grid-template-rows:auto 1fr auto;gap:3px 8px}
.sheet:first-child{page-break-before:auto}
.hdr{grid-column:1/3;display:flex;gap:10px;align-items:center}
.draw{grid-row:2;border:1px solid #ccc;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:0;overflow:hidden}
.draw svg{max-width:100%;max-height:125mm}
.views{display:grid;grid-template-columns:1fr 1fr;gap:2px;width:100%;height:100%}.views div{border:1px solid #eee;display:flex;flex-direction:column;align-items:center;justify-content:center;overflow:hidden}
.views svg{max-width:100%;max-height:58mm}.cap{font-size:7.5pt;color:#333}
.side{grid-row:2;min-height:0;overflow:hidden}.side table{width:100%;font-size:7.5pt}
.small{font-size:7pt;color:#444;margin:2px 0}.tb{grid-column:1/3;width:100%}
.doc h1{font-size:15pt;margin:4px 0}.doc h2{font-size:11pt;margin:8px 0 3px}.doc table{width:100%;margin-bottom:4px}.doc li{margin-bottom:2px}
.ok{color:#070}.bad{color:#b00;font-weight:700}
'''

TOL = 'พิกัดทั่วไป ISO 2768-m · ขอบตัด ±0.2 · ตำแหน่งรู ±0.15 · รู +0.1/−0 · มุมด้านใน R1.5 · หน่วย มม. · ใช้ DXF/STEP เป็นต้นฉบับ ห้ามวัดจากภาพ'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pdf(html_text, path):
    path = Path(path)
    tmp = path.with_suffix('.html')
    tmp.write_text('<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (CSS, html_text))
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(path), tmp.as_uri()],
                   check=True, capture_output=True, timeout=180)
    tmp.unlink()


def title_block(part_id, name, material, thickness, qty, files, mass, status):
    return ('<table class="tb"><tr><th>ID</th><td>%s</td><th>ชิ้นงาน</th><td>%s</td><th>จำนวน/คัน</th><td>%s</td></tr>'
            '<tr><th>วัสดุ</th><td colspan="3">%s</td><th>หนา</th><td>%s</td></tr>'
            '<tr><th>ไฟล์</th><td class="m" colspan="3">%s</td><th>มวล</th><td>%s</td></tr>'
            '<tr><th>แบบ</th><td colspan="3">Robot V4 R3 · ตัวถัง PC · %s · %s · Z ขึ้น, หัวรถ +Y</td><th>สถานะ</th><td class="%s">%s</td></tr></table>') % (
        e(part_id), e(name), e(str(qty)), e(material), e(thickness), e(files), e(mass), REV, DATE, 'ok' if status.startswith('RELEASED') else 'bad', e(status))


def stamp_class(status):
    return 'stamp rel' if status.startswith('RELEASED') else 'stamp hold' if status.startswith('HOLD') else 'stamp'


def panel_svg(p, filleted):
    us = [q[0] for q in p.outline]; vs = [q[1] for q in p.outline]
    u0, v0, w, h = min(us), min(vs), max(us) - min(us), max(vs) - min(vs)
    pad, dim = 10.0, 9.0
    W, H = w + 2 * pad + dim, h + 2 * pad + dim
    tx = lambda u: pad + dim + (u - u0)
    ty = lambda v: pad + (h - (v - v0))
    sw = max(0.25, w / 450)
    s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.2f %.2f">' % (W, H)]

    def loop(pts, cutout, color, dash=''):
        start, segs = filleted(pts, 1.5, cutout)
        d = 'M%.3f %.3f' % (tx(start[0]), ty(start[1]))
        for sg in segs:
            if sg[0] == 'L': d += ' L%.3f %.3f' % (tx(sg[1][0]), ty(sg[1][1]))
            else: d += ' L%.3f %.3f L%.3f %.3f' % (tx(sg[1][0]), ty(sg[1][1]), tx(sg[2][0]), ty(sg[2][1]))
        s.append('<path d="%s Z" fill="none" stroke="%s" stroke-width="%.2f" %s/>' % (d, color, sw, dash))
    loop(p.outline, False, '#000')
    for c in p.cutouts: loop(c, True, '#000')
    for c, depth, face in p.pockets: loop(c, True, '#06c', 'stroke-dasharray="1.5,1"')
    fs = max(1.8, min(w, h) / 34)
    for i, (u, v, r, csk) in enumerate(p.holes, 1):
        s.append('<circle cx="%.3f" cy="%.3f" r="%.3f" fill="none" stroke="#000" stroke-width="%.2f"/>' % (tx(u), ty(v), r, sw))
        if csk: s.append('<circle cx="%.3f" cy="%.3f" r="%.3f" fill="none" stroke="#090" stroke-width="%.2f"/>' % (tx(u), ty(v), csk / 2, sw))
        s.append('<text x="%.2f" y="%.2f" font-size="%.2f" fill="#c00" font-family="Menlo,monospace">H%d</text>' % (tx(u) + r + 0.5, ty(v) - r - 0.3, fs * 0.8, i))
    for j, c in enumerate(p.cutouts, 1):
        cu = sum(q[0] for q in c) / len(c); cv = sum(q[1] for q in c) / len(c)
        s.append('<text x="%.2f" y="%.2f" font-size="%.2f" fill="#c00" text-anchor="middle" font-family="Menlo,monospace">C%d</text>' % (tx(cu), ty(cv), fs, j))
    for j, (c, depth, face) in enumerate(p.pockets, 1):
        cu = sum(q[0] for q in c) / len(c); cv = sum(q[1] for q in c) / len(c)
        s.append('<text x="%.2f" y="%.2f" font-size="%.2f" fill="#06c" text-anchor="middle" font-family="Menlo,monospace">P%d</text>' % (tx(cu), ty(cv), fs, j))
    for k, (u, v) in enumerate(p.outline, 1):
        s.append('<text x="%.2f" y="%.2f" font-size="%.2f" fill="#777" font-family="Menlo,monospace">V%d</text>' % (tx(u) + 0.6, ty(v) + fs * 0.9, fs * 0.65, k))
    yd, xd = pad + h + dim * 0.7, pad + dim * 0.3
    s.append('<g stroke="#555" stroke-width="0.25" fill="#555" font-size="%.2f" font-family="Menlo,monospace">' % fs)
    s.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/><text x="%.2f" y="%.2f" text-anchor="middle" stroke="none">%.1f</text>' % (
        tx(u0), yd, tx(u0 + w), yd, tx(u0 + w / 2), yd + fs, w))
    s.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/><text x="%.2f" y="%.2f" text-anchor="middle" stroke="none" transform="rotate(-90 %.2f %.2f)">%.1f</text>' % (
        xd, ty(v0), xd, ty(v0 + h), xd - 1, ty(v0 + h / 2), xd - 1, ty(v0 + h / 2), h))
    s.append('<circle cx="%.2f" cy="%.2f" r="0.9" fill="#000"/><text x="%.2f" y="%.2f" stroke="none" fill="#000">0,0</text></g></svg>' % (
        tx(u0), ty(v0), tx(u0) + 1.2, ty(v0) - 1.0))
    return '\n'.join(s), (u0, v0, w, h)


def panel_sheet(p, filleted, mass, files, status, part_id):
    svg, (u0, v0, w, h) = panel_svg(p, filleted)
    rows = []
    for i, (u, v, r, csk) in enumerate(p.holes, 1):
        op = 'ทะลุ' + (' + countersink 90° Ø%.1f ด้าน FACE UP' % csk if csk else '')
        rows.append('<tr><td>H%d</td><td class="n">%.2f</td><td class="n">%.2f</td><td class="n">Ø%.1f</td><td>%s</td></tr>' % (i, u - u0, v - v0, 2 * r, op))
    for j, c in enumerate(p.cutouts, 1):
        cu = [q[0] - u0 for q in c]; cv = [q[1] - v0 for q in c]
        rows.append('<tr><td>C%d</td><td class="n">%.2f–%.2f</td><td class="n">%.2f–%.2f</td><td>ช่อง</td><td>ตัดทะลุ มุม R1.5</td></tr>' % (j, min(cu), max(cu), min(cv), max(cv)))
    for j, (c, depth, face) in enumerate(p.pockets, 1):
        cu = [q[0] - u0 for q in c]; cv = [q[1] - v0 for q in c]
        rows.append('<tr><td>P%d</td><td class="n">%.2f–%.2f</td><td class="n">%.2f–%.2f</td><td>ร่อง</td><td>ลึก %.1f ±0.1 จาก%s มุม R1.5</td></tr>' % (
            j, min(cu), max(cu), min(cv), max(cv), depth, 'ด้านล่าง (ตรงข้าม FACE UP)' if face == 'down' else ' FACE UP'))
    verts = ' · '.join('V%d (%.1f, %.1f)' % (k, u - u0, v - v0) for k, (u, v) in enumerate(p.outline, 1))
    head = '<tr><th>ID</th><th>X</th><th>Y</th><th>ขนาด</th><th>งาน</th></tr>'
    if len(rows) > 26:
        half = (len(rows) + 1) // 2
        table = '<div style="display:flex;gap:4px"><table>%s%s</table><table>%s%s</table></div>' % (head, ''.join(rows[:half]), head, ''.join(rows[half:]))
    else:
        table = '<table>%s%s</table>' % (head, ''.join(rows))
    side = ('%s'
            '<p class="small"><b>มุมขอบนอก (ก่อนลบมุม R1.5):</b> %s</p>'
            '<p class="small">พิกัดวัดจากจุด 0,0 = มุมล่างซ้ายของกรอบชิ้นงาน มองจาก FACE UP: %s</p><p class="small">%s</p>'
            '<p class="small">ห้ามตัดเลเซอร์ · CNC router ดอก O-flute Ø3 · ลบคมทุกขอบ · ห้ามตัวทำละลาย/น้ำยาล็อกเกลียวกับ PC · ห้าม mirror</p>') % (
        table, verts, e(p.face_up), TOL)
    return ('<section class="sheet"><div class="hdr"><span class="%s">%s</span><b>%s</b></div><div class="draw">%s</div><div class="side">%s</div>%s</section>' % (
        stamp_class(status), e(status), e(p.name), svg, side, title_block(part_id, p.name, 'แผ่นโพลีคาร์บอเนตตัน (solid PC) Makrolon GP / Lexan 9030 หรือเทียบเท่า', '%.1f มม.' % p.t, 1, files, '%.1f g' % mass, status)))


def cyl_holes(shape):
    found = {}
    for f in shape.Faces():
        if f.geomType() != 'CYLINDER': continue
        c = BRepAdaptor_Surface(f.wrapped).Cylinder()
        if f.Area() > 2 * math.pi * c.Radius() * 60: continue
        key = round(2 * c.Radius(), 2)
        loc = c.Location(); ax = c.Axis().Direction()
        tag = (round(loc.X() * abs(ax.Y() + ax.Z()) + loc.Y() * abs(ax.X() + ax.Z()), 1), round(loc.Z() * abs(ax.X() + ax.Y()), 1), key)
        found.setdefault(key, set()).add(tag)
    return {k: len(v) for k, v in sorted(found.items())}


def part_sheet(group, status, part_id, notes):
    shape = group['shape']
    bb = shape.BoundingBox()
    views = []
    for title, d in (('มองบน (−Z)', (0, 0, 1)), ('มองหน้า (−Y)', (0, -1, 0)), ('มองข้าง (+X)', (1, 0, 0)), ('Isometric', (1, -1, 1))):
        svg = getSVG(shape, {'width': 300, 'height': 220, 'marginLeft': 10, 'marginTop': 10, 'showAxes': False, 'projectionDir': d,
                             'strokeWidth': 0.4, 'showHidden': title != 'Isometric', 'hiddenColor': (160, 160, 160), 'strokeColor': (0, 0, 0)})
        svg = svg[svg.index('<svg'):]
        views.append('<div>%s<span class="cap">%s</span></div>' % (svg, title))
    holes = cyl_holes(shape)
    hrows = ''.join('<tr><td class="n">Ø%.2f</td><td class="n">%d</td></tr>' % (d, n) for d, n in holes.items())
    side = ('<table><tr><th>ขนาดรวม X × Y × Z (ท่าพิมพ์)</th><td class="n">%.1f × %.1f × %.1f มม.</td></tr>'
            '<tr><th>ไฟล์ STL</th><td class="m">%s</td></tr><tr><th>Insert ที่ต้องกด</th><td>%s</td></tr></table>'
            '<table><tr><th>รูทรงกระบอก</th><th>จำนวน</th></tr>%s</table>'
            '<p class="small">%s</p><p class="small">PETG · 4 walls · %s · ท่าพิมพ์ตามไฟล์ (Z ขึ้น ฐานพิมพ์ Z=0) · รู insert เป็นค่าทดลอง ต้องยืนยันด้วย coupon · '
            'ตรวจขนาดรวม ±0.3 และรู ±0.1 หลังพิมพ์ · ห้าม mirror: ซ้าย/ขวาใช้ไฟล์แยก</p>') % (
        bb.xlen, bb.ylen, bb.zlen, e('<br>'.join(group['files'])).replace('&lt;br&gt;', '<br>'), e(group['inserts']), hrows, e(group['note']), e(group['infill']))
    return ('<section class="sheet"><div class="hdr"><span class="%s">%s</span><b>%s</b></div><div class="draw"><div class="views">%s</div></div><div class="side">%s</div>%s</section>' % (
        stamp_class(status), e(status), e(group['title']), ''.join(views), side,
        title_block(part_id, group['title'], 'PETG (พิมพ์ 3D)', '-', group['qty'], ', '.join(group['files']), '%.2f g/ชิ้น (80%%)' % group['mass'], status)))


def work_order(ctx):
    r = ctx['report']; b = r['balance']; imp = r['stiffness']['impact']
    blk = ''.join('<li class="bad">%s</li>' % e(x) for x in ctx['blockers'])
    pan = ''.join('<tr><td>%s</td><td class="m">%s</td><td class="n">%.1f</td><td class="n">%s</td><td class="n">%d</td><td class="n">%d</td><td class="n">%.1f</td></tr>' % (
        p['id'], e(p['file']), p['thickness_mm'], ' × '.join('%g' % v for v in p['flat_size_mm']), p['holes'], p['countersinks'], p['mass_g']) for p in ctx['panels'])
    prt = ''.join('<tr><td>%s</td><td>%s</td><td class="n">%d</td><td class="m">%s</td><td>%s</td><td class="n">%.2f</td></tr>' % (
        g['id'], e(g['title']), g['qty'], e(', '.join(g['files'])), e(g['infill']), g['mass']) for g in ctx['groups'])
    fas = ''.join('<tr><td>%s</td><td class="n">%d</td><td class="n">%.2f</td><td>%s</td></tr>' % (e(k), n, ctx['fastener_g'][k], e(ctx['fastener_use'].get(k, ''))) for k, n in ctx['fastener_counts'].items())
    sfr = ''.join('<tr><td>%s</td><td>%s</td><td class="n">%s</td><td class="n">%s</td><td class="n %s">%s</td><td>%s</td></tr>' % (
        e(x['case']), e(x['path']), x['load'], x['capacity'], 'ok' if (x['sf'] or 99) >= 1.5 else 'bad', '–' if x['sf'] is None else '%.2f' % x['sf'], e(x['note'])) for x in imp['rows'])
    th = {'small parts solid (cleats, skirt brackets, twin saddles, lid bars), cradles/side blocks 80 % (expected case)': 'คาดว่าจะเกิดจริง: ชิ้นเล็กเนื้อเต็ม ชิ้นใหญ่ 80%',
          'new PETG printed 100 % instead of the R3 80 % rule': 'PETG ใหม่ทุกชิ้นเนื้อเต็ม', 'PC sheet 3.3 mm (+10 % thickness tolerance)': 'แผ่น PC หนากว่าแบบ 10%',
          'PC density 1.22 g/cm^3': 'PC density 1.22', 'fasteners +25 %': 'สกรู/insert หนักกว่าที่ประมาณ 25%', 'all four together': 'สี่กรณีบนพร้อมกัน'}
    sens = ''.join('<tr><td>%s</td><td class="n">%.2f</td><td class="n">%.3f</td><td class="%s">%s</td></tr>' % (
        e(th.get(k, k)), v['total_g'], v['rear_load_pct'], 'ok' if v['passes_hard_2000g_10pct'] else 'bad', 'ผ่าน' if v['passes_hard_2000g_10pct'] else 'ไม่ผ่าน') for k, v in b['sensitivity'].items())
    ex = b['expected_small_parts_solid']; nom = b['pc_body']
    return '''<section class="doc"><span class="stamp">%(status)s</span>
<h1>ใบสั่งผลิต — ตัวถังโพลีคาร์บอเนต (PC) Robot V4 R3 · %(rev)s</h1>
<p>%(date)s · แนวแกน Z ขึ้น พื้น XY ที่ Z=0 หัวรถชี้ +Y · หน่วย มม. · ไฟล์ทั้งหมดอยู่ใน <span class="m">robot_v4/print_revision/r3/pc_body/output/</span> · แบบ 2D อยู่ใน <span class="m">PRODUCTION/DRAWINGS_PC_BODY.pdf</span></p>
<h2>1. เงื่อนไขก่อนผลิต (ต้องปิดทุกข้อ ห้ามตัด/พิมพ์ชุดจริงก่อน)</h2><ol>%(blk)s</ol>
<p class="small">ขั้นที่อนุญาตตอนนี้: ขอราคา · ตัดแผ่น PC ทดลอง 1 ชุดเพื่อวัดและชั่ง · พิมพ์ coupon รู insert และชิ้นทดลองดึง insert</p>
<h2>2. น้ำหนักและสมดุล (จาก CAD)</h2>
<table><tr><th></th><th>มวลรวม g</th><th>เผื่อถึง 2000 g</th><th>โหลดล้อหลัง %%</th><th>เกณฑ์</th></tr>
<tr><td>กรณีคาดว่าจะเกิดจริง (ชิ้น PETG เล็กพิมพ์เนื้อเต็ม)</td><td class="n">%(ext).2f</td><td class="n">%(exm).2f</td><td class="n">%(exr).3f</td><td>≤2000 g, ≥10%%; เผื่อ ≥20 g</td></tr>
<tr><td>กฎ 80%% ของ R3</td><td class="n">%(nt).2f</td><td class="n">%(nm).2f</td><td class="n">%(nr).3f</td><td></td></tr></table>
<table><tr><th>ความไว</th><th>มวลรวม g</th><th>ล้อหลัง %%</th><th>2000 g / 10%%</th></tr>%(sens)s</table>
<h2>3. ความแข็งแรง: เป้าหมายตก 0.5 ม. (%(n).0f g) และแรงกระแทกอาวุธ (CALC_R3)</h2>
<p class="small">SF ต่ำสุดของตัวถัง %(minsf).2f (%(minp)s) · ต้องการ ≥1.5 · เพลามอเตอร์ขับ 4 มม. ตั้งใจให้เป็นจุดอ่อนที่สุด (ครากที่ %(fy).0f–%(fy2).0f N ต่อล้อ) · ค่าความแข็งแรงที่ ASSUMED ต้องทดสอบ (ข้อ 1)</p>
<table><tr><th>กรณี</th><th>เส้นทางแรง</th><th>แรง</th><th>รับได้</th><th>SF</th><th>หมายเหตุ</th></tr>%(sfr)s</table>
<h2>4. แผ่น PC (ร้าน CNC router — ห้ามเลเซอร์)</h2>
<table><tr><th>ID</th><th>ไฟล์ DXF/STEP</th><th>หนา</th><th>ขนาด</th><th>รู</th><th>CSK</th><th>g</th></tr>%(pan)s</table>
<p class="small">วัสดุ solid PC เกรดทั่วไป ไม่ใช่อะคริลิค/แผ่นลอน · วัดความหนาแผ่นจริง 4 จุดก่อนตัด · ชั่งแผ่นหลังตัด · ไฟล์ละ 1 ชิ้น ห้าม mirror · %(tol)s</p>
<h2>5. ชิ้น PETG (ร้านพิมพ์ 3D)</h2>
<table><tr><th>ID</th><th>ชิ้น</th><th>จำนวน</th><th>ไฟล์ STL</th><th>ข้อกำหนด</th><th>g/ชิ้น</th></tr>%(prt)s</table>
<h2>6. สกรู insert และวัสดุประกอบ (เพิ่มจาก R3)</h2>
<table><tr><th>รายการ</th><th>จำนวน</th><th>g/ชิ้น</th><th>ใช้ที่</th></tr>%(fas)s</table>
<p class="small">ขันสกรูที่ยึดผ่าน PC ≤0.3 N·m · ห้ามน้ำยาล็อกเกลียวสัมผัส PC · มวลสกรูเป็นค่าตาราง/ประมาณ ต้องชั่งจริง (ข้อ 1)</p>
<h2>7. ลำดับประกอบและการตรวจรับ</h2><ol>
<li>กด insert ทุกตัว ตรวจว่าไม่หมุนและไม่ยื่นเกินผิว</li>
<li>ยึดแท่นมอเตอร์ (cradle) twin saddle บล็อกอาวุธ ราง wedgelet และตัวยึดมุมล่างลงพื้นจากใต้พื้น</li>
<li>ยึดขายึด skirt เข้าผนังข้าง/หลัง ก่อนใส่ล้อ (สกรูหัวจม 4 ตัวอยู่หลังยาง)</li>
<li>ประกอบผนัง ผนังหลัง bulkhead และตัวยึดมุมบน</li>
<li>ติดแผ่น EPDM 0.5 มม. ใน cup ของ twin saddle และ lid bar; ใส่มอเตอร์ขับ ตรวจว่ามอเตอร์แนบ pad ทั้งสองปลาย</li>
<li>ใส่ล้อ ชุดอาวุธ (M3×10 เข้าแผ่นตั้ง) อุปกรณ์ไฟฟ้า skirt และฝา (lid bar ยึดใต้ฝาด้วย M2.5 หัวจม 2 ตัว)</li>
<li>ชั่งทั้งคันและวัดโหลดล้อหลังจริง: ต้อง ≤2000 g และ ≥10%%; บันทึกผล</li></ol>
</section>''' % dict(status=e(ctx['status']), rev=REV, date=DATE, blk=blk, ext=ex['total_g'], exm=ex['margin_to_2000_g'], exr=ex['rear_load_pct'],
                    nt=nom['total_g'], nm=nom['margin_to_2000_g'], nr=nom['rear_load_pct'], sens=sens, n=imp['N_drop_g'], minsf=imp['min_design_sf'],
                    minp=e(imp['min_design_path']), fy=imp['shaft']['F_yield_N'][0], fy2=imp['shaft']['F_yield_N'][1], sfr=sfr, pan=pan, tol=TOL, prt=prt, fas=fas)


def make(out, panels, panel_rows, groups, ctx, filleted):
    prod = Path(out) / 'PRODUCTION'
    (prod / 'drawings').mkdir(parents=True, exist_ok=True)
    for old in list(prod.glob('*.pdf')) + list((prod / 'drawings').glob('*.pdf')):
        old.unlink()
    status = ctx['status']
    status_of = ctx.get('status_of') or (lambda kind, name: status)
    sheets = []
    for k, (key, p) in enumerate(panels.items(), 1):
        row = next(r for r in panel_rows if r['file'] == p.name + '.dxf')
        row['id'] = 'PC%02d' % k
        sheets.append((row['id'], p.name, panel_sheet(p, filleted, row['mass_g'], p.name + '.dxf / .step', status_of('panel', p.name), row['id'])))
    for g in groups:
        sheets.append((g['id'], g['title'], part_sheet(g, status_of('petg', g['title']), g['id'], g.get('notes', {}))))
    index = []
    for pid, name, html_sheet in sheets:
        f = prod / 'drawings' / ('%s_%s.pdf' % (pid, re.sub(r'[^A-Za-z0-9+-]+', '_', name).strip('_')))
        pdf(html_sheet, f)
        index.append({'id': pid, 'name': name, 'file': str(f.relative_to(out)), 'sha256': sha(f)})
    pdf(''.join(h for _, _, h in sheets), prod / 'DRAWINGS_PC_BODY.pdf')
    ctx['panels'] = panel_rows; ctx['groups'] = groups
    pdf(work_order(ctx), prod / 'WORK_ORDER_PC_BODY_TH.pdf')
    return {'drawings': index, 'combined': 'PRODUCTION/DRAWINGS_PC_BODY.pdf', 'work_order': 'PRODUCTION/WORK_ORDER_PC_BODY_TH.pdf',
            'sha256': {'DRAWINGS_PC_BODY.pdf': sha(prod / 'DRAWINGS_PC_BODY.pdf'), 'WORK_ORDER_PC_BODY_TH.pdf': sha(prod / 'WORK_ORDER_PC_BODY_TH.pdf')}}
