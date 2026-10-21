"""Welding-shop package for the R4-steel frame: which piece is which, where it goes, what is welded to what.

Runs build_r4_steel.py (imports it, so the CAD and checks are rebuilt first), then writes RFQ_WELD/ + RFQ_WELD.zip:
  WELD_DRAWING_TH.pdf  Thai drawing: labelled 3D view, top/side/front views with the datum, tube cut list with
                       positions, plate list, weld list (from the CAD contacts), assembly order and jigs
  DXF/                 every steel plate flat (tabs; L-tabs split into two plates welded at 90 deg)
  FRAME_WELDMENT.step  the frame only, same coordinates as the robot (Z-up, nose +Y)
  images/              the views used in the PDF
Status: FOR QUOTE / DESIGN REVIEW. The drawing says what is still missing before anyone welds.
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r4_steel/make_weld_package.py
"""
import html
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_r4_steel as B                              # noqa: E402  (builds the CAD)
import cadquery as cq                                   # noqa: E402
import matplotlib                                       # noqa: E402
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['Sukhumvit Set', 'Tahoma']   # Thai labels in the views
import matplotlib.pyplot as plt                         # noqa: E402
from matplotlib.patches import Rectangle                # noqa: E402

PKG = HERE / 'RFQ_WELD'
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
DATUM = (-(B.WALL_X - B.SKIN), B.REAR_Y + B.SKIN, B.RAIL_Z0)    # rear-left-bottom corner of the rear cross
e = html.escape
if B.report['interference'] or B.report['frame_members_not_touching'] or B.report['edge_only_contacts'] or B.report['invalid_bodies']:
    raise RuntimeError('CAD checks failed; fix build_r4_steel.py first')

TH = {'Frame_Rear_Cross': 'คานท้ายล่าง', 'Frame_Bulkhead_Cross': 'คานขวางหน้าล่าง', 'Frame_Rail': 'รางข้าง',
      'Frame_Fork_Arm': 'แขนหน้า', 'Frame_Hinge_Beam': 'คานบานพับ', 'Frame_Post_Rear': 'เสาท้าย', 'Frame_Post_Front': 'เสาหน้า',
      'Frame_Top_Rear_Cross': 'คานบนท้าย', 'Frame_Top_Front_Stub': 'ท่อนบนหน้า', 'Tab_Drive_Motor': 'แผ่นยึดมอเตอร์ขับ',
      'Tab_Upright': 'แผ่นยึดแผ่นตั้งอาวุธ', 'Tab_Hinge': 'หูบานพับ wedgelet', 'Tab_Battery_Strap': 'หูสายรัดแบต',
      'Tab_Weapon_Mount': 'หูยึดแผ่นมอเตอร์อาวุธ', 'Strap_Drive_Motor': 'แผ่นรองมอเตอร์ขับ'}


def thai(name):
    key = max((k for k in TH if name.startswith(k)), key=len)
    side = ' ซ้าย' if '_L' in name[len(key):] else ' ขวา' if '_R' in name[len(key):] else ''
    return TH[key] + side


tubes = [b for b in B.steel if b.material == 'steel_tube']
tabs = [b for b in B.steel if b.material == 'steel_tab']
ID = {b.name: 'F%02d' % (i + 1) for i, b in enumerate(tubes)}
ID.update({b.name: 'T%02d' % (i + 1) for i, b in enumerate(tabs)})


def rel(bb):
    return tuple(round(v, 1) + 0.0 for v in (bb.xmin - DATUM[0], bb.ymin - DATUM[1], bb.zmin - DATUM[2]))


# ---------------------------------------------------------------- plates: flat DXF per plate
def plates_of(b):
    """(suffix, shape) planar plates. L tabs are split into two plates that the shop welds at 90 deg."""
    s = b.shape
    if b.name.startswith('Tab_Upright'):
        x0 = 33.0 if b.shape.BoundingBox().xmin > 0 else -34.5
        web = s.intersect(B.block(x0, x0 + B.TAB, -200, 200, B.RAIL_Z0 + B.TAB, 100))
        return [('a', web), ('b', s.cut(web))]
    if b.name.startswith('Tab_Weapon_Mount'):
        base = s.intersect(B.block(-200, 200, -200, 200, B.RAIL_Z0, B.RAIL_Z0 + B.TAB))
        return [('a', base), ('b', s.cut(base))]
    return [('', s)]


def flat(shape):
    """Rotate a plate so its thin axis is Z, move it to z 0..t, return (workplane section at mid-thickness, size, holes)."""
    bb = shape.BoundingBox()
    dims = {'x': bb.xlen, 'y': bb.ylen, 'z': bb.zlen}
    thin = min(dims, key=dims.get)
    if abs(dims[thin] - B.TAB) > 0.05:          # OCC boxes are loose by ~0.02
        raise RuntimeError('not a %.1f mm plate: %s' % (B.TAB, dims))
    s = shape.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.center.z))
    if thin == 'x':
        s = s.rotate(cq.Vector(), cq.Vector(0, 1, 0), 90)
    elif thin == 'y':
        s = s.rotate(cq.Vector(), cq.Vector(1, 0, 0), 90)
    sec = cq.Workplane('XY').add(s).section(0)
    fb = s.BoundingBox()
    holes = sorted({round(2 * c.radius(), 2) for c in sec.edges('%CIRCLE').vals()})
    n_holes = len(sec.edges('%CIRCLE').vals())
    return sec, (round(fb.xlen, 0), round(fb.ylen, 0)), holes, n_holes


if PKG.exists():
    shutil.rmtree(PKG)
(PKG / 'DXF').mkdir(parents=True)
(PKG / 'images').mkdir()
plate_rows = []
for b in tabs:
    for suf, shape in plates_of(b):
        sec, size, holes, n = flat(shape)
        pid = ID[b.name] + suf
        fname = '%s_%s.dxf' % (pid, b.name)
        cq.exporters.exportDXF(sec, str(PKG / 'DXF' / fname))
        plate_rows.append({'id': pid, 'part': b.name, 'size': size, 'holes': holes, 'n_holes': n, 'dxf': fname,
                           'pos': [round(v, 1) for v in rel(shape.BoundingBox())]})

# ---------------------------------------------------------------- images
COL = {'steel_tube': '#c0392b', 'steel_tab': '#e67e22'}


def iso(path):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(11, 8), facecolor='white')
    ax = fig.add_subplot(projection='3d')
    for b in B.steel:
        v, t = b.shape.tessellate(0.3, 0.3)
        import numpy as np
        v = np.array([p.toTuple() for p in v])
        ax.add_collection3d(Poly3DCollection(v[np.array(t)], facecolors=COL[b.material], edgecolors='#333', linewidths=0.15, alpha=0.95))
        c = b.shape.Center()
        ax.text(c.x, c.y, c.z + 6, ID[b.name], fontsize=7 if b.material == 'steel_tab' else 9,
                color='#000' if b.material == 'steel_tube' else '#7a3a00', fontweight='bold', ha='center')
    ax.scatter([DATUM[0]], [DATUM[1]], [DATUM[2]], s=60, c='blue')
    ax.text(DATUM[0], DATUM[1] - 12, DATUM[2], 'DATUM 0,0,0', color='blue', fontsize=9, fontweight='bold')
    ax.set_xlim(-95, 95); ax.set_ylim(-115, 110); ax.set_zlim(0, 70); ax.set_box_aspect((190, 225, 70)); ax.view_init(elev=28, azim=-60)
    ax.set_xlabel('X'); ax.set_ylabel('Y (หัวรถ +Y)'); ax.set_zlabel('Z')
    fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)


def ortho(path, a, b_, title, xlabel, ylabel):
    """Projected bounding boxes in datum coordinates. a, b_ in 'xyz'."""
    fig, ax = plt.subplots(figsize=(10, 6.2), facecolor='white')
    k = 'xyz'
    for m in sorted(B.steel, key=lambda m: m.material != 'steel_tube'):
        bb = m.shape.BoundingBox()
        lo, hi = rel(bb), (bb.xmax - DATUM[0], bb.ymax - DATUM[1], bb.zmax - DATUM[2])
        i, j = k.index(a), k.index(b_)
        ax.add_patch(Rectangle((lo[i], lo[j]), hi[i] - lo[i], hi[j] - lo[j], facecolor=COL[m.material], edgecolor='#222', lw=0.6, alpha=0.55))
        upper = (a, b_) == ('x', 'y') and lo[2] > 30            # top members sit over the bottom ones in plan view
        ax.text((lo[i] + hi[i]) / 2, (lo[j] + hi[j]) / 2 + (7 if upper else 0), ID[m.name] + (' (บน)' if upper else ''),
                fontsize=6.5 if m.material == 'steel_tab' else 8, ha='center', va='center', fontweight='bold')
    ax.plot(0, 0, 'bo'); ax.annotate('DATUM', (0, 0), (4, -9), color='blue', fontsize=8)
    ax.set_aspect('equal'); ax.grid(True, lw=0.3, alpha=0.6); ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.set_title(title)
    ax.autoscale_view(); fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)


iso(PKG / 'images' / 'iso_ids.png')
ortho(PKG / 'images' / 'top.png', 'x', 'y', 'มองจากด้านบน (X ไปขวา, หัวรถ +Y ขึ้นบน) หน่วย มม. จาก DATUM', 'X (มม.)', 'Y (มม.)')
ortho(PKG / 'images' / 'side.png', 'y', 'z', 'มองจากด้านขวา (+X) หัวรถไปทางขวา', 'Y (มม.)', 'Z (มม.)')
ortho(PKG / 'images' / 'rear.png', 'x', 'z', 'มองจากท้ายรถไปหัวรถ', 'X (มม.)', 'Z (มม.)')
shutil.copy2(B.OUT / 'FRAME_WELDMENT.step', PKG / 'FRAME_WELDMENT.step')

# ---------------------------------------------------------------- PDF
def axis_of(bb):
    return max((('X', bb.xlen), ('Y', bb.ylen), ('Z', bb.zlen)), key=lambda t: t[1])


tube_rows = ''
for m in tubes:
    bb = m.shape.BoundingBox(); ax_, L = axis_of(bb); p = rel(bb)
    tube_rows += '<tr><td><b>%s</b></td><td>%s</td><td class="n">%.1f</td><td>%s</td><td class="n">%.1f, %.1f, %.1f</td></tr>' % (
        ID[m.name], e(thai(m.name)), L, ax_, *p)
plate_html = ''.join('<tr><td><b>%s</b></td><td>%s</td><td class="n">%.1f × %.1f</td><td>%s</td><td class="n">%.1f, %.1f, %.1f</td><td class="m">%s</td></tr>' % (
    r['id'], e(thai(r['part'])), *r['size'], ', '.join('Ø%g' % h for h in r['holes']) + (' (%d รู/ร่อง)' % r['n_holes'] if r['n_holes'] else '') or '–',
    *r['pos'], e(r['dxf'])) for r in plate_rows)
weld_html = ''
for i, w in enumerate(B.welds):
    tube_tube = w['a'] in ID and w['b'] in ID and ID[w['a']][0] == ID[w['b']][0] == 'F'
    how = 'เชื่อมรอบรอยต่อ fillet 1 มม.' if tube_tube else 'fillet 1.5 มม. ตลอดแนวสัมผัส ทั้งสองด้านถ้าเข้าถึงได้'
    c = [w['contact_centre_mm'][k] - DATUM[k] for k in range(3)]
    weld_html += '<tr><td>W%02d</td><td><b>%s</b> + <b>%s</b></td><td class="n">%.0f, %.0f, %.0f</td><td class="n">%g × %g</td><td>%s</td></tr>' % (
        i + 1, ID[w['a']], ID[w['b']], *c, *w['contact_size_mm'], how)
mass = B.report['mass']
steel_g = sum(r['mass_g'] for r in B.rows if str(r.get('material', '')).startswith('steel'))
body = '''<span class="stamp">แบบสำหรับขอราคา / ตรวจแบบ — ยังไม่อนุมัติให้เชื่อม</span>
<h1>โครงเหล็กเชื่อม Robot V4 R4-steel</h1>
<p>หน่วย มม. · แกน Z ขึ้น หัวรถ +Y · <b>ซ้าย = −X</b> (ยืนท้ายรถมองไปหัวรถ) · ไฟล์ <span class="m">FRAME_WELDMENT.step</span> เป็นต้นฉบับ ห้ามวัดจากภาพ
· <b>DATUM (0,0,0)</b> = มุมซ้าย-ท้าย-ล่าง ของคานท้ายล่าง F01 · ตำแหน่งทุกชิ้นในตารางคือมุมต่ำสุดของชิ้น (X, Y, Z) วัดจาก DATUM</p>
<h2>1. ภาพรวมและรหัสชิ้น (F = ท่อ, T = แผ่นยึด)</h2><img src="images/iso_ids.png" style="width:100%%">
<h2>2. มุมมอง 3 ด้าน</h2><img src="images/top.png" style="width:100%%"><img src="images/side.png" style="width:49%%"><img src="images/rear.png" style="width:49%%">
<h2>3. วัสดุ</h2><ul>
<li>ท่อ: <b>เหล็กกล่องเหลี่ยม 10×10 หนา %(wall).1f มม.</b> (เหล็กเหนียวทั่วไป) ตัดปลายฉาก 90° ทุกท่อน ไม่ต้องตัดเฉียง</li>
<li>แผ่นยึด: <b>เหล็กแผ่นหนา %(tab).1f มม.</b> ตัดเลเซอร์ตามไฟล์ใน <span class="m">DXF/</span> ไฟล์ละ 1 ชิ้น
(ชิ้นที่ลงท้าย a/b คือแผ่น 2 แผ่นที่เชื่อมกันเป็นฉาก 90°)</li>
<li><b class="bad">ห้ามเชื่อมชิ้นอะลูมิเนียม จานอาวุธ หรือ wedgelet เข้ากับโครง</b> ชิ้นพวกนั้นยึดด้วยน็อตภายหลัง</li></ul>
<h2>4. รายการตัดท่อ (%(nt)d ท่อน รวม %(tl).0f มม.)</h2>
<table><tr><th>รหัส</th><th>ชื่อ</th><th>ยาว</th><th>แนว</th><th>ตำแหน่ง X, Y, Z</th></tr>%(tubes)s</table>
<h2>5. แผ่นยึด (%(np)d แผ่น)</h2>
<table><tr><th>รหัส</th><th>ชื่อ</th><th>ขนาดแผ่น</th><th>รู</th><th>ตำแหน่ง X, Y, Z</th><th>DXF</th></tr>%(plates)s</table>
<h2>6. รายการจุดเชื่อม (%(nw)d จุด คำนวณจากผิวที่ชิ้นสัมผัสกันใน CAD)</h2>
<table><tr><th>จุด</th><th>ชิ้นที่เชื่อม</th><th>ตำแหน่ง X, Y, Z</th><th>ขนาดผิวสัมผัส</th><th>วิธีเชื่อม</th></tr>%(welds)s</table>
<h2>7. ลำดับการประกอบ</h2><ol>
<li>วางบนโต๊ะเรียบ ประกอบ<b>กรอบล่าง</b>: F01 คานท้ายล่าง, F02 คานขวางหน้าล่าง และรางข้างที่อยู่ระหว่างสองคาน ยึด clamp แล้ว<b>วัดเส้นทแยงมุมสองเส้นให้ต่างกันไม่เกิน 1 มม.</b> ก่อนแต้มเชื่อม</li>
<li>แขนหน้าต่อจากหน้าคาน F02 แล้วคานบานพับต่อที่ปลายแขน</li>
<li>เสา 4 ต้นตั้งบนคาน F01/F02 ตรวจฉากกับโต๊ะ แล้วคานบนท้ายและท่อนบนหน้าวางบนหัวเสา</li>
<li>แผ่นยึด: <b>หูบานพับ 3 หูแต่ละข้างต้องร้อยแท่ง Ø3 ผ่านทั้ง 3 รูขณะเชื่อม</b> ให้รูตรงแนวเดียวกัน ·
<b>แผ่นยึดมอเตอร์ขับซ้าย/ขวาร้อยแท่ง Ø8 ผ่านทั้งสองรู</b> ให้ตรงแกนเดียวกัน · แผ่นยึดแผ่นตั้งอาวุธให้ขันแผ่นตั้งจริงหรือแผ่นจำลองไว้ขณะเชื่อม</li>
<li>แต้มเชื่อมสลับซ้าย-ขวาเพื่อลดการบิด แล้วเชื่อมเต็มตามตารางข้อ 6 · เจียรแนวเชื่อมด้านที่แผ่น PC จะแนบ (ผิวนอกของรางและเสา) ให้เรียบ</li>
<li>ทำสีกันสนิมบาง ๆ (สีทำให้หนักขึ้น) ห้ามมีสีหรือสะเก็ดในรูของแผ่นยึด</li></ol>
<h2>8. พิกัดและการตรวจ</h2><ul>
<li>ตำแหน่งชิ้น ±0.5 · รูของแผ่นยึด ±0.2 · พื้นล่างของกรอบเรียบภายใน 0.5 · เส้นทแยงกรอบล่างต่างกันไม่เกิน 1</li>
<li>น้ำหนักเหล็กตามแบบ %(sg).0f g (ไม่รวมแนวเชื่อม/สี) <b>ขอให้ร้านชั่งโครงเสร็จแล้วแจ้งน้ำหนัก</b> รถทั้งคันเหลือเผื่อถึง 2,000 g เพียง %(margin).0f g</li></ul>
<h2>9. คำถามถึงร้าน</h2><ol>
<li>เชื่อมท่อหนา %(wall).1f มม. ได้ไหม ใช้ MIG หรือ TIG (ถ้าต้องหนากว่านี้ แจ้งก่อน ผู้สั่งต้องคำนวณน้ำหนักใหม่)</li>
<li>ร้านตัดแผ่นยึด 1.5 มม. เองได้ไหม หรือต้องส่งตัดเลเซอร์</li><li>ราคา ระยะเวลา และการทำสี</li></ol>
<h2>10. ยังไม่ครบ — ห้ามเชื่อมจนกว่าผู้สั่งจะส่งฉบับอนุมัติ</h2><ul>
<li>รูยึดหน้าแปลนมอเตอร์ขับบนแผ่น T มอเตอร์ (รอวัดมอเตอร์ JGA25 ตัวจริง) ตอนนี้มีเฉพาะรู Ø8.5 ของบ่ามอเตอร์</li>
<li>ขายึดแผ่นกันข้าง/ท้าย และรู rivnut สำหรับผนัง PC</li><li>ผลคำนวณความแข็งแรงเมื่อโดนอาวุธ</li></ul>
''' % dict(wall=B.WALL, tab=B.TAB, nt=len(tubes), tl=sum(axis_of(m.shape.BoundingBox())[1] for m in tubes), tubes=tube_rows,
           np=len(plate_rows), plates=plate_html, nw=len(B.welds), welds=weld_html, sg=steel_g, margin=mass['margin_to_2000_g'])
CSS = '''@page{size:A4;margin:11mm}body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:9pt;color:#111;line-height:1.45}
.stamp{border:2px solid #b36b00;color:#b36b00;font-weight:700;padding:3px 10px;display:inline-block;font-size:11pt}
h1{font-size:15pt;margin:6px 0}h2{font-size:11pt;margin:12px 0 4px;border-bottom:1px solid #999;page-break-after:avoid}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #aaa;padding:2px 5px;text-align:left;vertical-align:top}th{background:#eee}
tr{page-break-inside:avoid}.m{font-family:Menlo,monospace;font-size:7.5pt}.n{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
.bad{color:#b00}img{page-break-inside:avoid}li{margin-bottom:2px}'''
tmp = PKG / 'WELD_DRAWING_TH.html'
tmp.write_text('<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (CSS, body))
subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(PKG / 'WELD_DRAWING_TH.pdf'), tmp.as_uri()],
               check=True, capture_output=True, timeout=180)
tmp.unlink()
(PKG / 'manifest.json').write_text(json.dumps({'status': 'FOR_QUOTE_NOT_APPROVED_FOR_WELDING', 'datum_mm': DATUM, 'ids': ID,
                                               'plates': plate_rows, 'welds': B.welds, 'tube_wall_mm': B.WALL, 'tab_mm': B.TAB},
                                              ensure_ascii=False, indent=1) + '\n')
zpath = HERE / 'RFQ_WELD.zip'
with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
    for p in sorted(PKG.rglob('*')):
        if p.is_file():
            z.write(p, Path('RFQ_WELD') / p.relative_to(PKG))
print(json.dumps({'zip': str(zpath), 'tubes': len(tubes), 'plates': len(plate_rows), 'welds': len(B.welds)}, ensure_ascii=False))
