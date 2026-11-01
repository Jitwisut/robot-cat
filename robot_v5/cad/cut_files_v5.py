"""V5 cut files: 1:1 DXF per steel part, a 1:1 printable PDF template set, and the cut list.

Outlines are taken from the CAD solids in build_v5.py (importing it rebuilds the CAD), so the cut files
always match the checked CAD. Status CONCEPT: the CAD is concept-level, so these are DRAFT cut files
for a home build (hand cut / grind to the printed template) or a laser-shop quote, not a release.
Run from the repository root:
  .venv-v4-print/bin/python robot_v5/cad/cut_files_v5.py
Writes robot_v5/cad/cut/.
"""
import csv
import math
import sys
from pathlib import Path

import ezdxf
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_v5 as b  # noqa: E402  (builds the CAD and runs its checks)

OUT = HERE / 'cut'
OUT.mkdir(exist_ok=True)
STEEL = 7.85e-3

frame = {x['name']: x['shape'] for x in b.bodies}
arm0 = {x['name']: x['shape0'] for x in b.arm}

# name, solid, thin axis ('x' -> profile in YZ, 'z' -> profile in XY), qty, stock, note (Thai, for the cut list)
PARTS = [
    ('V5_Arm_Plate_R_2.3mm_steel', arm0['Arm_Plate_R_2.3'], 'x', 1, 'เศษแผ่นเหล็กดำ 2.3 มม.',
     'มีรู Ø6 ให้เพลามอเตอร์ เจาะรูยึดแท่นมอเตอร์ตามขาจริงของ A2212; ประกบกับแผ่น L แล้วเจาะรูแกนพร้อมกัน'),
    ('V5_Arm_Plate_L_2.3mm_steel', arm0['Arm_Plate_L_2.3'], 'x', 1, 'เศษแผ่นเหล็กดำ 2.3 มม.',
     'เหมือนแผ่น R แต่ไม่มีรูมอเตอร์'),
    ('V5_Tower_1.6mm_steel', frame['Tower_R_1.6'], 'x', 2, 'เศษแผ่นเหล็กดำ 1.6 มม.',
     'ตัวเดียวกันทั้ง L/R (พลิกด้าน); เจาะรู Ø8.2 สองแผ่นพร้อมกันก่อนเชื่อม'),
    ('V5_Base_Plate_1.6mm_steel', frame['Arm_Module_Base_1.6'], 'z', 1, 'เศษแผ่นเหล็กดำ 1.6 มม.',
     'เจาะรู M3 ยึดพื้นถาดตามตำแหน่งจริงตอนประกอบ (ยังไม่มีใน CAD)'),
    ('V5_Nose_Tongue_1.6mm_steel', frame['Nose_Tongue_1.6'], 'z', 1, 'เศษแผ่นเหล็กดำ 1.6 มม.',
     'เชื่อมต่อหน้าแผ่นฐาน ลอดช่องผนังหน้าถาด'),
    ('V5_Stop_Post_3.2x12_steel', frame['Stop_Post_R_3.2x12'], 'x', 2, 'เหล็กแบน 1/2" × 1/8" (12.7 × 3.2)',
     'ตัวเดียวกันทั้ง L/R; ปลายบนเอียงตามแขนในท่าฟาด; เชื่อมบนลิ้นใต้แผ่นแขนพอดี'),
    ('V5_Fork_12x4_steel', b.fork_local, 'x', 4, 'เหล็กแบน 1/2" × 4 มม. ตั้งสัน',
     'รู Ø4.2 สำหรับหมุด M4; ปลายหน้าเอียงเป็นลิ่ม'),
    ('V5_Saw_Disc_3T_3mm_SS400', arm0['Saw_Disc_3T_SS400_3mm'], 'x', 1, 'เศษแผ่นเหล็ก 3 มม. (SS400)',
     'เจียตามแบบ 1:1 ให้ฟันด้านหน้าตั้งฉาก; รูกลาง Ø24 + รู M3 3 รู PCD 29; ถ่วงสมดุลก่อนใช้'),
]
# not flat-cut: bought or lathe parts that still belong on the list
OTHER = [
    ('Pivot housing ring ×2', 'ท่อเหล็กดำ 3/4" (OD 26.7, หนา 2.3, ID ≈22.1) ตัดยาว 7 มม.',
     'ใส่ลูกปืน 608 (OD 22) ถ้า ID หลวมให้ใช้น้ำยาล็อกลูกปืน; เชื่อมที่ผิวนอกของแผ่นแขนตรงรูแกน'),
    ('Arbor doubler ×2', 'แหวนหนา M8 (DIN 7349: Ø24 × 3) หรือเจาะจากเหล็กแบน 3.2',
     'เชื่อมที่ผิวนอกแผ่นแขนตรงรูเพลาจาน'),
    ('Pivot shaft', 'สกรู M8 × 50 เกรด 12.9 + น็อตล็อก', 'ทะลุเสาค้ำทั้งสองแผ่น'),
    ('Saw arbor', 'สกรู M8 × 45 เกรด 12.9 + น็อตล็อก', 'เกรด 8.8 ไม่พอ (SF 1.09)'),
    ('Fork pin', 'สกรู M4 × 150 หรือเหล็กเส้น Ø4 ยาว 148 + แหวนล็อก', 'ร้อยงา 4 ซี่กับบล็อก PETG'),
    ('Saw hub + 2 spacers', '6061 กลึง (ร้านกลึง)', 'ดุม Ø34 ยาว 16 มีลูกปืน 608 2 ตัว + ร่องสายพาน'),
    ('Nose skid', 'UHMW/HDPE หนา 6 มม. ตัด 27.2 × 16 มม.', 'ใต้ลิ้นหน้า ยกจากพื้น 1 มม.'),
]


def plane_face(solid, axis):
    n = np.array({'x': (1, 0, 0), 'z': (0, 0, 1)}[axis])
    best = None
    for f in solid.Faces():
        if f.geomType() != 'PLANE':
            continue
        nn = np.array(f.normalAt().toTuple())
        if abs(abs(np.dot(nn, n)) - 1) > 1e-6:
            continue
        if best is None or f.Area() > best.Area():
            best = f
    return best


def to2d(p, axis):
    return (p.y, p.z) if axis == 'x' else (p.x, p.y)


def wire_entities(wire, axis):
    """[('circle', (cx, cy), r)] or [('poly', [(x, y), ...])] in 2D."""
    ents = []
    for e in wire.Edges():
        gt = e.geomType()
        if gt == 'CIRCLE' and e.IsClosed():
            c = e.arcCenter()
            ents.append(('circle', to2d(c, axis), e.radius()))
        elif gt == 'LINE':
            ents.append(('poly', [to2d(e.startPoint(), axis), to2d(e.endPoint(), axis)]))
        else:
            n = max(8, int(e.Length() / 0.2))
            ents.append(('poly', [to2d(p, axis) for p in e.positions(np.linspace(0, 1, n + 1))]))
    return ents


def outline(solid, axis):
    solid = solid.clean()                                          # merge coplanar faces left by fuse (plate + end discs)
    f = plane_face(solid, axis)
    outer = wire_entities(f.outerWire(), axis)
    holes = [wire_entities(w, axis) for w in f.innerWires()]
    pts = [p for k in outer if k[0] == 'poly' for p in k[1]]
    xs, ys = zip(*pts)
    x0, y0 = min(xs), min(ys)

    def shift(ents):
        out = []
        for k in ents:
            if k[0] == 'circle':
                out.append(('circle', (k[1][0] - x0, k[1][1] - y0), k[2]))
            else:
                out.append(('poly', [(x - x0, y - y0) for x, y in k[1]]))
        return out
    o, h = shift(outer), [shift(w) for w in holes]
    allp = [p for k in o if k[0] == 'poly' for p in k[1]]
    thk = solid.Volume() / f.Area()                                # exact for a prism (the OCC bbox is padded)
    return {'outer': o, 'holes': h, 'w': max(p[0] for p in allp), 'h': max(p[1] for p in allp),
            'thk': thk, 'area': f.Area(), 'vol': solid.Volume()}


def write_dxf(path, geo):
    doc = ezdxf.new('R2010', units=4)                              # millimetres
    doc.header['$INSUNITS'] = 4
    doc.layers.add('CUT', color=7)
    doc.layers.add('HOLES', color=1)
    msp = doc.modelspace()
    for layer, groups in (('CUT', [geo['outer']]), ('HOLES', geo['holes'])):
        for ents in groups:
            for k in ents:
                if k[0] == 'circle':
                    msp.add_circle(k[1], k[2], dxfattribs={'layer': layer})
                else:
                    msp.add_lwpolyline(k[1], dxfattribs={'layer': layer})
    doc.saveas(path)


rows, geos = [], {}
for name, solid, axis, qty, stock, note in PARTS:
    g = outline(solid, axis)
    geos[name] = (g, qty)
    fn = f'{name}_x{qty}.dxf'
    write_dxf(OUT / fn, g)
    holes = ['Ø%.1f' % (2 * w[0][2]) if len(w) == 1 and w[0][0] == 'circle' else 'ช่อง' for w in g['holes']]
    rows.append({'part': name, 'qty': qty, 'stock': stock, 'thickness_mm': round(g['thk'], 1),
                 'blank_mm': f"{g['w']:.1f} × {g['h']:.1f}", 'holes': ', '.join(holes) or '-',
                 'mass_g_each': round(g['vol'] * STEEL, 1), 'file': fn, 'note': note})

# ------------------------------------------------ stock totals
stock_tot = {}
for r in rows:
    w, h = (float(v) for v in r['blank_mm'].split(' × '))
    s = stock_tot.setdefault(r['stock'], {'pieces': 0, 'area_mm2': 0.0, 'bar_mm': 0.0, 'mass_g': 0.0})
    s['pieces'] += r['qty']
    s['area_mm2'] += r['qty'] * (w + 5) * (h + 5)                  # 5 mm kerf/edge allowance
    s['bar_mm'] += r['qty'] * (max(w, h) + 3)                      # 3 mm saw kerf per piece
    s['mass_g'] += r['qty'] * r['mass_g_each']

with open(OUT / 'cut_list_v5.csv', 'w', newline='', encoding='utf-8-sig') as fh:
    wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    wr.writeheader()
    wr.writerows(rows)

# ------------------------------------------------ 1:1 printable PDF (A4 portrait, actual size)
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

A4 = (210.0, 297.0)
M, GAP = 12.0, 8.0
items = sorted(((n, i + 1, q, g) for n, (g, q) in geos.items() for i in range(q)), key=lambda t: -t[3]['h'])


def draw_part(ax, g, ox, oy):
    for groups, lw in (([g['outer']], 0.6), (g['holes'], 0.5)):
        for ents in groups:
            for k in ents:
                if k[0] == 'circle':
                    (cx, cy), r = k[1], k[2]
                    t = np.linspace(0, 2 * math.pi, 120)
                    ax.plot(ox + cx + r * np.cos(t), oy + cy + r * np.sin(t), 'k-', lw=lw)
                    ax.plot([ox + cx - 2, ox + cx + 2], [oy + cy, oy + cy], 'k-', lw=0.3)
                    ax.plot([ox + cx, ox + cx], [oy + cy - 2, oy + cy + 2], 'k-', lw=0.3)
                else:
                    xs, ys = zip(*k[1])
                    ax.plot([ox + x for x in xs], [oy + y for y in ys], 'k-', lw=lw)


def new_page(page_no):
    fig = plt.figure(figsize=(A4[0] / 25.4, A4[1] / 25.4))
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, A4[0]); ax.set_ylim(0, A4[1]); ax.set_aspect('equal'); ax.axis('off')
    ax.text(M, A4[1] - 8, 'ROBOT V5 cut templates - PRINT AT 100 % (ACTUAL SIZE), no "fit to page"', fontsize=8, weight='bold')
    ax.text(M, A4[1] - 12, 'DRAFT from concept CAD (robot_v5/cad). The bar below must measure 100 mm.  Page %d' % page_no, fontsize=6.5)
    ax.plot([M, M + 100], [M - 4, M - 4], 'k-', lw=1.2)
    for x in (M, M + 50, M + 100):
        ax.plot([x, x], [M - 6, M - 2], 'k-', lw=0.8)
    ax.text(M + 102, M - 5, '100 mm', fontsize=6)
    return fig, ax


page = 1
with PdfPages(OUT / 'V5_cut_templates_1to1_A4.pdf') as pdf:
    fig, ax = new_page(page)
    x, y_top, row_h = M, A4[1] - 18, 0.0
    for name, i, qty, g in items:
        w, h = g['w'], g['h']
        if x + w > A4[0] - M:                                      # next row
            x, y_top, row_h = M, y_top - row_h - GAP - 6, 0.0
        if y_top - h < M + 4:                                      # next page
            pdf.savefig(fig); plt.close(fig)
            page += 1
            fig, ax = new_page(page)
            x, y_top, row_h = M, A4[1] - 18, 0.0
        draw_part(ax, g, x, y_top - h)
        label = '%s (%d/%d) t=%.1f' % (name.replace('V5_', ''), i, qty, g['thk'])
        ax.text(x, y_top - h - 4, label, fontsize=5.5)
        if 'Saw_Disc' in name:
            # Drawn as seen from +X (Y right, Z up): +X spin is counter-clockwise here, radial faces leading.
            (cx, cy) = next(w_[0][1] for w_ in g['holes'] if w_[0][0] == 'circle' and w_[0][2] > 10)
            t = np.linspace(math.radians(30), math.radians(150), 40)
            ax.plot(x + cx + 20 * np.cos(t), y_top - h + cy + 20 * np.sin(t), 'k-', lw=0.8)
            ax.annotate('', xy=(x + cx + 20 * math.cos(t[-1]), y_top - h + cy + 20 * math.sin(t[-1])),
                        xytext=(x + cx + 20 * math.cos(t[-3]), y_top - h + cy + 20 * math.sin(t[-3])),
                        arrowprops=dict(arrowstyle='->', lw=0.8))
            ax.text(x + cx - 12, y_top - h + cy + 23, 'SPIN: radial tooth faces lead', fontsize=5)
        x += max(w, 1.15 * len(label)) + GAP                       # label width at 5.5 pt ~ 1.15 mm/char
        row_h = max(row_h, h)
    pdf.savefig(fig); plt.close(fig)

# ------------------------------------------------ cut list (Thai)
md = ['# V5 — รายการตัดเหล็ก (cut list) · DRAFT', '',
      'สร้างจาก [`cut_files_v5.py`](../cut_files_v5.py) โดยดึงรูปทรงจาก CAD ที่ผ่านการตรวจแล้ว (`build_v5.py`) ทุกครั้ง แกน Z ขึ้น หน้า +Y หน่วย มม.',
      '**สถานะ DRAFT:** CAD ยังเป็นระดับแนวคิด ใช้ตัดชิ้นทดลอง หรือส่งร้านขอใบเสนอราคาได้ แต่ยังไม่ใช่ไฟล์ผลิตที่ปล่อยแล้ว', '',
      '## ไฟล์', '',
      '- `*.dxf`: ขนาดจริง 1:1 หน่วย มม. layer `CUT` คือเส้นขอบนอก และ layer `HOLES` คือรูกับช่องด้านใน ส่งร้านตัดเลเซอร์ได้',
      '- `V5_cut_templates_1to1_A4.pdf`: แบบขนาดจริง **พิมพ์ที่ 100% (Actual size) ห้ามเลือก Fit to page** แล้ววัดเส้นสเกลในหน้ากระดาษให้ได้ 100 มม. ก่อนใช้ จากนั้นตัดกระดาษ ติดกาวบนเหล็ก แล้วตัดหรือเจียตามเส้น',
      '- `cut_list_v5.csv`: ข้อมูลเดียวกับตารางด้านล่าง เปิดใน Excel ได้', '',
      '## ชิ้นเหล็กที่ต้องตัด', '',
      '| ชิ้น | จำนวน | วัตถุดิบ | หนา (มม.) | ขนาดชิ้นตัด (มม.) | รู | กรัม/ชิ้น | ไฟล์ | หมายเหตุ |',
      '|---|---|---|---|---|---|---|---|---|']
for r in rows:
    md.append('| {part} | {qty} | {stock} | {thickness_mm} | {blank_mm} | {holes} | {mass_g_each} | `{file}` | {note} |'.format(**r))
md += ['', '## วัตถุดิบที่ต้องเตรียม', '',
       '| วัตถุดิบ | จำนวนชิ้น | ต้องมีอย่างน้อย (เผื่อรอยตัด) | น้ำหนักชิ้นงานรวม |', '|---|---|---|---|']
for k, s in stock_tot.items():
    side = math.sqrt(s['area_mm2']) * 1.25
    amount = ('ยาว %.0f มม.' % s['bar_mm']) if k.startswith('เหล็กแบน') else ('เศษแผ่นประมาณ %.0f × %.0f มม.' % (side, side))
    md.append('| %s | %d | %s | %.0f g |' % (k, s['pieces'], amount, s['mass_g']))
md += ['', 'สำหรับแผ่นเหล็ก ให้**ซื้อเศษแผ่น**ที่มีขนาดอย่างน้อยเท่าที่บอกในตาราง ไม่ต้องซื้อทั้งแผ่น 4×8 ฟุต ส่วนเหล็กแบนมักขายเป็นเส้นยาว 6 ม. ลองถามร้านว่าขายเป็นเศษได้ไหม', '',
       '## ชิ้นที่ไม่ได้ตัดจากแผ่นเหล็ก (ซื้อหรือกลึง)', '', '| ชิ้น | วัสดุ/ขนาด | หมายเหตุ |', '|---|---|---|']
for n, m_, note in OTHER:
    md.append('| %s | %s | %s |' % (n, m_, note))
md += ['', '## ลำดับการทำ', '',
       '1. พิมพ์ PDF ที่ 100% แล้ววัดเส้นสเกลให้ได้ 100 มม. ถ้าไม่ได้ให้พิมพ์ใหม่',
       '2. ตัดชิ้นทั้งหมด แล้วเจาะรูโดยเริ่มด้วยดอกนำ จากนั้นขยายเป็นขนาดจริง ชิ้นที่เป็นคู่ (แผ่นแขน, เสาค้ำแกนแขน) ให้ประกบกันแล้วเจาะพร้อมกัน รูแกนจะได้ตรงกันทั้งสองข้าง',
       '3. เชื่อมแขน: แหวนรองลูกปืน (housing) กับแหวนเสริมที่ผิวนอกของแผ่นแขน ระหว่างเชื่อมให้ร้อยแท่ง Ø8 ผ่านรูไว้ แขนทั้งสองข้างจะได้อยู่ในแนวเดียวกัน',
       '4. เชื่อมโมดูลแขน: ลิ้นต่อหน้าแผ่นฐาน ตั้งเสาค้ำแกนแขนบนแผ่นฐาน และตั้งเสากันแขนบนลิ้น ให้ประกอบแขนเข้าที่ในท่าฟาดก่อน แล้ว**เชื่อมเสากันขณะที่แผ่นแขนวางทับอยู่บนเสา** จะได้แตะกันพอดี',
       '5. จาน: เจียตามแบบ แล้วตั้งบนแกน Ø8 ตรวจสมดุล ถ้าด้านไหนหนักให้เจาะเนื้อออกด้านนั้น',
       '6. พ่นสีกันสนิม ยกเว้นผิวที่ลูกปืนนั่ง', '',
       '**ยังไม่มีใน CAD:** รูยึดแท่นมอเตอร์ A2212 (วัดจากของจริง), รูยึดแผ่นฐานกับพื้นถาด, ข้อต่อจากเซอร์โว/clutch เข้าแขน', '']
(OUT / 'CUT_LIST_V5.md').write_text('\n'.join(md), encoding='utf-8')

print('wrote', sorted(p.name for p in OUT.iterdir()))
for r in rows:
    print(r['part'], r['qty'], r['thickness_mm'], r['blank_mm'], r['holes'], r['mass_g_each'])
print('pages', page, 'stock', {k: (round(v['mass_g']), round(v['bar_mm'])) for k, v in stock_tot.items()})
