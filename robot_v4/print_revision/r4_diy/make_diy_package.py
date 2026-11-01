"""Home-build package for V4 R4-DIY: what to buy where, what to cut, how to weld it with a stick welder.

Runs build_r4_diy.py (imports it, so the CAD and checks are rebuilt first) and calc/frame_strength_diy.py, then
writes DIY_PACKAGE/ + DIY_PACKAGE.zip:
  BUILD_GUIDE_TH.pdf      Thai guide: shopping list per shop, tools, ARC settings + practice coupons (hold point),
                          tube cut list, parts list, build order with jigs, weld list with stitch lengths, holes drilled
                          after welding, wedgelet folding, discs + balancing (hold point), lathe-shop list, weights, checks
  TEMPLATES_1to1_TH.pdf   every hand-cut flat part at 1:1 (print at 100 %, check the 100 mm bars), holes marked
  DXF/                    the same flat parts as DXF (laser shop also fine)
  LATHE/                  STEP files for the lathe shop (weapon hub, motor pulley, spacers, wheel hub)
  FRAME_WELDMENT_DIY.step the welded frame as named parts (P/F ids), robot coordinates (Z up, nose +Y, mm)
  images/
Status: CONCEPT CAD, DIY build drawings (not a shop release). r4_steel/WELD_RELEASE (W1) is unchanged.
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r4_diy/make_diy_package.py
"""
import html
import json
import math
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
PR = HERE.parent
sys.path.insert(0, str(HERE))
import build_r4_diy as B                                # noqa: E402  (builds the CAD)
import cadquery as cq                                   # noqa: E402
import ezdxf                                            # noqa: E402
import numpy as np                                      # noqa: E402
import matplotlib                                       # noqa: E402
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['Sukhumvit Set', 'Tahoma']
import matplotlib.pyplot as plt                         # noqa: E402

PKG = HERE / 'DIY_PACKAGE'
REV, REV_DATE = 'D1', '2026-10-27'
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
RELEASED_DXF = PR / 'r3' / 'PRODUCTION' / 'V4_R3_METAL_RELEASED' / 'DXF'
LATHE_SRC = [PR / 'r3' / 'output_R3' / 'DRAFT' / 'metal' / n for n in
             ('Beater_Hub_6061_D30.step', 'Motor_Pulley_D30_1to1.step', 'Spacer_Inner_ID8.2_OD11.step',
              'Spacer_Outer_L_ID8.2_OD11.step', 'Spacer_Outer_R_ID8.2_OD11.step')] + [PR / 'r4_steel' / 'LATHE_HUB' / 'WHEEL_HUB_6061.step']
e = html.escape
rep = B.report
if rep['interference'] or rep['frame_members_not_touching'] or rep['edge_only_contacts'] or rep['invalid_bodies'] or rep['rotor_sweep_hits']:
    raise RuntimeError('CAD checks failed; fix build_r4_diy.py first')
subprocess.run([sys.executable, str(HERE / 'calc' / 'frame_strength_diy.py')], check=True, capture_output=True)
STRENGTH = json.loads((HERE / 'calc' / 'frame_strength_diy.json').read_text())
if STRENGTH['summary']['design_fails']:
    raise RuntimeError('strength design fails: %s' % STRENGTH['summary']['design_fails'])
SIM_PATH = PR.parent / 'mujoco_sim' / 'r4_diy_2wd.json'
SIM = json.loads(SIM_PATH.read_text()) if SIM_PATH.exists() else None

TH = {'Frame_Rear_Cross': 'คานท้าย', 'Frame_Bulkhead_Cross': 'คานขวางหน้า (กลาง)', 'Frame_Bulkhead_Stub': 'คานขวางหน้า (ท่อนนอก)',
      'Frame_Rail': 'รางข้าง', 'Frame_Hinge_Beam': 'คานบานพับ', 'Frame_Motor_Cross': 'คานรองมอเตอร์ขับ', 'Frame_Clamp_Bar': 'คานกดมอเตอร์ขับ',
      'Tab_Drive_Motor': 'แผ่นยึดมอเตอร์ขับ', 'Tab_Hinge': 'หูบานพับ wedgelet', 'Tab_Battery_Strap': 'หูสายรัดแบต', 'Tab_Wall': 'หูยึดผนัง PC',
      'Clamp_Drive_Motor_Post': 'เสาตัวกดมอเตอร์ขับ', 'Upright_Steel': 'แผ่นตั้งอาวุธ', 'Rung': 'แผ่นค้ำแผ่นตั้ง',
      'Weapon_Base_Plate': 'แผ่นฐานอาวุธ', 'Weapon_Top_Brace_Steel': 'แผ่นยึดบนแผ่นตั้ง', 'Weapon_Motor_Mount_Steel': 'แผ่นยึดมอเตอร์อาวุธ',
      'Weapon_Motor_Mount_Foot': 'แผ่นรองใต้แผ่นยึดมอเตอร์อาวุธ', 'Weapon_Motor_Mount_Gusset': 'แผ่นค้ำสามเหลี่ยมแผ่นยึดมอเตอร์อาวุธ',
      'Hall_Post_Steel': 'ขายึดเซนเซอร์ Hall', 'Weapon_Motor_Mount_Shelf': 'แผ่นรับบนหลังรางของแผ่นยึดมอเตอร์อาวุธ', 'Wedgelet_DIY': 'wedgelet (ลิ่มหน้า)', 'Skin_PC2_Floor': 'พื้น PC', 'Skin_PC2_Rear_Wall': 'ผนังท้าย PC',
      'Skin_PC2_Side_Wall': 'ผนังข้าง PC', 'Skin_PC2_Lid': 'ฝา PC', 'Electronics_Plate_PC2': 'แผ่นวางบอร์ด PC', 'Skirt_Bracket': 'ขายึดแผ่นกัน',
      'Rear_Skid': 'แผ่นไถลท้าย', 'Nose_Skid': 'แผ่นไถลหน้า', 'Beater_Disc': 'จานอาวุธ', 'Skirt_Plate': 'แผ่นกัน (skirt)'}


def thai(name):
    key = max((k for k in TH if name.startswith(k)), key=len)
    rest = name[len(key):]
    side = ' ซ้าย' if ('_L' in rest or rest.endswith('L')) else ' ขวา' if ('_R' in rest or rest.endswith('R')) else ''
    return TH[key] + side


tubes = [b for b in B.steel if b.material == 'steel_tube']
sheets = [b for b in B.steel if b.material == 'steel_tab']
ID = {b.name: 'F%02d' % (i + 1) for i, b in enumerate(tubes)}
ID.update({b.name: 'P%02d' % (i + 1) for i, b in enumerate(sheets)})
DATUM = (-(B.WALL_X - B.SKIN), B.REAR_Y + B.SKIN, B.RAIL_Z0)      # rear-left-bottom corner of the rear cross


def rel(bb):
    return tuple(round(v, 1) + 0.0 for v in (bb.xmin - DATUM[0], bb.ymin - DATUM[1], bb.zmin - DATUM[2]))


# ---------------------------------------------------------------- flat parts -> outlines (mm, 2D polylines)
def flat_shape(shape, t):
    """Rotate a plate so its thin axis is Z; return the mid-thickness section."""
    bb = shape.BoundingBox()
    dims = {'x': bb.xlen, 'y': bb.ylen, 'z': bb.zlen}
    thin = min(dims, key=dims.get)
    if abs(dims[thin] - t) > 0.15:
        raise RuntimeError('not a %.1f mm plate: %s' % (t, dims))
    s = shape.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.center.z))
    if thin == 'x':
        s = s.rotate(cq.Vector(), cq.Vector(0, 1, 0), 90)
    elif thin == 'y':
        s = s.rotate(cq.Vector(), cq.Vector(1, 0, 0), 90)
    return cq.Workplane('XY').add(s).section(0)


def outlines_of_section(sec):
    """Every edge of a planar section as a polyline; circles reported separately (holes)."""
    lines, holes = [], []
    for edge in sec.edges().vals():
        if edge.geomType() == 'CIRCLE' and abs(edge.startPoint().sub(edge.endPoint()).Length) < 1e-6:
            c = edge.Center(); holes.append((c.x, c.y, edge.radius()))
            continue
        n = 2 if edge.geomType() == 'LINE' else 24
        pts = [edge.positionAt(t) for t in np.linspace(0, 1, n)]
        lines.append([(p.x, p.y) for p in pts])
    return lines, holes


def outlines_of_dxf(path):
    doc = ezdxf.readfile(path)
    lines, holes = [], []
    for ent in doc.modelspace():
        t = ent.dxftype()
        if t == 'LINE':
            lines.append([tuple(ent.dxf.start)[:2], tuple(ent.dxf.end)[:2]])
        elif t == 'CIRCLE':
            holes.append((ent.dxf.center[0], ent.dxf.center[1], ent.dxf.radius))
        elif t == 'ARC':
            c, r, a0, a1 = ent.dxf.center, ent.dxf.radius, math.radians(ent.dxf.start_angle), math.radians(ent.dxf.end_angle)
            if a1 < a0: a1 += 2 * math.pi
            lines.append([(c[0] + r * math.cos(a), c[1] + r * math.sin(a)) for a in np.linspace(a0, a1, 24)])
        elif t == 'LWPOLYLINE':
            pts = [tuple(p[:2]) for p in ent.get_points()]
            lines.append(pts + ([pts[0]] if ent.closed else []))
    return lines, holes


def wedgelet_flat():
    """Flat blank of the R wedgelet (the L one is its mirror): x across (27..88 -> 0..61), s down-slope from the rear
    edge. Plate outline = the top face (longest), two tongue cut outlines (ear hull below the underside, unfolded into
    the plate beside its fold line), fold lines dashed. Ear holes are drilled after folding, through the frame tabs."""
    spec = B.spec
    a = math.atan2(spec['underside_rear_yz_mm'][1] - spec['underside_tip_yz_mm'][1], spec['underside_tip_yz_mm'][0] - spec['underside_rear_yz_mm'][0])
    t = spec['thickness_mm']
    L_under = math.hypot(spec['underside_tip_yz_mm'][0] - spec['underside_rear_yz_mm'][0], spec['underside_tip_yz_mm'][1] - spec['underside_rear_yz_mm'][1])
    L_top = L_under + t * math.tan(a)                  # front end ground vertical: the top face runs t*tan(a) further
    x0 = 27.0
    W = 88.0 - 27.0
    lines = [[(0, 0), (W, 0), (W, L_top), (0, L_top), (0, 0)]]
    folds = []
    sd = [((np.array(p) - B.REAR) @ B.U, -((np.array(p) - B.REAR) @ B.N)) for p in B.EAR_YZ]     # (s, depth below underside)
    for ex0, ex1, d in B.EARS_X:
        xf = (ex1 if d > 0 else ex0) - x0              # fold line (the slot side of the ear)
        poly = []
        for s_, dep in sd:
            dep = max(dep, 0.0)
            poly.append((xf + d * dep, s_))
        poly.append(poly[0])
        lines.append(poly)
        folds.append([(xf, 0), (xf, B.EAR_S)])
    return lines, [], folds, (W, L_top)


FLAT = []          # dicts: key, name, title, material, t, qty, lines, holes, folds, note


def add_flat(key, title, material, t, qty, lines, holes, folds=(), note=''):
    xs = [p[0] for l in lines for p in l]; ys = [p[1] for l in lines for p in l]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    lines = [[(x - cx, y - cy) for x, y in l] for l in lines]
    holes = [(x - cx, y - cy, r) for x, y, r in holes]
    folds = [[(x - cx, y - cy) for x, y in l] for l in folds]
    FLAT.append({'key': key, 'title': title, 'material': material, 't': t, 'qty': qty, 'lines': lines, 'holes': holes, 'folds': folds,
                 'size': (round(max(xs) - min(xs), 1), round(max(ys) - min(ys), 1)), 'note': note})


STOCK = {1.6: 'เหล็กแผ่นดำ 1.6', 2.3: 'เหล็กแผ่นดำ 2.3', 3.2: 'เหล็กแบน 2"×1/8" (3.2)'}
for b in sheets:
    t = B.THK[b.name]
    sec = flat_shape(B.AS_WELDED.get(b.name, b.shape), t)
    lines, holes = outlines_of_section(sec)
    note = ''
    if b.name.startswith('Tab_Drive_Motor'):
        note = 'รู Ø8.5 เจาะซ้อนคู่ซ้าย-ขวา · รู M3/Ø5.5 เจาะหลังเชื่อมจากมอเตอร์จริง'
    elif b.name.startswith('Upright_Steel'):
        note = 'รู Ø8.0 เจาะซ้อนคู่ซ้าย-ขวาก่อนเชื่อม'
    elif b.name.startswith('Tab_Hinge'):
        note = 'เจาะ Ø3.2 ซ้อน 6 แผ่นพร้อมกัน'
    add_flat(ID[b.name], thai(b.name), STOCK[t], t, 1, lines, holes, note=note)
wl, wh, wf, wsize = wedgelet_flat()
add_flat('W1', 'wedgelet ขวา (+X)', 'เหล็กแผ่นดำ 1.6', 1.6, 1, wl, wh, wf, 'ตัดตามเส้นลิ้น พับลง 90° ที่เส้นประ · เจียรขอบหน้าเอียง 35.5° ให้ตั้งฉากพื้น · รูหูเจาะหลังพับ')
add_flat('W2', 'wedgelet ซ้าย (−X) = กลับด้านของ W1', 'เหล็กแผ่นดำ 1.6', 1.6, 1, [[(-x, y) for x, y in l] for l in wl], [], [[(-x, y) for x, y in l] for l in wf],
         'กลับด้าน: ลิ้นอยู่ฝั่งตรงข้าม')
dl, dh = outlines_of_dxf(RELEASED_DXF / 'Beater_Disc_L.dxf')
add_flat('D1', 'จานอาวุธ (ทำ 2 ชิ้นเหมือนกัน)', 'เหล็กแผ่น 6 มม. SS400', 6.0, 2, dl, dh,
         note='Ø3.4 ×4 เคาเตอร์ซิงค์ 90° Ø6.3 ด้านนอก · Ø14 รูกลาง · Ø8 ×2 รูลดน้ำหนัก · ชั่งให้ 2 ชิ้นต่างกัน ≤ 0.5 g')
for key, nm, title, q in (('S1', 'Skirt_Plate_08mm_R', 'แผ่นกันข้าง (ซ้าย = พลิกแผ่น)', 2), ('S2', 'Skirt_Plate_08mm_Rear', 'แผ่นกันท้าย', 1)):
    sl, sh = outlines_of_dxf(RELEASED_DXF / (nm + '.dxf'))
    add_flat(key, title, 'อลูมิเนียมแผ่น 0.8', 0.8, q, sl, sh, note='รู Ø2.2 ขยายเป็นร่องปรับสูง ~1 มม. · ลบคม')
for k, (name, fl) in enumerate(rep['skirt_brackets_flat'].items()):
    if k not in (0, 8): continue                      # side brackets identical (8), rear identical (3): one template each
    W_, L1, L2 = fl['width'], fl['wall_leg'], fl['seat_leg']
    add_flat('A%d' % (1 if k == 0 else 2), 'ขายึดแผ่นกัน%s' % (' ข้าง' if k == 0 else ' ท้าย'), 'อลูมิเนียม 1.5', 1.5, 8 if k == 0 else 3,
             [[(0, 0), (W_, 0), (W_, L1 + L2), (0, L1 + L2), (0, 0)]],
             [(W_ / 2 + fl['m3_hole_offset_from_centre'], fl['m3_hole_from_leg_end'], 1.7), (W_ / 2, L1 + fl['m2_hole_from_bend'], 1.2)],
             [[(0, L1), (W_, L1)]], 'พับ %.1f° ที่เส้นประ ด้วยปากกาจับชิ้นงาน' % fl['bend_deg_from_flat'])
pcs = [b for b in B.new if b.material == 'PC'] + [b for b in B.new if b.material == 'UHMW']
for b in pcs:
    sec = flat_shape(b.shape, B.THK[b.name])
    lines, holes = outlines_of_section(sec)
    add_flat('PC' if b.material == 'PC' else 'U', thai(b.name), 'PC ตัน 2 มม.' if b.material == 'PC' else 'UHMW/HDPE %g มม.' % B.THK[b.name],
             B.THK[b.name], 1, lines, holes, note='รู rivnut/สกรู เจาะตอนประกอบโดยใช้โครงจริงเป็นแบบ' if b.material == 'PC' else '')
for i, f in enumerate([f for f in FLAT if f['key'] in ('PC', 'U')]):
    f['key'] = '%s%02d' % (f['key'], i + 1)
for title in {f['title'] for f in FLAT}:                 # same name twice (e.g. two rungs a side): number them
    same = [f for f in FLAT if f['title'] == title]
    if len(same) > 1:
        for k, f in enumerate(same):
            f['title'] += ' #%d' % (k + 1)

# ---------------------------------------------------------------- write DXF + 1:1 template pages
if PKG.exists():
    shutil.rmtree(PKG)
for d in ('DXF', 'images', 'LATHE'):
    (PKG / d).mkdir(parents=True)
for f in FLAT:
    doc = ezdxf.new('R2010'); msp = doc.modelspace()
    for layer in ('CUT', 'BEND', 'NOTES'):
        doc.layers.add(layer)
    for l in f['lines']:
        msp.add_lwpolyline(l, dxfattribs={'layer': 'CUT'})
    for x, y, r in f['holes']:
        msp.add_circle((x, y), r, dxfattribs={'layer': 'CUT'})
    for l in f['folds']:
        msp.add_lwpolyline(l, dxfattribs={'layer': 'BEND'})
    msp.add_text('%s t%g x%d' % (f['key'], f['t'], f['qty']), dxfattribs={'layer': 'NOTES', 'height': 2}).set_placement((0, -f['size'][1] / 2 - 5))
    f['dxf'] = '%s_%s.dxf' % (f['key'], f['title'].split(' ')[0].replace('/', '-'))
    doc.saveas(PKG / 'DXF' / f['dxf'])

PAGE_W, PAGE_H, MARGIN, GAP = 190.0, 270.0, 4.0, 9.0


X_MAX, Y0, Y_MAX = PAGE_W - MARGIN - 10, MARGIN + 14, PAGE_H - 22     # right strip and bottom strip keep the 100 mm bars


def pack(items):
    """First-fit-decreasing shelf packing on A4 portrait (mm), over every open page; a part turns 90 deg when it only
    fits that way or when that makes it shorter (the label line needs the width)."""
    pages = []                                    # each: {'shelves': [[y, h, x_next]], 'y_next': y, 'placed': [...]}
    for f in sorted(items, key=lambda f: -(f['size'][0] * f['size'][1])):
        w0, h0 = f['size']
        options = [(w0, h0, False), (h0, w0, True)]
        options = [o for o in options if o[0] + 2 <= X_MAX - MARGIN and o[1] + GAP + 2 <= Y_MAX - Y0]
        options.sort(key=lambda o: (o[1], o[2]))
        if not options:
            raise RuntimeError('part larger than an A4 page: %s %s' % (f['key'], f['size']))
        done = False
        for pg in pages:
            for w, h, rot in options:
                W, H = max(w, 48) + 2, h + GAP + 2          # label needs ~48 mm
                for sh in pg['shelves']:
                    if H <= sh[1] and sh[2] + W <= X_MAX:
                        pg['placed'].append((f, sh[2], sh[0], rot)); sh[2] += W + 4; done = True; break
                if done: break
                if pg['y_next'] + H <= Y_MAX:
                    pg['shelves'].append([pg['y_next'], H, MARGIN + W + 4]); pg['placed'].append((f, MARGIN, pg['y_next'], rot))
                    pg['y_next'] += H + 2; done = True; break
            if done: break
        if not done:
            w, h, rot = options[0]
            W, H = max(w, 48) + 2, h + GAP + 2
            pages.append({'shelves': [[Y0, H, MARGIN + W + 4]], 'y_next': Y0 + H + 2, 'placed': [(f, MARGIN, Y0, rot)]})
    return [pg['placed'] for pg in pages]


def svg_page(placed, n, total):
    o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%gmm" height="%gmm" viewBox="0 0 %g %g">' % (PAGE_W, PAGE_H, PAGE_W, PAGE_H)]
    o.append('<text x="%g" y="7" font-size="4" font-weight="700">แบบตัด 1:1 · Robot V4 R4-DIY rev %s · หน้า %d/%d · พิมพ์ 100%% (Actual size) แล้ววัดแถบ 100 มม. ก่อนใช้</text>' % (MARGIN, REV, n, total))
    for f, x0, y0, rot in placed:
        w, h = f['size']
        if rot: w, h = h, w
        cx, cy = x0 + 1 + w / 2, y0 + GAP + 1 + h / 2
        tf = (lambda x, y: (cx - y, cy - x)) if rot else (lambda x, y: (cx + x, cy - y))
        o.append('<text x="%.1f" y="%.1f" font-size="3" font-weight="700">%s %s</text>' % (x0 + 1, y0 + 4, e(f['key']), e(f['title'])))
        o.append('<text x="%.1f" y="%.1f" font-size="2.3">%s · หนา %g · ทำ %d ชิ้น · %g×%g</text>' % (x0 + 1, y0 + 7.4, e(f['material']), f['t'], f['qty'], *f['size']))
        for l in f['lines']:
            pts = ' '.join('%.2f,%.2f' % tf(x, y) for x, y in l)
            o.append('<polyline points="%s" fill="none" stroke="#000" stroke-width="0.25"/>' % pts)
        for l in f['folds']:
            pts = ' '.join('%.2f,%.2f' % tf(x, y) for x, y in l)
            o.append('<polyline points="%s" fill="none" stroke="#0a4fa3" stroke-width="0.3" stroke-dasharray="1.5 1"/>' % pts)
        for x, y, r in f['holes']:
            hx, hy = tf(x, y)
            o.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="none" stroke="#b00" stroke-width="0.25"/>' % (hx, hy, r))
            o.append('<path d="M%.2f %.2fh%.2fM%.2f %.2fv%.2f" stroke="#b00" stroke-width="0.15"/>' % (hx - r - 1, hy, 2 * r + 2, hx, hy - r - 1, 2 * r + 2))
            o.append('<text x="%.2f" y="%.2f" font-size="1.8" fill="#b00">Ø%g</text>' % (hx + r + 0.6, hy - r - 0.3, round(2 * r, 1)))
    yb = PAGE_H - 10
    o.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#000" stroke-width="0.4"/>' % (MARGIN, yb, MARGIN + 100, yb))
    for k in range(0, 101, 10):
        o.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#000" stroke-width="0.25"/>' % (MARGIN + k, yb, MARGIN + k, yb - (3 if k % 50 == 0 else 1.5)))
    o.append('<text x="%g" y="%g" font-size="3">100 มม. (แนวนอน)</text>' % (MARGIN + 102, yb + 1))
    o.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#000" stroke-width="0.4"/>' % (PAGE_W - MARGIN, yb, PAGE_W - MARGIN, yb - 100))
    o.append('<text x="%g" y="%g" font-size="3" text-anchor="middle" transform="rotate(-90 %g %g)">100 มม. (แนวตั้ง)</text>' % (
        PAGE_W - MARGIN + 3, yb - 50, PAGE_W - MARGIN + 3, yb - 50))
    o.append('</svg>')
    return ''.join(o)


cut_flats = [f for f in FLAT]
pages = pack(cut_flats)
tmpl_html = ''.join('<div class="pg">%s</div>' % svg_page(p, i + 1, len(pages)) for i, p in enumerate(pages))
CSS_T = '@page{size:A4;margin:0}body{margin:0;font-family:"Sukhumvit Set",sans-serif}.pg{width:210mm;height:297mm;padding:10mm;box-sizing:border-box;page-break-after:always}'


def pdf(html_body, css, out):
    tmp = out.with_suffix('.html')
    tmp.write_text('<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (css, html_body))
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(out), tmp.as_uri()],
                   check=True, capture_output=True, timeout=240)
    tmp.unlink()


pdf(tmpl_html, CSS_T, PKG / 'TEMPLATES_1to1_TH.pdf')

# ---------------------------------------------------------------- images, STEP, lathe files
COL = {'steel_tube': '#c0392b', 'steel_tab': '#e67e22'}


def iso(path, view=(28, -60), others=False):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(11, 8), facecolor='white')
    ax = fig.add_subplot(projection='3d')
    shown = B.steel + ([b for b in B.bodies if b not in B.steel and b.material != 'keepout' and not b.name.startswith('Skin')] if others else [])
    for b in shown:
        v, t = B.AS_WELDED.get(b.name, b.shape).tessellate(0.3, 0.3)
        v = np.array([p.toTuple() for p in v])
        ax.add_collection3d(Poly3DCollection(v[np.array(t)], facecolors=COL.get(b.material, '#b8bec6'), edgecolors='#333' if b in B.steel else 'none',
                                             linewidths=0.12, alpha=0.95 if b in B.steel else 0.55))
        if b in B.steel and not others:
            c = b.shape.Center()
            ax.text(c.x, c.y, c.z + 6, ID[b.name], fontsize=7 if b.material == 'steel_tab' else 9,
                    color='#000' if b.material == 'steel_tube' else '#7a3a00', fontweight='bold', ha='center')
    ax.set_xlim(-95, 95); ax.set_ylim(-115, 115); ax.set_zlim(0, 70); ax.set_box_aspect((190, 230, 70)); ax.view_init(elev=view[0], azim=view[1])
    ax.set_xlabel('X'); ax.set_ylabel('Y (หัวรถ +Y)'); ax.set_zlabel('Z')
    fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)


iso(PKG / 'images' / 'iso_ids.png')
iso(PKG / 'images' / 'iso_ids_front.png', view=(30, 35))
iso(PKG / 'images' / 'in_robot.png', view=(32, -55), others=True)
shutil.copy(B.OUT / 'assembly_preview.png', PKG / 'images' / 'assembly_preview.png')


def step_name(m):
    bb = B.AS_WELDED.get(m.name, m.shape).BoundingBox()
    if m.material == 'steel_tube':
        return '%s_%s_12x12x%.1f_L%.0f' % (ID[m.name], m.name.replace('Frame_', ''), B.WALL, max(bb.xlen, bb.ylen, bb.zlen))
    return '%s_%s_t%.1f' % (ID[m.name], m.name, B.THK[m.name])


assy = cq.Assembly(name='R4_DIY_FRAME_WELDMENT')
for m in sorted(B.steel, key=lambda m: ID[m.name]):
    assy.add(B.AS_WELDED.get(m.name, m.shape), name=step_name(m),
             color=cq.Color(0.80, 0.36, 0.30) if m.material == 'steel_tube' else cq.Color(0.95, 0.66, 0.28))
assy.save(str(PKG / 'FRAME_WELDMENT_DIY.step'), exportType='STEP')
for p in LATHE_SRC:
    shutil.copy(p, PKG / 'LATHE' / p.name)
shutil.copy(PR / 'r4_steel' / 'LATHE_HUB' / 'HUB_DRAWING_TH.pdf', PKG / 'LATHE' / 'WHEEL_HUB_DRAWING_TH.pdf')
for p in ('MACHINING.md',):
    shutil.copy(PR / 'r3' / p, PKG / 'LATHE' / ('R3_' + p))


# ---------------------------------------------------------------- 1:1 plan of the bottom frame (lay the tubes on the table)
def plan_svg():
    lo_x = min(m.shape.BoundingBox().xmin for m in B.steel) - DATUM[0] - 4
    hi_x = max(m.shape.BoundingBox().xmax for m in B.steel) - DATUM[0] + 4
    hi_y = max(m.shape.BoundingBox().ymax for m in B.steel) - DATUM[1] + 6
    lo_y = -16.0
    s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.1fmm" height="%.1fmm" viewBox="%.1f %.1f %.1f %.1f">' % (
        hi_x - lo_x, hi_y - lo_y, lo_x, -hi_y, hi_x - lo_x, hi_y - lo_y)]
    for m in sorted(B.steel, key=lambda m: (m.material != 'steel_tube', m.shape.BoundingBox().zmin)):
        bb = m.shape.BoundingBox()
        x0, y0, x1, y1 = bb.xmin - DATUM[0], bb.ymin - DATUM[1], bb.xmax - DATUM[0], bb.ymax - DATUM[1]
        upper = bb.zmin - DATUM[2] > 20
        tube = m.material == 'steel_tube'
        s.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" stroke="%s" stroke-width="0.25"%s/>' % (
            x0, -y1, x1 - x0, y1 - y0, 'none' if upper else '#f3c1b9' if tube else '#f8d9b5', '#7a1c10' if tube else '#a35400',
            ' stroke-dasharray="1.5 1"' if upper else ''))
        if tube or (x1 - x0) * (y1 - y0) > 150:
            ly = y0 - 3.5 if upper else (y0 + y1) / 2 - 1     # an upper member sits over a bottom one: label below its edge
            s.append('<text x="%.2f" y="%.2f" font-size="%.1f" text-anchor="middle" font-family="Menlo,monospace">%s</text>' % (
                (x0 + x1) / 2, -ly, 2.6 if tube else 2.0, ID[m.name] + ("'" if upper else '')))
    s.append('<circle cx="0" cy="0" r="1.2" fill="blue"/><text x="2" y="5" font-size="3" fill="blue">DATUM 0,0</text>')
    for k in range(0, 101, 10):
        s.append('<line x1="%d" y1="12" x2="%d" y2="%s" stroke="#000" stroke-width="0.2"/>' % (k, k, 9 if k % 50 else 7))
        s.append('<line x1="%.1f" y1="%d" x2="%s" y2="%d" stroke="#000" stroke-width="0.2"/>' % (lo_x + 3, -k, lo_x + (6 if k % 50 else 8), -k))
    s.append('<line x1="0" y1="12" x2="100" y2="12" stroke="#000" stroke-width="0.3"/><text x="50" y="15.5" font-size="3" text-anchor="middle">100 มม. (X)</text>')
    s.append('<line x1="%.1f" y1="0" x2="%.1f" y2="-100" stroke="#000" stroke-width="0.3"/>' % (lo_x + 3, lo_x + 3))
    s.append('<text x="%.1f" y="-102" font-size="3">100 มม. (Y) ↑ หัวรถ</text></svg>' % (lo_x + 1))
    return ''.join(s)


# ---------------------------------------------------------------- guide tables
def axis_of(bb):
    return max((('X', bb.xlen), ('Y', bb.ylen), ('Z', bb.zlen)), key=lambda t: t[1])


tube_rows = ''.join('<tr><td><b>%s</b></td><td>%s</td><td class="n">%.1f</td><td>%s</td><td class="n">%.1f, %.1f, %.1f</td></tr>' % (
    ID[m.name], e(thai(m.name)), axis_of(m.shape.BoundingBox())[1], axis_of(m.shape.BoundingBox())[0], *rel(m.shape.BoundingBox())) for m in tubes)
tube_total = sum(axis_of(m.shape.BoundingBox())[1] for m in tubes)
sheet_rows = ''
for m in sheets:
    f = next(f for f in FLAT if f['key'] == ID[m.name])
    sheet_rows += '<tr><td><b>%s</b></td><td>%s</td><td>%s</td><td class="n">%g × %g</td><td>%s</td><td class="n">%.1f, %.1f, %.1f</td></tr>' % (
        ID[m.name], e(f['title']), e(STOCK[B.THK[m.name]]), *f['size'],
        e(', '.join(sorted({'Ø%g' % round(2 * h[2], 1) for h in f['holes']})) or '–') + (' · ' + e(f['note']) if f['note'] else ''),
        *rel(m.shape.BoundingBox()))
def how(a, b):
    na, nb = (a, b) if a <= b else (b, a)
    tt = lambda n: n.startswith('Frame_')
    if tt(a) and tt(b):
        return 'ท่อ+ท่อ: แต้ม 2 มุมก่อน แล้วเชื่อมรอบทีละด้าน ด้านละ ≤ 12 มม. รอให้เย็นก่อนด้านถัดไป'
    if 'Upright' in a + b and 'Base' in a + b:
        return 'ทั้งสองหน้าแผ่นตั้ง: แนวละ 3 ช่วง × 10 มม. (y 64–74, 76–86, 88–98)'
    if 'Upright' in a + b and 'Bulkhead_Cross' in a + b:
        return 'เท้าแผ่นตั้งชนหน้าคานขวาง: ทั้งสองหน้า เต็มความสูง 10 มม.'
    if 'Upright' in a + b and 'Rung' in a + b:
        return 'แผ่นค้ำชนแผ่นตั้ง: ทั้งสองหน้าแผ่นค้ำ เต็มความสูง 20 มม.'
    if 'Base' in a + b and 'Rail' in a + b:
        return 'ขอบแผ่นฐานชนรางด้านใน: ด้านบน 2 ช่วง × 10 มม.'
    if 'Base' in a + b and 'Bulkhead' in a + b:
        return 'ขอบแผ่นฐานชนหน้าคานขวาง: ด้านบน เป็นช่วง 10 มม. เว้น 10 มม.'
    if 'Motor_Mount' in a + b and ('Foot' in a + b or 'Gusset' in a + b) and 'Frame' not in a + b:
        return 'ทำบนโต๊ะก่อนติดโครง (ข้อ 7.5): ทั้งสองหน้า เต็มแนว'
    if 'Motor_Mount_Foot' in a + b:
        return 'กลับโครงคว่ำ เชื่อมจากด้านล่าง (ด้านบนเป็นช่อง 1.7 มม. ลวดเข้าไม่ได้)'
    if 'Motor_Mount_Shelf' in a + b:
        return 'เชื่อมจากด้านบน เต็มแนว'
    if 'Motor_Mount_Steel' in a + b:
        return 'บนหลังคานขวาง: ทั้งสองหน้า เต็มแนว 12 มม. · ขอบหลังชนคานรองมอเตอร์: เฉพาะหน้าด้านใน'
    if 'Tab_Drive_Motor' in a + b:
        return 'บนหลังราง: ด้านใน (ฝั่งมอเตอร์) เต็มแนว 32 มม. ด้านนอกมีที่แค่ 0.4 มม. ไม่ต้องเชื่อม'
    if 'Tab_Hinge' in a + b:
        return 'หูบานพับ: ร้อยแท่ง Ø3 (หรือสกรู M3) ผ่าน 3 หูก่อน แต้ม 2 จุดต่อหู'
    return 'แผ่น+ท่อ/แผ่น: ช่วงละ 10 มม. เว้น 10 มม. เล็งอาร์กไปชิ้นที่หนากว่า'


weld_rows = ''
for i, w in enumerate(rep['welds']):
    c = [w['contact_centre_mm'][k] - DATUM[k] for k in range(3)]
    weld_rows += '<tr><td>W%02d</td><td><b>%s</b> + <b>%s</b></td><td class="n">%.0f, %.0f, %.0f</td><td class="n">%g × %g</td><td>%s</td></tr>' % (
        i + 1, ID[w['a']], ID[w['b']], *c, *w['contact_size_mm'], e(how(w['a'], w['b'])))
drill_rows = ''.join('<tr><td><b>%s</b></td><td>%s</td><td class="n">Ø%.1f</td><td class="n">%+.1f, %+.1f</td></tr>' % (
    ID[h['part']], e(h['what']), h['d_mm'], h['yz_mm'][0] - B.FRONT_Y, h['yz_mm'][1] - 32.0) for h in rep['drill_after_weld'])
mass = rep['mass']
paths = {(p['path'][:40], p['case']): p for p in STRENGTH['paths']}
str_rows = ''.join('<tr><td>%s</td><td>%s</td><td class="n">%s</td><td>%s</td></tr>' % (
    e(p['path']), e(p['case']), p['sf'] if p['sf'] is not None else '–', 'ผ่าน' if p['pass'] else ('ยอมรับ (ทิศที่ไม่เกิด)' if p.get('accepted_non_credible') else 'ไม่ผ่าน' if 'rejected' not in p['note'] else 'ไม่ใช้ (ตัวเลือกที่ตัดทิ้ง)'))
    for p in STRENGTH['paths'] if 'check' not in p['case'] or not p['pass'] or 'fore' in p['case'])
sim_txt = ''
if SIM:
    tt = SIM['tests']
    sim_txt = ('จอดนิ่ง: ล้อรับ %.0f%%/%.0f%% · ออกตัวเต็มความเร็วสูงสุด %.2f m/s ไม่ยกหน้า · เบรกกลับทิศ: ปลายฟันต่ำสุด %.1f มม. จากพื้น (ไม่แตะ) · '
               'แผ่นกันข้างแตะพื้นตอนจอด (รับน้ำหนัก %.1f%%%%) · หมุนตัวเต็มที่ขณะจานหมุน: ล้อข้างหนึ่งเหลือ %.1f%% (แรง gyro เหมือนรุ่น W1) · จำกัดความเร็วหมุนตัวใน firmware ที่ 5 rad/s: ล้อเบาเหลือ %.0f%%') % (
        tt['rest']['min_50ms_wheel_load_pct']['left'], tt['rest']['min_50ms_wheel_load_pct']['right'], tt['launch']['top_speed_m_s'],
        tt['brake']['min_tip_z_mm'], sum(v for k, v in tt['rest']['last_0.5s_mean_load_pct'].items() if k.startswith('skirt')), tt['spin_rotor_on']['min_50ms_wheel_load_pct']['right'],
        tt.get('spin_rotor_on_yawlimit_5', tt['spin_rotor_on'])['min_50ms_wheel_load_pct']['right'])

g = mass['groups_g']
GROUP_TH = {'R3 parts kept + bought': 'ชิ้นเดิมของ R3 + ของซื้อ (มอเตอร์ ไฟฟ้า จาน ดุม ล้อ สกรู ฯลฯ)', 'frame tubes': 'ท่อโครง 12×12×1.2',
            'frame + weapon sheet/flat parts': 'แผ่นเหล็กโครง + ชุดอาวุธ (1.6 / 2.3 / 3.2)', 'PC skins': 'แผ่น PC 2 มม.',
            'weld beads + spatter + paint (ARC)': 'แนวเชื่อม ARC + สะเก็ด + สี'}
body = '''<span class="stamp">แบบสร้างเอง (DIY) · CONCEPT CAD · rev %(rev)s · %(date)s</span>
<h1>Robot V4 R4-DIY — คู่มือสร้างเองด้วยเครื่องเชื่อมไฟฟ้า (ARC)</h1>
<p>หน่วย มม. · แกน Z ขึ้น หัวรถ +Y · <b>ซ้าย = −X</b> (ยืนท้ายรถมองไปหัวรถ) · DATUM (0,0,0) = มุมซ้าย-ท้าย-ล่างของคานท้าย F01 ·
ต้นฉบับคือ <span class="m">FRAME_WELDMENT_DIY.step</span> และ <span class="m">TEMPLATES_1to1_TH.pdf</span> · แบบนี้แทน W1 สำหรับการสร้างเองเท่านั้น (ไฟล์ W1 ไม่ถูกแก้)</p>
<img src="images/assembly_preview.png" style="width:100%%">
<h2>1. เปลี่ยนอะไรจากแบบเดิม (W1) และทำไม</h2>
<table><tr><th>เดิม (ต้องสั่งร้าน)</th><th>DIY</th><th>ซื้อที่</th></tr>
<tr><td>ท่อ 10×10×1.0/1.5 (ไม่มีขายทั่วไป)</td><td>เหล็กกล่องดำ 1/2"×1/2" หนา 1.2 (12×12×1.2) ทั้งโครง</td><td>ร้านเหล็กทั่วไป</td></tr>
<tr><td>แผ่นยึดตัดเลเซอร์ 1.5</td><td>เหล็กแผ่นดำ 1.6 ตัดเป็นสี่เหลี่ยมด้วยหินเจียร เจาะเอง</td><td>ร้านเหล็ก (เศษแผ่น)</td></tr>
<tr><td>แผ่นตั้งอาวุธ 6061 หนา 6 + แผ่นยึด/สกรู M3</td><td>เหล็กแบน 2"×1/8" (3.2) <b>เชื่อมติดโครง</b> รูปเดิม</td><td>ร้านเหล็ก</td></tr>
<tr><td>brace บน/ล่าง 6061 หนา 3</td><td>แผ่นยึดบน 1.6 + แผ่นฐานอาวุธ 1.6 (รวม brace ล่างกับแผ่นพื้นเดิม)</td><td>ร้านเหล็ก</td></tr>
<tr><td>แผ่นยึดมอเตอร์อาวุธ 6061 หนา 3</td><td>เหล็กแผ่น <b>2.3</b> + แผ่นรอง + แผ่นค้ำสามเหลี่ยม (1.6 ไม่ผ่านการตก 0.5 ม.)</td><td>ร้านเหล็ก</td></tr>
<tr><td>จาน Hardox 450 ตัดเลเซอร์</td><td>เหล็กแผ่น 6 มม. SS400 เจียรตามแบบ 1:1 รูปเดิม (พลังงาน 97 J เท่าเดิม ฟันสึกเร็วกว่า)</td><td>ร้านเหล็ก (เศษแผ่น 6)</td></tr>
<tr><td>wedgelet 65Mn + carrier 6061 กลึง + แกน Ø3 กลึงร่อง E-clip</td><td>เหล็กแผ่น 1.6 มีลิ้นพับเป็นหูบานพับ + สกรู M3×55 + น็อตล็อก</td><td>ร้านเหล็ก, ร้านน็อต</td></tr>
<tr><td>เสา Hall 6061 กลึง</td><td>แผ่นเหล็ก 1.6 รูปตัว L เชื่อมบนคานขวาง</td><td>ร้านเหล็ก</td></tr>
<tr><td>เสาท้าย 2 ต้น + คานบนท้าย</td><td><b>ตัดออก</b> (ผู้ใช้เลือก) ฝายึดผนังด้วยฉากอลูมิเนียมสำเร็จ 4 ตัว ผนังยึดหูยึดผนัง 4 หู</td><td>ไทวัสดุ/ฮาร์ดแวร์</td></tr>
<tr><td>เพลาอาวุธ M8×80</td><td>M8×75 เกรด 12.9 (แผ่นตั้งบางลง)</td><td>ร้านน็อต</td></tr>
<tr><td>แผ่นไถลหน้า 3 มม.</td><td>UHMW/HDPE 5 มม. (เติมช่องใต้แผ่นฐาน)</td><td>ร้านพลาสติก หรือเขียง PE</td></tr></table>
<p><b>ยังต้องใช้ร้านกลึง (ทำด้วยหินเจียร/สว่านไม่ได้):</b> ดุมอาวุธ (บ่าลูกปืน 608 + ร่องสายพาน), รอกมอเตอร์ (รู Ø4), spacer 3 ตัว, ดุมล้อ 2 ตัว — ไฟล์ในโฟลเดอร์ <span class="m">LATHE/</span> รูปทรงเดิมทุกชิ้น ·
<b>แผ่น PC ตัน 2 มม.</b> ซื้อร้านพลาสติก/อะคริลิค (ไทวัสดุมีแต่แผ่นกลวง 6 มม. และลอน ใช้แทนไม่ได้) · <b>อลูมิเนียม 0.8/1.5</b> ร้านอลูมิเนียม/ร้านกระจกอลูมิเนียม (ไทวัสดุไม่มีแผ่นหนา)</p>
<h2>2. น้ำหนัก (CAD %(total).1f g · เหลือถึง 2,000 g: <span class="%(mcls)s">%(margin).1f g</span>)</h2>
<table><tr><th>กลุ่ม</th><th class="n">g</th></tr>%(groups)s</table>
<p>แนวเชื่อม ARC + สะเก็ด + สี คิดไว้ 8%% ของเหล็ก (<b>สมมติ</b>) · CG y %(cgy).1f (หน้าแกนล้อ %(cga).1f มม.) ล้อขับรับ %(wl).1f%% แรงดัน %(push).2f N ·
<b class="bad">เหลือเผื่อน้อย: ชั่งชิ้นจริงทุกขั้นตามข้อ 11</b> ถ้าเกิน: จานหนา 5 มม. (−27 g, พลังงาน −15%%, คำนวณไว้ใน r4_steel/calc) · รูลดน้ำหนักแผ่นตั้ง Ø12 ×2 ต่อแผ่น (−11 g) <b>เป็นแค่ค่าประมาณน้ำหนัก ยังไม่ได้ตรวจความแข็งแรงหรือใส่ใน CAD</b></p>
<h2>3. รายการซื้อแยกตามร้าน</h2>
<table><tr><th>ร้าน</th><th>ของ</th><th>จำนวน</th><th>ราคาโดยประมาณ</th></tr>
<tr><td rowspan="5">ร้านเหล็กทั่วไป</td><td>เหล็กกล่องดำ (ไม่ชุบ) 1/2"×1/2" หนา 1.2</td><td>1 เส้น 6 ม. (ใช้ %(tl).0f มม. + ฝึกเชื่อม) ขอตัดแบ่งได้</td><td>~86 บาท (2.4 กก. × ~36 บาท/กก.)<sup>1</sup></td></tr>
<tr><td>เศษเหล็กแผ่นดำ 1.6 มม.</td><td>~250×250 มม.</td><td>ถามราคา</td></tr>
<tr><td>เศษเหล็กแผ่นดำ 2.3 มม.</td><td>~70×60 มม.</td><td>ถามราคา</td></tr>
<tr><td>เหล็กแบน 2"×1/8" (50.8×3.2)</td><td>~150 มม.</td><td>ถามราคา</td></tr>
<tr><td>เศษเหล็กแผ่น 6 มม. (SS400)</td><td>~140×70 มม. (จาน 2 ชิ้น)</td><td>ถามราคา</td></tr>
<tr><td rowspan="5">ร้านอุปกรณ์เชื่อม/ไทวัสดุ/โฮมโปร</td><td>ลวดเชื่อม E6013 <b>Ø2.0</b> (Ø2.6 ใหญ่ไปสำหรับท่อ 1.2)</td><td>1 กก.</td><td>ถามราคา (โฮมโปรมีแต่ 2.6/3.2)<sup>2</sup></td></tr>
<tr><td>ใบตัดเหล็ก 4" หนา 1 มม.</td><td>10 ใบ</td><td>9–19 บาท/ใบ<sup>3</sup></td></tr>
<tr><td>ใบเจียร/ใบทรายเรียงซ้อน 4" เบอร์ 60–80</td><td>2 ใบ</td><td>ถามราคา</td></tr>
<tr><td>ดอกสว่าน HSS 2.5, 3.2, 3.4, 3.6, 5.5, 8.0, 8.5 + ดอกเคาเตอร์ซิงค์ 90°</td><td>1 ชุด</td><td>ถามราคา</td></tr>
<tr><td>สีสเปรย์รองพื้นกันสนิม, แปรงลวด, เหล็กนำศูนย์</td><td>-</td><td>ถามราคา</td></tr>
<tr><td rowspan="4">ร้านน็อต/ฮาร์ดแวร์</td><td>M8×75 เกรด 12.9 + น็อตล็อก M8 + แหวน 2</td><td>1 ชุด</td><td>ถามราคา</td></tr>
<tr><td>M3×55 + น็อตล็อก M3 (แกนบานพับ)</td><td>2 ชุด</td><td>ถามราคา</td></tr>
<tr><td>rivnut เหล็ก M3 + คีมยิงรีเวทนัท</td><td>30 ตัว + 1 อัน</td><td>ถามราคา<sup>4</sup></td></tr>
<tr><td>สกรู M3 หัวจม/หัวกลม + น็อตล็อก, ฉากอลูมิเนียมเล็ก 4 ตัว</td><td>-</td><td>ถามราคา</td></tr>
<tr><td rowspan="2">ร้านพลาสติก/อะคริลิค</td><td><b>PC ตัน (solid) 2 มม.</b> ขอตัดตามขนาดในแบบ 1:1 (PC01–PC06)</td><td>~0.1 ตร.ม.</td><td>ถามราคา</td></tr>
<tr><td>UHMW หรือ HDPE 3 มม. และ 5 มม. (เขียง PE ใช้ได้)</td><td>ชิ้นเล็ก</td><td>ถามราคา</td></tr>
<tr><td>ร้านอลูมิเนียม/กระจก</td><td>อลูมิเนียมแผ่น 0.8 มม. (แผ่นกัน 3 ชิ้น) + 1.5 มม. (ขายึด 11 ชิ้น)</td><td>~220×60 / 15×40 ×11</td><td>ถามราคา</td></tr>
<tr><td>ร้านกลึง</td><td>ดุมอาวุธ, รอกมอเตอร์, spacer 3, ดุมล้อ 2 (โฟลเดอร์ LATHE/)</td><td>7 ชิ้น</td><td>ขอใบเสนอราคา</td></tr></table>
<p style="font-size:7.5pt">1 steellead.com/steel-tube-price.html (ตาราง 12×12×1.2 = 2.4 กก./6 ม., มัธยฐาน 36 บาท/กก., ต.ค. 2569) · 2 homepro.co.th/p/1171491 · 3 homepro.co.th/p/1313266, /p/1232012, /p/1231985 · 4 ไม่พบราคาไทวัสดุ — ราคาอื่นทั้งหมดต้องถามร้าน</p>
<p><b>เครื่องมือ:</b> ตู้เชื่อม ARC (มีแล้ว), หน้ากากเชื่อมกรองแสง, ถุงมือหนัง, หินเจียร 4", สว่าน + แท่นหรือปากกาจับ, แคลมป์ตัว C 4 ตัว, ฉากเหล็ก, เวอร์เนียร์, ตะไบ, <b>ตาชั่งดิจิตอลละเอียด 0.1 g</b> (ถ่วงจาน), โต๊ะเหล็กหรือแผ่นเรียบที่เชื่อมได้</p>
<h2>4. ตั้งค่าเครื่องเชื่อม และ <span class="bad">จุดหยุดที่ 1: ฝึกเชื่อมก่อนแตะชิ้นจริง</span></h2><ul>
<li>ลวด E6013 Ø2.0 · ท่อ 1.2 ↔ แผ่น 1.6: <b>~40–55 A</b> · แผ่นตั้ง 3.2 ↔ แผ่นฐาน 1.6: ~55–65 A · ถ้าตู้สลับขั้วได้ ใช้ <b>ลวดต่อขั้วลบ (DCEN)</b> ไฟลึกน้อย ทะลุยากกว่า</li>
<li>อาร์กสั้น เชื่อมทีละช่วง <b>≤ 10 มม.</b> เล็งอาร์กไปที่ชิ้นหนากว่า แล้วลากไปหาชิ้นบาง · แต้มทุกชิ้นให้ครบก่อน แล้วเชื่อมสลับซ้าย-ขวา หน้า-หลัง รอเย็นระหว่างช่วง</li>
<li>เจียรสนิม/สีออกรอบรอยเชื่อม 10 มม. · <b class="bad">ห้ามใช้ท่อหรือแผ่นชุบกัลวาไนซ์/ทาสีมา</b> (ควันพิษ เชื่อมยาก) · เคาะสแลกทุกช่วงก่อนเชื่อมทับ</li>
<li><b>ชิ้นทดสอบ 3 ชิ้นจากเศษ:</b> (ก) ท่อ+ท่อ ตัว T (ข) แผ่น 1.6 วางบนท่อ (ค) แผ่น 3.2 ตั้งบนแผ่น 1.6 · ผ่านเมื่อ: ไม่มีรูทะลุ แนวหลอมทั้งสองชิ้น
และหักงอด้วยปากกาจับแล้ว<b>เนื้อเหล็กงอก่อนแนวเชื่อมหลุด</b> · ถ้าทะลุซ้ำ ๆ: ลดกระแส 5 A, ใช้ลวด Ø1.6, แต้มเป็นจุดต่อกัน · ถ้ายังไม่ได้ ห้ามเปลี่ยนเป็นท่อหนาขึ้นเอง (น้ำหนักเหลือแค่ %(margin).0f g) ให้กลับมาคุยก่อน</li></ul>
<h2>5. ตัดท่อ (%(nt)d ท่อน รวม %(tl).0f มม.)</h2>
<p>ตัดปลายฉาก 90° ทุกท่อน ด้วยใบตัด 1 มม. ลบคม เขียนรหัสด้วยปากกาเมทัลลิก · ตำแหน่ง = มุมต่ำสุดของชิ้นจาก DATUM</p>
<table><tr><th>รหัส</th><th>ชื่อ</th><th>ยาว</th><th>แนว</th><th>ตำแหน่ง X, Y, Z</th></tr>%(tubes)s</table>
<p>ตัดเพิ่ม: <b>ท่อ 12×12 ยาว 54.0 มม. 1 ท่อน = spacer หลอก</b> (ข้อ 7) และเศษท่อสำหรับฝึกเชื่อม</p>
<h2>6. ชิ้นแผ่น (%(ns)d ชิ้นเหล็ก + wedgelet 2 + จาน 2 + PC/อลูมิเนียม) — แบบ 1:1 ใน TEMPLATES_1to1_TH.pdf</h2>
<p>พิมพ์แบบ 100%% วัดแถบ 100 มม. ทั้งสองแนว ติดกาวบนแผ่นเหล็ก ตอกนำศูนย์ทุกรู<b>ก่อน</b>ตัด เจาะนำ Ø2.5 แล้วขยาย ตัดตามเส้นด้วยใบตัด 1 มม. แล้วเจียรเข้าเส้น ·
ชิ้นซ้าย/ขวามีแบบแยก (ห้ามใช้แบบเดียวกลับด้านเอง ยกเว้นที่เขียนไว้)</p>
<table><tr><th>รหัส</th><th>ชื่อ</th><th>วัสดุ</th><th>ขนาด</th><th>รู / หมายเหตุ</th><th>ตำแหน่ง X, Y, Z</th></tr>%(sheets)s</table>
<h2>7. ลำดับการประกอบ (โครงต้องเย็นทุกครั้งก่อนวัด)</h2><ol>
<li><b>เจาะซ้อนก่อนเชื่อม (ให้รูตรงแกน):</b> แผ่นตั้งซ้าย+ขวา หนีบซ้อนกันให้ขอบตรง เจาะ Ø3 → Ø6 → <b>Ø8.0</b> ทีเดียวทะลุทั้งคู่ ·
แผ่นยึดมอเตอร์ขับซ้าย+ขวา เจาะ Ø8.5 ซ้อนกัน · หูบานพับทั้ง 6 หู เจาะ Ø3.2 ซ้อนกัน</li>
<li><b>กรอบล่าง</b> วางบนแบบ 1:1 (หน้าสุดท้าย) บนโต๊ะเรียบ: คานท้าย F01 + รางข้าง 2 เส้น + คานขวางหน้า (กลาง + ท่อนนอก) + คานรองมอเตอร์ + คานบานพับ ·
แคลมป์ แล้ว<b>วัดเส้นทแยงต่างกันไม่เกิน 1 มม.</b> แต้มทุกมุม ตรวจใหม่ แล้วค่อยเชื่อมเต็มสลับมุม</li>
<li><b>แผ่นยึดมอเตอร์ขับ</b> วางบนหลังราง ร้อยแท่ง Ø8 (หรือสกรู M8) ผ่านทั้งสองรู แต้ม ตรวจว่าแท่งขนานคานรองมอเตอร์ แล้วเชื่อม · ถอดแท่ง
แล้ววางแท่งกลม Ø25 ยาว 50 (หรือมอเตอร์จริงที่<b>ถอดออกก่อนเชื่อม</b>) เพื่อจัดเสาตัวกด + คานกด ให้ช่องยางบน/ล่าง %(pad_lo).1f / %(pad_hi).1f มม.</li>
<li><b>ชุดอาวุธ (สำคัญที่สุด):</b> แผ่นฐานอาวุธวางบนพื้นชิดหน้าคานขวางและผิวในราง แต้ม · ประกอบแผ่นตั้ง 2 แผ่นบน<b>สกรู M8 + spacer หลอกท่อ 54.0 มม.</b> ขันแน่น ·
ตั้งบนแผ่นฐาน ให้เท้าแผ่นตั้งชนหน้าคานขวาง วัดระยะแผ่นตั้งถึงรางสองข้างให้เท่ากัน (±0.3) และสกรู M8 ตั้งฉากกับราง (ฉาก) แต้ม ·
<b>เชื่อมแนวแผ่นตั้ง→แผ่นฐาน ด้านนอกให้เสร็จก่อนใส่แผ่นค้ำ</b> (หลังใส่แผ่นค้ำจะเหลือช่องแคบเกินกว่าจะเชื่อม) แล้วใส่แผ่นค้ำ 4 แผ่น และแผ่นยึดบน แต้ม · เชื่อมเต็มตามตารางข้อ 8 สลับซ้าย-ขวา <b>โดย spacer หลอกยังอยู่</b> · ถอดเมื่อเย็นแล้วเท่านั้น
<b class="bad">ห้ามเชื่อมขณะใส่ลูกปืน ดุม หรือจาน</b></li>
<li><b>แผ่นยึดมอเตอร์อาวุธ (ช่องระหว่างแผ่นกับรางกว้างแค่ 1.7 มม. ลวดเชื่อมเข้าไม่ได้ ต้องทำตามลำดับนี้):</b>
(ก) <b>บนโต๊ะ:</b> เชื่อมแผ่น 2.3 + แผ่นรองใต้ (%(foot_id)s) + แผ่นค้ำสามเหลี่ยม ทั้งสองหน้าให้เสร็จเป็นชุดเดียว ·
(ข) วางชุดนี้ลงโครง ให้หน้าแผ่นด้านมอเตอร์อยู่ที่ <b>X −42.0 (ห่างผิวในรางซ้าย 4.0 มม.)</b> เพราะเป็นตัวกำหนดแนวสายพาน (ร่องรอกต้องตรงร่องดุมอาวุธ ±0.5) ·
แต้ม แล้วเชื่อมแผ่นกับหลังคานขวางทั้งสองหน้า · (ค) วางแผ่นรับบนหลังราง (%(shelf_id)s) ชนผิวนอกแผ่นยึด เชื่อมจากด้านบน ·
(ง) <b>กลับโครงคว่ำ (ก่อนใส่พื้น PC)</b> เชื่อมแผ่นรองใต้กับผิวในราง คานรองมอเตอร์ และคานขวาง จากด้านล่าง</li>
<li>หูบานพับบนหน้าคานบานพับ: ร้อยแท่ง Ø3/สกรู M3 ผ่าน 3 หูต่อข้างก่อนแต้ม · ขายึด Hall, หูสายรัดแบต, หูยึดผนัง</li>
<li>เคาะสแลก แปรงลวด ตรวจรอยร้าว/รูทะลุ เติมเฉพาะจุด · <b>ชั่งโครง</b> (ข้อ 11) · พ่นสีรองพื้นบาง ๆ ห้ามสีเข้ารู</li>
<li>เจาะหลังเชื่อม: รูมอเตอร์ขับ (ข้อ 9), รู rivnut ใต้พื้น/ผนังท้าย และรูผนังข้างที่หูยึดผนัง โดยใช้แผ่น PC จริงเป็นแบบ</li></ol>
<h2>8. รายการจุดเชื่อม (%(nw)d จุด จากผิวสัมผัสใน CAD) และความยาวแนวเชื่อม</h2>
<table><tr><th>จุด</th><th>ชิ้น</th><th>ตำแหน่ง X, Y, Z</th><th>ผิวสัมผัส</th><th>วิธีเชื่อม</th></tr>%(welds)s</table>
<h2>9. รูที่เจาะหลังเชื่อม (มอเตอร์ขับ JB361 — วัดมอเตอร์จริงก่อน)</h2>
<p>ตัวเลขจากแบบผู้ขาย ยังไม่วัดของจริง ถ้าต่างเกิน 0.2 มม. ใช้ค่าที่วัดได้ · เจาะจากด้านล้อ ใช้รู Ø8.5 เป็นศูนย์ · รู M3 ใช้ Ø3.6 + แหวนรอง</p>
<table><tr><th>แผ่น</th><th>รู</th><th>Ø</th><th>จากศูนย์รู Ø8.5 (Y, Z)</th></tr>%(drill)s</table>
<h2>10. wedgelet และจานอาวุธ</h2><ul>
<li><b>wedgelet:</b> ตัดแผ่นตามแบบ W1/W2 ตัดตามเส้นลิ้น 2 ลิ้น พับลง 90° ที่เส้นประด้วยปากกาจับ (ลิ้นชี้ลงใต้แผ่นด้านพื้น) เจียรขอบหน้าให้ตั้งฉากพื้นเมื่อประกอบ ·
วาง wedgelet ในตำแหน่งจริง หนีบไว้ แล้ว<b>เจาะ Ø3.2 ผ่านรูหูบานพับบนโครงทะลุลิ้น</b> (ใช้หูบนโครงเป็นแบบ) · ใส่สกรู M3×55 + น็อตล็อก ·
ตรวจ: ยกขึ้นได้ ≥ 14° และลงได้ ≥ 2° โดยไม่ติด (CAD: ขึ้น %(sw_up)d° ลง %(sw_dn)d°)</li>
<li><b>จาน (D1 ×2):</b> เหล็ก 6 มม. SS400 รูปเดิม → พลังงานอาวุธเท่าเดิม แต่ฟันอ่อนกว่า Hardox จะสึก/บี้ ให้ตรวจหลังทุกแมตช์ ·
เจาะรูทั้งหมดจากแบบก่อนตัด เคาเตอร์ซิงค์ด้านนอก · <b class="bad">จุดหยุดที่ 2:</b> ชั่งจาน 2 ชิ้นให้ต่างกัน ≤ 0.5 g ·
ประกอบดุม+จาน+ลูกปืนบนแกน M8 วางบนสันมีด 2 อัน ปล่อยหมุนเอง ด้านที่หยุดอยู่ล่างคือด้านหนัก เจาะรูตื้น Ø3 ด้านนั้นทีละน้อยจนหยุดได้ทุกมุม ·
หมุนครั้งแรกไฟต่ำหลังที่กำบัง</li></ul>
<h2>11. ชั่งตรวจระหว่างสร้าง</h2>
<table><tr><th>ขั้น</th><th class="n">ตามแบบ (g)</th><th>ถ้าเกิน</th></tr>
<tr><td>โครงเชื่อมเสร็จ ก่อนทำสี (ท่อ + แผ่นเหล็กทุกชิ้น)</td><td class="n">%(steel).0f + แนวเชื่อม ≤ %(bead).0f</td><td>เกินแบบ: ทั้งคันจะเกิน 2,000 g ได้ — หยุดและกลับมาคำนวณทางลดก่อน</td></tr>
<tr><td>แผ่น PC ทั้งหมด</td><td class="n">%(pc).0f</td><td>ตรวจความหนา 1.9–2.1</td></tr>
<tr><td>ทั้งคันพร้อมแข่ง</td><td class="n">%(total).0f</td><td><b>ต้อง ≤ 2,000</b></td></tr></table>
<h2>12. ความแข็งแรง (calc/frame_strength_diy.py) — SF ต่ำสุดที่ใช้ออกแบบ %(sf).2f</h2>
<table><tr><th>เส้นทางแรง</th><th>กรณี</th><th class="n">SF</th><th>ผล</th></tr>%(strength)s</table>
<p>สมมติ: เหล็ก SS400 fy 235 MPa, ลวด E6013 แนวเชื่อม 0.6×420 MPa, ขาเชื่อม = แผ่นบาง (1.6 หรือผนังท่อ 1.2), สกรู M3 ฮาร์ดแวร์เกรด 4.8 ·
แรงดันขึ้นจากใต้จานเป็นทิศที่ไม่เกิดกับจานที่ฟันหน้าหมุนขึ้น (เหมือน W1)</p>
<h2>13. จำลองการวิ่ง MuJoCo (mujoco_sim/r4_diy_2wd.json)</h2><p>%(sim)s ·
<b>ตั้งเพดานความเร็วหมุนตัวใน firmware ตอนจานหมุน ~5 rad/s</b> (เหมือนข้อแนะนำ W1)</p>
<h2>14. ยังไม่ยืนยัน</h2><ul>
<li>ฝีมือเชื่อม ARC บนท่อ 1.2: ต้องผ่านชิ้นทดสอบ (ข้อ 4) · น้ำหนักแนวเชื่อมจริง (คิด 8%%)</li>
<li>ขนาดมอเตอร์ขับ/อาวุธจริง, ความยาวสกรูยึดมอเตอร์อาวุธ (แผ่น 2.3 บางกว่า 3 มม. เดิม 0.7 มม. — สกรูต้องไม่จมลึกเกินเกลียวในมอเตอร์)</li>
<li>เซนเซอร์ Hall ติดกับขาเหล็ก: ลองวัดสัญญาณกับแม่เหล็กในรอกก่อนประกอบจริง</li>
<li>ฟันจาน SS400 สึกเร็ว · ราคาส่วนใหญ่ต้องถามร้าน</li>
<li><b>ผนังข้าง PC อ่อนเมื่อแผ่นกันโดนดัน:</b> ผนังยึดแค่หูยึดผนัง 2 หู (y −102 และ 56) ช่วงห่าง 158 มม. ถ้าคู่แข่งดันแผ่นกันด้วยแรง ~8 N ผนังยุบเข้า ~%(defl).0f มม. (คำนวณแบบคานช่วงเดียว)
แผ่นกันจะยกจากพื้นได้ · W1 มีช่วงนี้ยาวเท่ากัน · การจำลอง MuJoCo ถือว่าแผ่นกันแข็งเกร็ง จึงยังไม่ได้ตอบเรื่องนี้ · ต้องออกแบบที่ยึดกลางผนังเพิ่ม (กินน้ำหนัก) ก่อนแข่ง</li></ul>
<h2 style="page-break-before:always">15. ภาพรหัสชิ้น</h2><img src="images/iso_ids.png" style="width:100%%"><img src="images/iso_ids_front.png" style="width:100%%">
<img src="images/in_robot.png" style="width:100%%">
<h2 style="page-break-before:always">16. แบบวางโครงเต็มสเกล 1:1 (มองจากบน หัวรถขึ้นบน)</h2>
<p>พิมพ์ 100%% วัดแถบ 100 มม. ก่อนใช้ · แดง = ท่อ · ส้ม = แผ่น · เส้นประ ' = ชิ้นที่อยู่สูง</p>%(layout)s
''' % dict(rev=REV, date=REV_DATE, total=mass['total_g'], margin=mass['margin_to_2000_g'], mcls='bad' if mass['margin_to_2000_g'] < 30 else '',
           groups=''.join('<tr><td>%s</td><td class="n">%.1f</td></tr>' % (e(GROUP_TH.get(k, k)), v) for k, v in g.items()),
           cgy=mass['cg_mm'][1], cga=mass['cg_mm'][1] - B.FRONT_Y, wl=rep['support_2wd']['drive_wheel_load_pct'], push=rep['support_2wd']['push_N_mu0.6'],
           tl=tube_total, nt=len(tubes), tubes=tube_rows, ns=len(sheets), sheets=sheet_rows, pad_lo=19.5 - B.RAIL_Z1, pad_hi=45.0 - 44.5,
           nw=len(rep['welds']), welds=weld_rows, drill=drill_rows,
           sw_up=rep['wedgelet_swing']['R']['up']['free_deg'], sw_dn=-rep['wedgelet_swing']['R']['down']['free_deg'],
           steel=mass['steel_g'], bead=B.WELD * mass['steel_g'], pc=g.get('PC skins', 0), sf=STRENGTH['summary']['min_design_sf'],
           strength=str_rows, sim=e(sim_txt) or 'ยังไม่ได้รัน', layout=plan_svg(),
           foot_id=ID['Weapon_Motor_Mount_Foot'], shelf_id=ID['Weapon_Motor_Mount_Shelf'],
           defl=next(p['load'] for p in STRENGTH['paths'] if p['path'].startswith('Side wall PC 2 mm, inward')))
CSS = '''@page{size:A4;margin:11mm}body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:9pt;color:#111;line-height:1.45}
.stamp{border:2px solid #b36b00;color:#b36b00;font-weight:700;padding:3px 10px;display:inline-block;font-size:11pt}
h1{font-size:15pt;margin:6px 0}h2{font-size:11pt;margin:12px 0 4px;border-bottom:1px solid #999;page-break-after:avoid}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #aaa;padding:2px 5px;text-align:left;vertical-align:top}th{background:#eee}
tr{page-break-inside:avoid}.m{font-family:Menlo,monospace;font-size:7.5pt}.n{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
.bad{color:#b00}img{page-break-inside:avoid}li{margin-bottom:2px}'''
pdf(body, CSS, PKG / 'BUILD_GUIDE_TH.pdf')
(PKG / 'manifest.json').write_text(json.dumps({
    'status': 'CONCEPT_CAD_DIY_DRAWINGS', 'revision': REV, 'date': REV_DATE, 'datum_mm': DATUM, 'ids': ID,
    'hold_points': ['ARC practice coupons pass before any frame weld', 'discs weighed (<= 0.5 g apart) and rotor statically balanced before first spin'],
    'stack_drill': rep['stack_drill'], 'owner_drills_after_weld': rep['drill_after_weld'],
    'flat_parts': [{k: f[k] for k in ('key', 'title', 'material', 't', 'qty', 'size', 'dxf', 'note')} for f in FLAT],
    'tube_cut_list_mm': rep['tube_cut_list_mm'], 'welds': rep['welds'], 'mass': mass}, ensure_ascii=False, indent=1) + '\n')
zpath = HERE / 'DIY_PACKAGE.zip'
with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
    for p in sorted(PKG.rglob('*')):
        if p.is_file():
            z.write(p, Path('DIY_PACKAGE') / p.relative_to(PKG))
print(json.dumps({'zip': str(zpath), 'tubes': len(tubes), 'sheets': len(sheets), 'flat_parts': len(FLAT), 'template_pages': len(pages),
                  'welds': len(rep['welds']), 'mass_g': mass['total_g']}, ensure_ascii=False))
