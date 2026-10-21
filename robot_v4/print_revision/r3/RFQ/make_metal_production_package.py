"""Production package for the R3 laser-cut metal parts: dimensioned drawings + release gates.

Reads the current output_R3/DRAFT/metal files and inputs_R3.json/build_report.json, checks every release gate,
and writes PRODUCTION/V4_R3_METAL_<STATUS>/ with copied DXF/STEP (SHA-checked), one dimensioned drawing per part
(hole table, edge taps found in STEP, tolerances, title block) and a Thai production work order PDF.
STATUS is RELEASED only when no gate fails; otherwise every page is stamped BLOCKED and the blockers are listed.
If PRODUCTION/RELEASE_DECISION.json exists (a partial release the user recorded for this exact build), the parts it
releases go to V4_R3_METAL_RELEASED with its accepted risks listed, and every other part stays in V4_R3_METAL_BLOCKED.
Run from the repository: .venv-v4-print/bin/python robot_v4/print_revision/r3/RFQ/make_metal_production_package.py
"""
import html
import itertools
import json
import math
import os
import shutil
import subprocess
import zipfile
from pathlib import Path

import cadquery as cq
import ezdxf
import ezdxf.bbox

import hashlib

HERE = Path(__file__).resolve().parent
R3 = HERE.parent
SRC = R3 / 'output_R3' / 'DRAFT' / 'metal'
INPUTS = json.loads((R3 / 'inputs_R3.json').read_text())
REPORT = json.loads((R3 / 'output_R3' / 'build_report.json').read_text())
DATE = '2026-10-19'
CHROME = os.environ.get('CHROME', '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome')
DECISION_PATH = R3 / 'PRODUCTION' / 'RELEASE_DECISION.json'
DECISION = json.loads(DECISION_PATH.read_text()) if DECISION_PATH.exists() else None
if DECISION and not REPORT['build_sha256'].startswith(DECISION['basis']['r3_build_sha256_prefix']):
    raise SystemExit('RELEASE_DECISION.json was made for build %s, current build is %s: re-decide or delete it' % (
        DECISION['basis']['r3_build_sha256_prefix'], REPORT['build_sha256'][:8]))
# unmeasured hardware values the user accepted for the released parts: 'part.key' -> reason
ACCEPTED = DECISION['accepted_risks'] if DECISION else {}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

STEEL, AL = 7.85e-3, 2.70e-3
STANDARD_SHEETS = (0.8, 1.5, 1.6, 2.0, 3.0, 6.0)
MIN_HOLE_WALL = 1.5          # mm between any two holes
MIN_CSK_LAND = 0.5           # mm of straight bore left under a countersink
# 90 deg countersink diameter at the surface, by clearance hole (DIN 7991 heads + 0.3 mm)
CSK_FOR_HOLE = {3.4: 6.3, 2.9: 5.3}
TAP_FOR_DRILL = {2.5: 'M3', 2.05: 'M2.5'}

PARTS = [
    dict(id='M01', file='Beater_Disc_L', name='จานอาวุธ (beater disc)', qty=2, density=STEEL,
         material='เหล็ก 4140 ชุบแข็ง 45±2 HRC หลังเจาะ หรือ Hardox 450 (ความแข็งใกล้เคียงกัน)', csk_holes=(3.4,),
         finish=['เจาะและ countersink ให้เสร็จก่อนชุบแข็ง (ถ้าใช้ 4140)', 'countersink ด้านเดียว: ด้านนอกของแต่ละจาน สองชิ้นเหมือนกัน',
                 'ลบคมทุกขอบ หลังชุบแข็งห้ามเจียรรูกลาง', 'ถ่วงสมดุลร่วมกับดุมหลังประกอบ (ผู้ใช้ทำ)']),
    dict(id='M02', file='Upright_6061_6mm_L', name='แผ่นตั้งซ้าย (upright L)', qty=1, density=AL, material='อลูมิเนียม 6061-T6',
         finish=['ส่วนเว้าด้านหลังหลบมอเตอร์มีเฉพาะชิ้นซ้าย ห้ามใช้ไฟล์ขวาแทน', 'ลบคมทุกขอบ']),
    dict(id='M03', file='Upright_6061_6mm_R', name='แผ่นตั้งขวา (upright R)', qty=1, density=AL, material='อลูมิเนียม 6061-T6',
         finish=['ลบคมทุกขอบ']),
    dict(id='M04', file='Weapon_Top_Brace_3mm', name='คานยึดบน (top brace)', qty=1, density=AL, material='อลูมิเนียม 6061-T6',
         finish=['ร่องกลางหลบสายพาน ห้ามปิด', 'ลบคมทุกขอบ']),
    dict(id='M05', file='Weapon_Bottom_Brace_3mm', name='คานยึดล่าง (bottom brace)', qty=1, density=AL, material='อลูมิเนียม 6061-T6',
         finish=['ลบคมทุกขอบ']),
    dict(id='M06', file='Weapon_Motor_Mount_3mm', name='แผ่นยึดมอเตอร์อาวุธ', qty=1, density=AL, material='อลูมิเนียม 6061-T6',
         finish=['ลบคมทุกขอบ']),
    dict(id='M07', file='Wedgelet_L', name='แผ่นลิ่ม (wedgelet) ซ้าย 1 + ขวา 1', qty=2, density=STEEL, sloped=True,
         material='เหล็กสปริง 65Mn หรือ SK5 (ชุบแข็งแล้ว)', csk_holes=(3.4,),
         csk_override=INPUTS['design'].get('wedgelet', {}).get('countersink_diameter_mm'),
         finish=['สกรู ' + INPUTS['design'].get('wedgelet', {}).get('fastener', 'M3 หัวจม') + ' — วัดหัวสกรูจริงก่อนทำ countersink',
                 'ชิ้นซ้าย countersink ด้าน A ชิ้นขวา countersink ด้านตรงข้าม (ชิ้นสะท้อนกัน)',
                 'เจียรขอบหน้าเอียงตามหน้าตัดใน STEP ให้ขอบวางแนบพื้น', 'ลบคมทุกขอบยกเว้นขอบหน้า']),
    dict(id='M08', file='Skirt_Plate_08mm_R', name='แผ่นกันข้าง (side skirt) ซ้าย + ขวา', qty=2, density=AL, sloped=True,
         material='อลูมิเนียม 5052-H32', finish=['สองชิ้นเหมือนกัน', 'ลบคมทุกขอบ']),
    dict(id='M09', file='Skirt_Plate_08mm_Rear', name='แผ่นกันท้าย (rear skirt)', qty=1, density=AL, sloped=True,
         material='อลูมิเนียม 5052-H32', finish=['ลบคมทุกขอบ']),
]
# Which measured hardware values each part's geometry depends on (release gate).
DEPENDS = {
    'Weapon_Motor_Mount_3mm': [('weapon_motor', 'mount_holes_yz_mm'), ('weapon_motor', 'mount_thread_diameter_mm'),
                               ('weapon_motor', 'boss_diameter_mm'), ('drive_Front_L', 'diameter_mm'), ('drive_Front_R', 'diameter_mm')],
    'Upright_6061_6mm_L': [('weapon_motor', 'diameter_mm'), ('dead_shaft', 'diameter_mm')],
    'Upright_6061_6mm_R': [('dead_shaft', 'diameter_mm')],
}


def hardware_actual(part, key):
    return INPUTS['hardware'][part]['actual'].get(key)


def plate_frame(shape):
    face = max((f for f in shape.Faces() if f.geomType() == 'PLANE'), key=lambda f: (len(f.Wires()), f.Area()))
    return face, cq.Plane(origin=face.Center(), normal=face.normalAt())


def plate_thickness(shape, face):
    """Distance to the largest planar face opposite the drawing face (immune to countersinks and blind holes)."""
    n, c = face.normalAt(), face.Center()
    opposite = [f for f in shape.Faces() if f.geomType() == 'PLANE' and f.normalAt().dot(n) < -0.999]
    back = max(opposite, key=lambda f: f.Area())
    return abs((back.Center() - c).dot(n))


def holes_from_step(shape, plane, thickness):
    """Through holes (axis along the plate normal) and edge holes (axis in the plate plane)."""
    n = plane.zDir
    through, edge = [], []
    for f in shape.Faces():
        if f.geomType() != 'CYLINDER':
            continue
        cyl = f._geomAdaptor().Cylinder()
        axis = cyl.Axis().Direction()
        d = round(2 * cyl.Radius(), 3)
        centre = plane.toLocalCoords(f.Center())
        if abs(axis.X() * n.x + axis.Y() * n.y + axis.Z() * n.z) > 0.9:
            through.append((d, f))
        else:
            bb = f.BoundingBox()
            depth = max(bb.xlen, bb.ylen, bb.zlen) if d < thickness else 0
            ax = (axis.X() * plane.xDir.x + axis.Y() * plane.xDir.y + axis.Z() * plane.xDir.z,
                  axis.X() * plane.yDir.x + axis.Y() * plane.yDir.y + axis.Z() * plane.yDir.z)
            edge.append(dict(d=d, x=centre.x, y=centre.y, depth=round(depth, 2), face=f, along_x=abs(ax[0]) > abs(ax[1])))
    return through, edge


def min_hole_wall(faces):
    best = (math.inf, None)
    for (a, fa), (b, fb) in itertools.combinations(faces, 2):
        gap = fa.distance(fb)
        if gap < best[0]:
            best = (gap, (a, b))
    return best


def dxf_geometry(path):
    msp = ezdxf.readfile(path).modelspace()
    box = ezdxf.bbox.extents(msp)
    x0, y0 = box.extmin.x, box.extmin.y
    lines, arcs, circles = [], [], []
    for e in msp:
        kind = e.dxftype()
        if kind == 'LINE':
            lines.append((e.dxf.start.x - x0, e.dxf.start.y - y0, e.dxf.end.x - x0, e.dxf.end.y - y0))
        elif kind == 'ARC':
            arcs.append((e.dxf.center.x - x0, e.dxf.center.y - y0, e.dxf.radius, e.dxf.start_angle, e.dxf.end_angle))
        elif kind == 'CIRCLE':
            circles.append((e.dxf.center.x - x0, e.dxf.center.y - y0, 2 * e.dxf.radius))
    return dict(w=box.size.x, h=box.size.y, x0=x0, y0=y0, lines=lines, arcs=arcs, circles=circles)


def hole_operation(d, thickness, part):
    d = round(d, 2)
    if d in TAP_FOR_DRILL:
        return 'มาร์กด้วยเลเซอร์ → เจาะ Ø%g → ต๊าป %s-6H ทะลุ' % (d, TAP_FOR_DRILL[d])
    op = 'ตัดเลเซอร์' if d >= thickness else 'มาร์กด้วยเลเซอร์ → เจาะ Ø%g' % d
    if d in part.get('csk_holes', ()) and d in CSK_FOR_HOLE:
        op += ' → countersink 90° Ø%g' % csk_diameter(part, d)
    return op


def csk_diameter(part, d):
    return part.get('csk_override') or CSK_FOR_HOLE[d]


def svg_drawing(g, edge_holes):
    pad, dim = 14.0, 9.0
    W, H = g['w'] + 2 * pad + dim, g['h'] + 2 * pad + dim
    tx = lambda x: pad + dim + x
    ty = lambda y: pad + g['h'] - y
    s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.2f %.2f" class="part">' % (W, H),
         '<g fill="none" stroke="#000" stroke-width="%.2f">' % max(0.25, g['w'] / 400)]
    for x1, y1, x2, y2 in g['lines']:
        s.append('<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f"/>' % (tx(x1), ty(y1), tx(x2), ty(y2)))
    for cx, cy, r, a0, a1 in g['arcs']:
        p0 = (tx(cx + r * math.cos(math.radians(a0))), ty(cy + r * math.sin(math.radians(a0))))
        p1 = (tx(cx + r * math.cos(math.radians(a1))), ty(cy + r * math.sin(math.radians(a1))))
        large = 1 if (a1 - a0) % 360 > 180 else 0
        s.append('<path d="M%.3f %.3f A%.3f %.3f 0 %d 0 %.3f %.3f"/>' % (p0[0], p0[1], r, r, large, p1[0], p1[1]))
    for cx, cy, d in g['circles']:
        s.append('<circle cx="%.3f" cy="%.3f" r="%.3f"/>' % (tx(cx), ty(cy), d / 2))
    s.append('</g>')
    fs = max(2.2, min(g['w'], g['h']) / 14)
    label = '<text x="%.2f" y="%.2f" font-size="%.2f" fill="#c00" font-family="Menlo,monospace">%s</text>'
    for i, (cx, cy, d) in enumerate(g['circles'], 1):
        s.append(label % (tx(cx) + d / 2 + 0.6, ty(cy) - d / 2 - 0.4, fs, 'H%d' % i))
    for j, e in enumerate(edge_holes, 1):
        x, y = tx(e['x'] - g['x0']), ty(e['y'] - g['y0'])
        s.append('<circle cx="%.2f" cy="%.2f" r="1.1" fill="#06c"/>' % (x, y))
        s.append(label.replace('#c00', '#06c') % (x + 1.5, y + (fs if y < H / 2 else -1), fs, 'E%d' % j))
    # overall dimensions and datum
    y_dim, x_dim = pad + g['h'] + dim * 0.75, pad + dim * 0.25
    s.append('<g stroke="#555" stroke-width="0.2" fill="#555" font-size="%.2f" font-family="Menlo,monospace">' % fs)
    s.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (tx(0), y_dim, tx(g['w']), y_dim))
    s.append('<text x="%.2f" y="%.2f" text-anchor="middle" stroke="none">%.1f</text>' % (tx(g['w'] / 2), y_dim + fs, g['w']))
    s.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (x_dim, ty(0), x_dim, ty(g['h'])))
    s.append('<text x="%.2f" y="%.2f" text-anchor="middle" stroke="none" transform="rotate(-90 %.2f %.2f)">%.1f</text>' % (
        x_dim - 1, ty(g['h'] / 2), x_dim - 1, ty(g['h'] / 2), g['h']))
    s.append('<circle cx="%.2f" cy="%.2f" r="1" fill="#000"/><text x="%.2f" y="%.2f" stroke="none" fill="#000">0,0</text>' % (
        tx(0), ty(0), tx(0) + 1.4, ty(0) - 1.2))
    s.append('</g></svg>')
    return '\n'.join(s)


def analyse(part, blockers, accepted):
    dxf, step = SRC / (part['file'] + '.dxf'), SRC / (part['file'] + '.step')
    shape = cq.importers.importStep(str(step)).val()
    face, plane = plate_frame(shape)
    thickness = plate_thickness(shape, face)
    std = min(STANDARD_SHEETS, key=lambda t: abs(t - thickness))
    if abs(std - thickness) > 0.03:
        blockers.append('%s %s: ความหนาใน CAD %.3f มม. ไม่ใช่ความหนาแผ่นมาตรฐาน (แผ่นเอียงต้องวัดตั้งฉากผิว)' % (part['id'], part['file'], thickness))
    through, edge = holes_from_step(shape, plane, thickness)
    g = dxf_geometry(dxf)
    rows = []
    for i, (cx, cy, d) in enumerate(g['circles'], 1):
        rows.append(dict(id='H%d' % i, x=cx, y=cy, d=d, op=hole_operation(d, std, part)))
        if d in part.get('csk_holes', ()) and d in CSK_FOR_HOLE:
            land = std - (csk_diameter(part, d) - d) / 2
            if land < MIN_CSK_LAND:
                blockers.append('%s %s: countersink Ø%g บนแผ่น %g มม. เหลือเนื้อตรง %.2f มม. (ต้อง ≥ %g)' % (
                    part['id'], part['file'], csk_diameter(part, d), std, land, MIN_CSK_LAND))
    edge_rows = []
    for j, e in enumerate(edge, 1):
        tap = TAP_FOR_DRILL.get(round(e['d'], 2))
        spec = INPUTS['design'].get('upright_edge_tap') if part['file'].startswith('Upright') else None
        if spec:
            op = 'เจาะ Ø%g ลึก %g (ปลายดอก %d°) → ต๊าป %s-6H ลึก %g' % (spec['drill_diameter_mm'], spec['drill_depth_mm'],
                                                                     spec['drill_point_deg'], spec['thread'], spec['thread_depth_mm'])
        else:
            op = ('เจาะ Ø%g ลึก %.1f → ต๊าป %s-6H' % (e['d'], e['depth'], tap)) if tap else 'รูขอบ Ø%g ลึก %.1f' % (e['d'], e['depth'])
        x, y = e['x'] - g['x0'], e['y'] - g['y0']
        # report the entry point on the edge, not the middle of the blind hole
        if e['along_x']:
            x = 0.0 if x < g['w'] / 2 else g['w']
        else:
            y = 0.0 if y < g['h'] / 2 else g['h']
        edge_rows.append(dict(id='E%d' % j, x=x, y=y, d=e['d'], depth=e['depth'], op=op))
    if part['file'].startswith('Upright') and len(edge) < 2:
        blockers.append('%s %s: ยังไม่มีรูต๊าป M3 ที่ขอบบน/ล่างใน CAD (คานบน/ล่างยึดไม่ได้)' % (part['id'], part['file']))
    faces = [(d, f) for d, f in through] + [(e['d'], e['face']) for e in edge]
    wall, pair = min_hole_wall(faces)
    if faces and len(faces) > 1 and wall < MIN_HOLE_WALL:
        blockers.append('%s %s: ผนังระหว่างรู Ø%g กับ Ø%g บางสุด %.2f มม. (ต้อง ≥ %g)' % (part['id'], part['file'], pair[0], pair[1], wall, MIN_HOLE_WALL))
    for hw, key in DEPENDS.get(part['file'], []):
        if hardware_actual(hw, key) is None and '%s.%s' % (hw, key) in ACCEPTED:
            accepted.append('%s %s: %s.%s — %s' % (part['id'], part['file'], hw, key, ACCEPTED['%s.%s' % (hw, key)]))
        elif hardware_actual(hw, key) is None:
            blockers.append('%s %s: ใช้ค่า %s.%s ที่ยังไม่ได้วัดจริง' % (part['id'], part['file'], hw, key))
    mass = face.Area() * std * part['density']
    return dict(part, thickness=std, cad_thickness=round(thickness, 3), rows=rows, edge_rows=edge_rows, g=g,
                svg=svg_drawing(g, edge), mass=mass, min_wall=None if len(faces) < 2 else round(wall, 2))


def main():
    per_part, accepted, parts = {}, [], []
    for p in PARTS:
        per_part[p['id']] = []
        parts.append(analyse(p, per_part[p['id']], accepted))
    mass = REPORT['mass']
    common = []
    if mass['total_g'] > INPUTS['design']['mass_limit_g']:
        common.append('build ล่าสุดหนัก %.2f g เกิน %d g' % (mass['total_g'], INPUTS['design']['mass_limit_g']))
    if mass['rear_static_load_fraction'] < INPUTS['design']['rear_load_min']:
        common.append('build ล่าสุดล้อหลังรับ %.2f%% ต่ำกว่า %g%%' % (100 * mass['rear_static_load_fraction'], 100 * INPUTS['design']['rear_load_min']))
    for old in (R3 / 'PRODUCTION').glob('V4_R3_METAL_*'):
        shutil.rmtree(old) if old.is_dir() else old.unlink()
    released = set(DECISION['released']['laser_metal']) if DECISION else set()
    unknown = released - set(per_part)
    if unknown:
        raise SystemExit('RELEASE_DECISION.json releases unknown parts: %s' % sorted(unknown))
    still = {i: per_part[i] for i in released if per_part[i]}
    if released and (common or still):
        raise SystemExit('RELEASE_DECISION.json releases parts that still fail a gate: %s' % (common + sum(still.values(), [])))
    results = []
    if released:
        results.append(package('RELEASED', [p for p in parts if p['id'] in released], [],
                               [a for a in accepted if a.split()[0] in released], 'WORK_ORDER_PRODUCTION_TH.pdf'))
    held = [p for p in parts if p['id'] not in released]
    if held:
        blockers = list(dict.fromkeys(sum((per_part[p['id']] for p in held), []) + common))
        if not blockers and DECISION:
            blockers = ['%s: ผู้สั่งยังไม่อนุมัติใน RELEASE_DECISION.json' % p['id'] for p in held]
        status = 'BLOCKED' if blockers else 'RELEASED'
        results.append(package(status, held, blockers, [a for a in accepted if a.split()[0] not in released],
                               'WORK_ORDER_HOLD_TH.pdf' if released else 'WORK_ORDER_PRODUCTION_TH.pdf'))
    print(json.dumps(results, ensure_ascii=False, indent=1))


def package(status, parts, blockers, notes, pdf_name):
    stamp = 'ใช้ขอราคาได้ — ห้ามผลิตจนกว่าเงื่อนไขหน้าแรกจะผ่าน (BLOCKED)' if blockers else 'อนุมัติผลิต (RELEASED)'
    if status == 'RELEASED' and DECISION:
        stamp = 'อนุมัติผลิต (RELEASED) %s — เฉพาะชิ้นในใบนี้' % DECISION['date']
    mass = REPORT['mass']
    out = R3 / 'PRODUCTION' / ('V4_R3_METAL_' + status)
    (out / 'DXF').mkdir(parents=True)
    (out / 'STEP').mkdir()
    files = []
    for p in parts:
        for ext, sub in (('.dxf', 'DXF'), ('.step', 'STEP')):
            src, dst = SRC / (p['file'] + ext), out / sub / (p['file'] + ext)
            shutil.copy2(src, dst)
            if sha(src) != sha(dst):
                raise RuntimeError('copy changed ' + src.name)
            files.append({'file': '%s/%s' % (sub, dst.name), 'sha256': sha(dst)})
    for name in ('DERIVED_Wedgelet_R_mirror_of_Wedgelet_L', 'DERIVED_Skirt_Plate_08mm_L_mirror_of_Skirt_Plate_08mm_R'):
        src_name, axis_part = name.split('_mirror_of_')[1], name[len('DERIVED_'):].split('_mirror_of_')[0]
        if src_name not in {p['file'] for p in parts}:
            continue
        shape = cq.importers.importStep(str(SRC / (src_name + '.step'))).val().mirror('YZ', (0, 0, 0))
        ref = next(r for r in mass['parts'] if r['part'] == axis_part)
        if max(abs(a - b) for a, b in zip(shape.Center().toTuple(), ref['centroid_mm'])) > 0.01:
            raise RuntimeError(name + ' does not match the build')
        shape.exportStep(str(out / 'STEP' / (name + '.step')))
        files.append({'file': 'STEP/%s.step' % name, 'sha256': sha(out / 'STEP' / (name + '.step')), 'derived': True})
    write_pdf(out, parts, blockers, status, stamp, files, notes, pdf_name)
    manifest = {'status': status, 'revision': REPORT['revision'], 'build_sha256': REPORT['build_sha256'], 'date': DATE,
                'blockers': blockers, 'accepted_risks': notes, 'mass_total_g': mass['total_g'], 'rear_load': mass['rear_static_load_fraction'],
                'parts': [{k: p[k] for k in ('id', 'file', 'name', 'material', 'qty', 'thickness', 'cad_thickness', 'min_wall')} |
                          {'mass_g': round(p['mass'], 2), 'holes': [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()} for r in p['rows']],
                           'edge_holes': [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()} for r in p['edge_rows']]} for p in parts],
                'files': files}
    if DECISION:
        manifest['release_decision'] = {'file': str(DECISION_PATH.relative_to(R3)), 'sha256': sha(DECISION_PATH)}
    (out / 'production_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    (out / 'SHA256SUMS.txt').write_text(''.join('%s  %s\n' % (f['sha256'], f['file']) for f in files))
    zip_path = out.with_suffix('.zip')
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as z:
        for path in sorted(out.rglob('*')):
            if path.is_file():
                z.write(path, Path(out.name) / path.relative_to(out))
    return {'status': status, 'out': str(out.relative_to(R3)), 'parts': [p['id'] for p in parts], 'blockers': blockers, 'accepted_risks': notes}


def write_pdf(out, parts, blockers, status, stamp, files, notes=None, pdf_name='WORK_ORDER_PRODUCTION_TH.pdf'):
    e = html.escape
    build_short = REPORT['build_sha256'][:12]
    color = '#b00' if blockers else '#070'
    bom = ''.join('<tr><td>%s</td><td class="m">%s.dxf</td><td>%s</td><td>%s</td><td class="n">%g</td><td class="n">%d</td><td class="n">%.1f</td></tr>' % (
        p['id'], e(p['file']), e(p['name']), e(p['material']), p['thickness'], p['qty'], p['mass']) for p in parts)
    block = ''.join('<li>%s</li>' % e(b) for b in blockers) or '<li>ไม่มี — ผ่านทุกเงื่อนไข</li>'
    block += ''.join('<li>ผู้สั่งยอมรับ: %s</li>' % e(n) for n in (notes or []))
    pages = []
    for p in parts:
        holes = ''.join('<tr><td>%s</td><td class="n">%.2f</td><td class="n">%.2f</td><td class="n">Ø%g</td><td>%s</td></tr>' % (
            r['id'], r['x'], r['y'], r['d'], e(r['op'])) for r in p['rows'])
        holes += ''.join('<tr class="edge"><td>%s</td><td class="n">%.2f</td><td class="n">%.2f</td><td class="n">Ø%g</td><td>%s (รูที่ขอบ ตำแหน่งตามแนวขอบ)</td></tr>' % (
            r['id'], r['x'], r['y'], r['d'], e(r['op'])) for r in p['edge_rows'])
        fin = ''.join('<li>%s</li>' % e(f) for f in p['finish'])
        pages.append('''<section class="sheet"><div class="stamp" style="color:%(c)s;border-color:%(c)s">%(stamp)s</div>
<div class="draw">%(svg)s</div>
<div class="side"><table class="holes"><tr><th>รู</th><th>X</th><th>Y</th><th>ขนาด</th><th>งาน</th></tr>%(holes)s</table>
<p class="small">พิกัดวัดจากจุด 0,0 (มุมล่างซ้ายของกรอบชิ้นงาน) หน่วย มม. รู H อยู่ใน DXF · รู E อยู่ที่ขอบ มีใน STEP</p>
<ul class="fin">%(fin)s</ul>
<p class="small">พิกัดทั่วไป ISO 2768-m · ขอบตัดเลเซอร์ ±0.1 · ตำแหน่งรู ±0.1 · รู Ø8.2 +0.1/0 · เกลียว 6H · ห้ามวัดจากภาพ ใช้ DXF/STEP</p></div>
<table class="tb"><tr><th>ID</th><td>%(id)s</td><th>ชิ้นงาน</th><td>%(name)s</td><th>จำนวน</th><td>%(qty)d</td></tr>
<tr><th>วัสดุ</th><td colspan="3">%(mat)s</td><th>หนา</th><td>%(t)g มม.</td></tr>
<tr><th>ไฟล์</th><td class="m" colspan="3">%(file)s.dxf / .step</td><th>มวล</th><td>%(mass).1f g</td></tr>
<tr><th>แบบ</th><td colspan="3">Robot V4 R3 · build %(b)s · %(date)s</td><th>สถานะ</th><td style="color:%(c)s">%(status)s</td></tr></table></section>''' % dict(
            c=color, stamp=e(stamp), svg=p['svg'], holes=holes, fin=fin, id=p['id'], name=e(p['name']), qty=p['qty'], mat=e(p['material']),
            t=p['thickness'], file=e(p['file']), mass=p['mass'], b=build_short, date=DATE, status=status))
    doc = '''<!doctype html><html lang="th"><head><meta charset="utf-8"><title>V4 R3 Metal Production</title><style>
@page{size:A4 landscape;margin:9mm}
body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:9pt;color:#111;margin:0}
.m{font-family:Menlo,monospace;font-size:8pt}.n{text-align:right;font-variant-numeric:tabular-nums}
table{border-collapse:collapse}td,th{border:1px solid #999;padding:2px 5px;text-align:left;vertical-align:top}th{background:#eee}
.stamp{border:2px solid;font-weight:700;padding:3px 8px;display:inline-block;font-size:11pt}
.cover h1{font-size:16pt;margin:6px 0}.cover table{width:100%%}.cover li{margin-bottom:3px}
.sheet{page-break-before:always;height:188mm;display:grid;grid-template-columns:1.25fr 1fr;grid-template-rows:auto 1fr auto;gap:4px 10px}
.sheet .stamp{grid-column:1/3;justify-self:start}
.draw{grid-row:2;display:flex;align-items:center;justify-content:center;border:1px solid #ccc;min-height:0}
.draw svg{max-width:100%%;max-height:118mm;width:100%%}
.side{grid-row:2;min-height:0;overflow:hidden}.holes{width:100%%;font-size:8pt}.holes tr.edge td{color:#06c}
.small{font-size:7.5pt;color:#444;margin:3px 0}.fin{margin:4px 0 0 16px;padding:0}
.tb{grid-column:1/3;width:100%%}
</style></head><body>
<section class="cover"><div class="stamp" style="color:%(c)s;border-color:%(c)s">%(stamp)s</div>
<h1>ใบสั่งผลิตชิ้นโลหะตัดเลเซอร์ — Robot V4 R3</h1>
<p>Build SHA256 <span class="m">%(build)s</span> · %(date)s · รวม %(pieces)d ชิ้น จาก %(n)d ไฟล์ DXF · หน่วย มม.</p>
<h2>เงื่อนไขก่อนผลิต</h2><ol>%(block)s</ol>
<table><tr><th>ID</th><th>ไฟล์</th><th>ชิ้นงาน</th><th>วัสดุ</th><th>หนา มม.</th><th>จำนวน</th><th>มวล/ชิ้น g</th></tr>%(bom)s</table>
<p class="small">ไม่รวม: ชิ้นกลึง (ดุม รอก spacer) ส่งร้านกลึงตาม MACHINING.md · สกรู insert แกนบานพับ ผู้ใช้ซื้อเอง · หน้าถัดไปเป็นแบบของแต่ละชิ้น</p></section>
%(pages)s</body></html>''' % dict(c=color, stamp=e(stamp), build=REPORT['build_sha256'], date=DATE, pieces=sum(p['qty'] for p in parts), n=len(parts),
                                      block=block, bom=bom, pages=''.join(pages))
    html_path = out / (pdf_name[:-4] + '.html')
    html_path.write_text(doc)
    pdf = out / pdf_name
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(pdf), html_path.as_uri()],
                   check=True, capture_output=True, timeout=120)
    html_path.unlink()
    files.append({'file': pdf.name, 'sha256': sha(pdf)})


if __name__ == '__main__':
    main()
