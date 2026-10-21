"""Lathe-shop package for the R4-steel drive wheel hub (6061) and the O-ring tyres it carries.

Reads output/build_report.json ['wheel_hub'] and output/WHEEL_HUB_6061.step (written by build_r4_steel.py, so run
that first) and writes LATHE_HUB/ + LATHE_HUB.zip:
  HUB_DRAWING_TH.pdf   half-section and end view with every turned dimension, the D-bore, set screw and holes,
                       tolerances, and the O-ring purchase spec
  WHEEL_HUB_6061.step  the hub, axis along +x, inner (motor-side) face at x 0
Qty 2 (left and right are the same part, the left one is fitted turned round).
Run from the repository root:
  .venv-v4-print/bin/python robot_v4/print_revision/r4_steel/make_hub_package.py
"""
import json
import math
import shutil
import subprocess
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['Sukhumvit Set', 'Tahoma']
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

HERE = Path(__file__).resolve().parent
OUT = HERE / 'output'
PKG = HERE / 'LATHE_HUB'
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
rep = json.loads((OUT / 'build_report.json').read_text())
H = rep['wheel_hub']
W = H['width']
lw, g = H['land_w'], H['groove_w']
grooves = [(lw[0], lw[0] + g), (lw[0] + g + lw[1], lw[0] + 2 * g + lw[1])]
xs = H['set_screw_from_inner_face']
if not grooves[0][0] < xs < grooves[0][1]:
    raise RuntimeError('set screw is not under the inner groove: the key access hole would miss the O-ring seat')

if PKG.exists():
    shutil.rmtree(PKG)
PKG.mkdir()
shutil.copy2(OUT / 'WHEEL_HUB_6061.step', PKG / 'WHEEL_HUB_6061.step')

# ---------------------------------------------------------------- drawing
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={'width_ratios': [1.05, 1]}, facecolor='white')
R = lambda d: d / 2
# half section (x along the axis, r up); hatched solid = metal
solid = '#b9c4d0'
ax.add_patch(Rectangle((0, R(4.10)), W, R(H['boss_d']) - R(4.10), facecolor=solid, edgecolor='k', lw=0.8))           # boss
ax.add_patch(Rectangle((W - H['web_t'], R(H['boss_d'])), H['web_t'], R(H['rim_id']) - R(H['boss_d']), facecolor=solid, edgecolor='k', lw=0.8))
ax.add_patch(Rectangle((0, R(H['rim_id'])), W, R(H['groove_root_d']) - R(H['rim_id']), facecolor=solid, edgecolor='k', lw=0.8))  # rim
x = 0.0
for w_, kind in ((lw[0], 'land'), (g, 'groove'), (lw[1], 'land'), (g, 'groove'), (lw[2], 'land')):
    if kind == 'land':
        ax.add_patch(Rectangle((x, R(H['groove_root_d'])), w_, R(H['land_d']) - R(H['groove_root_d']), facecolor=solid, edgecolor='k', lw=0.8))
    else:
        ax.add_patch(Circle((x + w_ / 2, R(H['groove_root_d']) + 2.82), 2.82, facecolor='#333', alpha=0.35, edgecolor='k', lw=0.6))
    x += w_
ax.plot([-3, W + 3], [0, 0], 'k-.', lw=0.7)
ax.plot([xs, xs], [R(4.10), R(H['land_d']) + 1], 'r--', lw=1)
ax.text(xs + 0.3, R(H['land_d']) + 1.3, 'M3 set screw\n+ D3.5 key hole', color='r', fontsize=8)


def dim_v(xp, r0, r1, label, side=1):
    ax.annotate('', (xp, r0), (xp, r1), arrowprops=dict(arrowstyle='<->', lw=0.7))
    ax.text(xp + 0.4 * side, (r0 + r1) / 2, label, fontsize=8, ha='left' if side > 0 else 'right', va='center')


def dim_h(x0, x1, rp, label):
    ax.annotate('', (x0, rp), (x1, rp), arrowprops=dict(arrowstyle='<->', lw=0.7))
    ax.text((x0 + x1) / 2, rp + 0.4, label, fontsize=8, ha='center')


dim_v(-1.5, 0, R(H['land_d']), 'Ø%g' % H['land_d'], -1)
dim_v(W + 1.0, 0, R(H['groove_root_d']), 'Ø%g' % H['groove_root_d'])
dim_v(W + 5.5, 0, R(H['rim_id']), 'Ø%g (ใน)' % H['rim_id'])
dim_v(W + 10.0, 0, R(H['boss_d']), 'Ø%g' % H['boss_d'])
dim_h(0, W, -2.5, '%g' % W)
dim_h(*grooves[0], R(H['land_d']) + 4.5, 'ร่อง %g' % g)
dim_h(*grooves[1], R(H['land_d']) + 4.5, 'ร่อง %g' % g)
dim_h(0, xs, -5.5, '%.1f' % xs)
dim_h(W - H['web_t'], W, R(H['boss_d']) + 3, '%g' % H['web_t'])
ax.text(0.2, R(4.10) + 0.5, 'รู D Ø4.10\nหน้าแบน 1.55 จากแกน', fontsize=7.5)
ax.text(-2.5, -8.5, 'ด้านมอเตอร์ (หน้าใน)', fontsize=8)
ax.set_xlim(-12, W + 22); ax.set_ylim(-10, R(H['land_d']) + 8); ax.set_aspect('equal'); ax.axis('off')
ax.set_title('ภาพตัดครึ่ง (มม.) — แกนอยู่ด้านล่าง', fontsize=11)
# end view from the outer (web) face
lim = R(H['land_d']) + 4
ax2.add_patch(Circle((0, 0), R(H['land_d']), facecolor=solid, edgecolor='k'))
ax2.add_patch(Circle((0, 0), R(H['rim_id']), facecolor='#dfe5ec', edgecolor='k'))
ax2.add_patch(Circle((0, 0), R(H['boss_d']), facecolor=solid, edgecolor='k'))
n, hr, pr = H['holes']
for k in range(n):
    a = math.radians(60 * (k + 1))
    ax2.add_patch(Circle((pr * math.cos(a), pr * math.sin(a)), hr, facecolor='white', edgecolor='k'))
th = [math.radians(t) for t in range(0, 361, 4)]
pts = [(min(R(4.10) * math.cos(t), 1.55), R(4.10) * math.sin(t)) for t in th]
ax2.fill([p[0] for p in pts], [p[1] for p in pts], color='white', ec='k', lw=0.8)
ax2.plot([1.55, R(H['land_d'])], [0, 0], 'r--', lw=1)
ax2.text(R(H['boss_d']) + 1, 1.0, 'สกรูตัวหนอน M3 (ด้านหน้าแบน)', color='r', fontsize=8)
ax2.text(-lim + 1, -lim + 1, 'รูลดน้ำหนัก %d × Ø%g บนวง Ø%g ที่ 60°,120°,180°,240°,300°\nนับจากแนวสกรูตัวหนอน (0°)' % (n, 2 * hr, 2 * pr), fontsize=8)
ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim); ax2.set_aspect('equal'); ax2.axis('off')
ax2.set_title('มองจากด้านนอก (ด้านแผ่นเว็บ)', fontsize=11)
fig.tight_layout(); fig.savefig(PKG / 'hub_drawing.png', dpi=170); plt.close(fig)

# ---------------------------------------------------------------- order sheet
body = '''<span class="stamp">แบบสำหรับขอราคา / ตรวจแบบ — ยังไม่อนุมัติผลิต</span>
<h1>ดุมล้อขับ อะลูมิเนียม 6061 — Robot V4 R4-steel (ร้านกลึง)</h1>
<p>จำนวน <b>2 ชิ้น</b> (ซ้ายและขวาเป็นชิ้นเดียวกัน ตอนประกอบกลับด้านหนึ่งชิ้น) · วัสดุ <b>6061-T6</b> · หน่วย มม. · ไฟล์ <span class="m">WHEEL_HUB_6061.step</span> เป็นต้นฉบับ
(แกนตามแนว +X หน้าในด้านมอเตอร์อยู่ที่ X 0)</p>
<img src="hub_drawing.png" style="width:100%%">
<h2>1. ขนาดที่กลึง</h2>
<table><tr><th>รายการ</th><th>ขนาด</th><th>พิกัด</th></tr>
<tr><td>ความกว้างรวม</td><td class="n">%(W)g</td><td>±0.1</td></tr>
<tr><td>Ø ขอบร่อง (land)</td><td class="n">%(land)g</td><td>±0.1</td></tr>
<tr><td><b>Ø ก้นร่องโอริง</b> (2 ร่อง)</td><td class="n"><b>%(root)g</b></td><td><b>±0.05</b> (กำหนดความตึงของโอริง)</td></tr>
<tr><td>ความกว้างร่อง</td><td class="n">%(g)g</td><td>+0.2 / 0</td></tr>
<tr><td>ขอบร่อง ใน / กลาง / นอก</td><td class="n">%(l0)g / %(l1)g / %(l2)g</td><td>±0.1</td></tr>
<tr><td>Ø ในของขอบ (rim)</td><td class="n">%(rim)g</td><td>±0.2</td></tr>
<tr><td>แผ่นเว็บด้านนอก หนา</td><td class="n">%(web)g</td><td>±0.1</td></tr>
<tr><td>Ø บอส</td><td class="n">%(boss)g</td><td>±0.1</td></tr>
<tr><td>รูลดน้ำหนักในเว็บ</td><td class="n">%(n)d × Ø%(hd)g บนวง Ø%(pd)g</td><td>±0.2</td></tr></table>
<h2>2. รูเพลาและสกรูตัวหนอน</h2><ul>
<li><b>รู D</b> สำหรับเพลามอเตอร์ Ø4 หน้าแบนลึก 0.5 (ORTECH JB361): Ø4.10 +0.02/0 หน้าแบนห่างแกน 1.55 +0.02/0 ทะลุตลอด
— ถ้าร้านทำรู D ไม่ได้ ให้คว้านรูกลม Ø4.00 H7 แล้วใช้สกรูตัวหนอนกดหน้าแบนอย่างเดียว (แจ้งผู้สั่งก่อน)</li>
<li><b>สกรูตัวหนอน M3</b> ตั้งฉากกับหน้าแบน ห่างหน้าใน <b>%(xs).1f</b>: เจาะ Ø2.5 ต๊าป M3 ผ่านบอส
และเจาะ <b>Ø3.5 ผ่านก้นร่องในให้ตรงแนวเดียวกัน</b> สำหรับสอดประแจหกเหลี่ยม (ใส่สกรูตัวหนอน M3×4 ปลายถ้วย 1 ตัว)</li>
<li>ลบคมทุกขอบ ขอบร่องโอริงลบมุม R0.3 ห้ามมีครีบ (ครีบบาดโอริง) · ไม่ต้องชุบ</li></ul>
<h2>3. ยางล้อ = โอริงสำเร็จ (ผู้สั่งซื้อเอง ร้านไม่ต้องจัดหา)</h2><ul>
<li><b>%(oring)s</b> จำนวน <b>4 วง</b> (ล้อละ 2) + สำรอง 2 · ยี่ห้อ NOK หรือเทียบเท่า หาได้ที่ร้านซีล/ร้านอะไหล่ไฮดรอลิก-นิวแมติก</li>
<li>ยืด %(stretch).1f%% บนก้นร่อง → Ø นอกของยางประมาณ <b>%(od).1f</b> (แบบรถใช้ Ø64)</li>
<li>มวลตามแบบ: ดุม %(hubg).1f g + โอริง 2 × %(og).1f g = <b>%(wheel).1f g ต่อล้อ</b></li></ul>
<h2>4. สิ่งที่ร้านต้องส่งกลับ</h2><ul><li>ขนาด Ø ก้นร่องที่วัดได้ทั้ง 2 ร่องของทั้ง 2 ชิ้น และผลลองสวมกับเพลา Ø4 (ถ้ามีมอเตอร์ให้ลองกับมอเตอร์จริง)</li>
<li>น้ำหนักจริงของแต่ละชิ้น (g) · ใบเสนอราคา แยกค่าวัสดุ ค่ากลึง ค่าทำรู D/ต๊าป และค่าส่ง</li></ul>
''' % dict(W=W, land=H['land_d'], root=H['groove_root_d'], g=g, l0=lw[0], l1=lw[1], l2=lw[2], rim=H['rim_id'], web=H['web_t'],
           boss=H['boss_d'], n=n, hd=2 * hr, pd=2 * pr, xs=xs, oring=H['oring'], stretch=H['oring_stretch_pct'], od=H['tyre_od'],
           hubg=H['hub_mass_g'], og=H['oring_mass_g'], wheel=H['hub_mass_g'] + 2 * H['oring_mass_g'])
CSS = '''@page{size:A4;margin:11mm}body{font-family:"Sukhumvit Set","Thonburi",sans-serif;font-size:9.5pt;color:#111;line-height:1.45}
.stamp{border:2px solid #b36b00;color:#b36b00;font-weight:700;padding:3px 10px;display:inline-block;font-size:11pt}
h1{font-size:14pt;margin:6px 0}h2{font-size:11pt;margin:10px 0 4px;border-bottom:1px solid #999}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #aaa;padding:2px 5px;text-align:left}th{background:#eee}
.m{font-family:Menlo,monospace;font-size:8pt}.n{text-align:right;white-space:nowrap}li{margin-bottom:2px}'''
tmp = PKG / 'HUB_DRAWING_TH.html'
tmp.write_text('<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (CSS, body))
subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(PKG / 'HUB_DRAWING_TH.pdf'), tmp.as_uri()],
               check=True, capture_output=True, timeout=180)
tmp.unlink()
with zipfile.ZipFile(HERE / 'LATHE_HUB.zip', 'w', zipfile.ZIP_DEFLATED) as z:
    for p in sorted(PKG.rglob('*')):
        z.write(p, Path('LATHE_HUB') / p.relative_to(PKG))
print(json.dumps({'zip': str(HERE / 'LATHE_HUB.zip'), 'wheel_g': round(H['hub_mass_g'] + 2 * H['oring_mass_g'], 2), 'tyre_od': H['tyre_od']}, ensure_ascii=False))
