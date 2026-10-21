"""Simple dimensioned sketch of the R4-steel frame for the welder (WELD_SKETCH_TH.pdf, 6 A4 landscape pages).

The full drawing (WELD_DRAWING_TH.pdf) lists every part by coordinate; the shop found it hard to read. This sketch
shows the same CAD as plain views with chained dimensions from the rear-left corner (DATUM):
  1. bottom frame from above + tube cut list
  2. rear view (posts, top cross) + section at the drive-motor cross (motor plates, clamp post, clamp bar)
  3. front tabs (weapon-upright plates, hinge tabs) + the small tabs on the rails
  4-5. shaded 3D views from the four top corners, then from low and from underneath (labelled F/T ids)
  6. the frame inside the assembled robot (rest of the robot grey, one view with see-through PC skins)
Every number comes from build_r4_steel.py (bounding boxes of the as-welded parts), not typed by hand.
Called by make_weld_package.py.
"""
import html
import subprocess

e = html.escape
INK, TUBE_FILL, TAB_FILL, DIM, NOTE = '#111', '#f1c9c1', '#f9dcb8', '#0a4fa3', '#b00000'


def fmt(v):
    return ('%.1f' % v).rstrip('0').rstrip('.')


class Page:
    """One SVG page in mm. Plan coordinates (u, v) map to the page by p(u, v) = (ox + k u, oy - k v)."""

    def __init__(self, title, w=277.0, h=186.0):
        self.w, self.h, self.out = w, h, []
        self.title = title

    def view(self, ox, oy, k):
        self.ox, self.oy, self.k = ox, oy, k

    def p(self, u, v):
        return self.ox + self.k * u, self.oy - self.k * v

    def rect(self, u0, v0, u1, v1, fill, dash=False, label=None, size=3.2, lw=0.35):
        (x0, y1), (x1, y0) = self.p(u0, v0), self.p(u1, v1)
        self.out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" stroke="%s" stroke-width="%.2f"%s/>' % (
            x0, y0, x1 - x0, y1 - y0, fill, INK, lw, ' stroke-dasharray="1.6 1.2"' if dash else ''))
        if label:
            self.text((x0 + x1) / 2, (y0 + y1) / 2 + size * 0.35, label, size, bold=True)

    def circle(self, u, v, r, dash=False):
        x, y = self.p(u, v)
        self.out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="#fff" stroke="%s" stroke-width="0.35"%s/>' % (
            x, y, r * self.k, INK, ' stroke-dasharray="1.2 1"' if dash else ''))

    def line(self, x0, y0, x1, y1, color=INK, lw=0.25, dash=False):
        self.out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="%.2f"%s/>' % (
            x0, y0, x1, y1, color, lw, ' stroke-dasharray="1.2 1"' if dash else ''))

    def text(self, x, y, s, size=3.2, color=INK, bold=False, anchor='middle', rotate=None):
        tr = ' transform="rotate(%d %.2f %.2f)"' % (rotate, x, y) if rotate else ''
        self.out.append('<text x="%.2f" y="%.2f" font-size="%.2f" fill="%s" text-anchor="%s"%s%s>%s</text>' % (
            x, y, size, color, anchor, ' font-weight="700"' if bold else '', tr, e(s)))

    def utext(self, u, v, s, **kw):
        x, y = self.p(u, v)
        self.text(x, y, s, **kw)

    def arrow(self, x, y, dx, dy):
        n = (dx * dx + dy * dy) ** 0.5
        dx, dy = dx / n, dy / n
        a, b = 1.6, 0.6
        self.out.append('<path d="M%.2f %.2f L%.2f %.2f L%.2f %.2f Z" fill="%s"/>' % (
            x, y, x - a * dx - b * dy, y - a * dy + b * dx, x - a * dx + b * dy, y - a * dy - b * dx, DIM))

    def chain_x(self, us, v_from, v_dim, size=3.0, total=True):
        """Horizontal dimension chain through plan u values, extension lines from v_from (single value or per-u list)."""
        vf = v_from if isinstance(v_from, (list, tuple)) else [v_from] * len(us)
        _, yd = self.p(0, v_dim)
        for u, v0 in zip(us, vf):
            x, y0 = self.p(u, v0)
            self.line(x, y0 + (1 if yd > y0 else -1), x, yd + (1.5 if yd > y0 else -1.5), DIM, 0.2)
        for a, b in zip(us, us[1:]):
            xa, xb = self.p(a, 0)[0], self.p(b, 0)[0]
            self.line(xa, yd, xb, yd, DIM, 0.25)
            self.arrow(xa, yd, xa - xb, 0); self.arrow(xb, yd, xb - xa, 0)
            self.text((xa + xb) / 2, yd - 1.0, fmt(b - a), size, DIM, bold=True)
        if total and len(us) > 2:
            self.chain_x([us[0], us[-1]], [vf[0], vf[-1]], v_dim + (-1 if v_dim < 0 or yd > self.p(0, vf[0])[1] else 1) * 9 / self.k, size, False)

    def chain_v(self, vs, u_from, u_dim, size=3.0, total=True):
        """Vertical dimension chain through plan v values."""
        uf = u_from if isinstance(u_from, (list, tuple)) else [u_from] * len(vs)
        xd, _ = self.p(u_dim, 0)
        for v, u0 in zip(vs, uf):
            x0, y = self.p(u0, v)
            self.line(x0 + (1 if xd > x0 else -1), y, xd + (1.5 if xd > x0 else -1.5), y, DIM, 0.2)
        for a, b in zip(vs, vs[1:]):
            ya, yb = self.p(0, a)[1], self.p(0, b)[1]
            self.line(xd, ya, xd, yb, DIM, 0.25)
            self.arrow(xd, ya, 0, ya - yb); self.arrow(xd, yb, 0, yb - ya)
            self.text(xd - 1.0, (ya + yb) / 2, fmt(b - a), size, DIM, bold=True, rotate=-90)
        if total and len(vs) > 2:
            left = xd < self.p(uf[0], 0)[0]
            self.chain_v([vs[0], vs[-1]], [uf[0], uf[-1]], u_dim + (-1 if left else 1) * 9 / self.k, size, False)

    def image(self, x, y, w, h, href):
        self.out.append('<image x="%.2f" y="%.2f" width="%.2f" height="%.2f" href="%s" preserveAspectRatio="xMidYMid meet"/>' % (
            x, y, w, h, e(href)))
        self.out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="#bbb" stroke-width="0.25"/>' % (x, y, w, h))

    def svg(self):
        head = '<text x="0" y="7" font-size="6" font-weight="700" fill="%s">%s</text>' % (INK, e(self.title))
        return ('<svg xmlns="http://www.w3.org/2000/svg" style="display:block" width="%gmm" height="%gmm" viewBox="0 0 %g %g" '
                'font-family="Sukhumvit Set, Thonburi, sans-serif">%s%s</svg>') % (self.w, self.h, self.w, self.h, head, ''.join(self.out))


def table(pg, x, y, rows, widths, size=3.2, head=True):
    rh = size * 1.9
    for i, r in enumerate(rows):
        cx = x
        for j, (c, w) in enumerate(zip(r, widths)):
            pg.out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" stroke="#888" stroke-width="0.2"/>' % (
                cx, y + i * rh, w, rh, '#eee' if head and i == 0 else '#fff'))
            pg.text(cx + (w - 1.2 if j > 1 else 1.2), y + i * rh + rh * 0.68, str(c), size, bold=head and i == 0 or j == 0,
                    anchor='end' if j > 1 else 'start')
            cx += w
    return y + len(rows) * rh


FRAME_RGB = {'steel_tube': (0.80, 0.36, 0.30), 'steel_tab': (0.95, 0.66, 0.28)}
# everything that is not the welded frame, in greys so the frame stands out in the assembly views
OTHER_RGB = {'steel': (0.42, 0.44, 0.47), '6061': (0.74, 0.76, 0.79), 'al6061_hub': (0.74, 0.76, 0.79),
             'al5052_bracket': (0.66, 0.69, 0.72), 'purchased': (0.30, 0.32, 0.35), 'NBR': (0.16, 0.16, 0.16),
             'TPU95A': (0.85, 0.85, 0.85), 'UHMW': (0.88, 0.88, 0.88), 'EPDM': (0.16, 0.16, 0.16), 'belt': (0.2, 0.2, 0.2),
             'foam': (0.5, 0.5, 0.5), 'battery_650': (0.34, 0.42, 0.64), 'nylon_standoff': (0.9, 0.9, 0.9)}
# component name -> (Thai label, offset from the top centre of its box). Parts that share a Y station (wheel and drive
# motor, disc and upright) are pushed apart along Y so their labels do not stack in the side view.
PART_TH = {'JGA25_370_400rpm_Front_L': ('มอเตอร์ขับ', (0, -14, 4)), 'Wheel_Hub_6061_Front_R': ('ล้อ', (0, 30, 8)),
           'Beater_Disc_L': ('จานอาวุธ', (0, 26, 4)), 'D3536_1250kV_Can_ENVELOPE': ('มอเตอร์อาวุธ', (0, -6, 4)),
           'Upright_6061_6mm_R': ('แผ่นตั้งอาวุธ', (0, -20, 6)), 'Wedgelet_R': ('ลิ่มหน้า', (0, 12, 4)),
           '3S_650mAh_LiPo_ENVELOPE': ('แบต', (0, 0, 4)), 'ESP32_DevKit_51.5x28.3': ('บอร์ด', (0, 0, 4))}


def _view(DATUM, path, elev, azim, title, zoom, solids, overlay, labels, lim):
    """Shade solids [(shape, rgb, tol)] lit by face normal, draw overlay [(shape, rgb, alpha)] see-through on top, add
    labels [(xyz, text, colour)], save. Datum coordinates, Z up, nose +Y."""
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    light = np.array([0.35, -0.55, 0.76])
    light /= np.linalg.norm(light)
    d0 = np.array(DATUM)

    def tris_of(shape, rgb, tol):
        v, t = shape.tessellate(tol, 0.3)
        v = np.array([p.toTuple() for p in v]) - d0
        out, cols = [], []
        for a, b, c in t:
            tri = v[[a, b, c]]
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            k = 0.45 + 0.55 * abs(np.dot(n / nn, light))
            out.append(tri); cols.append(np.clip(np.array(rgb) * k + 0.08, 0, 1))
        return out, cols

    fig = plt.figure(figsize=(6.8, 4.25), facecolor='white')
    ax = fig.add_axes([0, 0, 1, 0.93], projection='3d', computed_zorder=False)   # whole figure, so nothing is clipped
    tris, cols = [], []
    for shape, rgb, tol in solids:
        a, b = tris_of(shape, rgb, tol); tris += a; cols += b
    pc = Poly3DCollection(tris, facecolors=cols, edgecolors=cols, linewidths=0.15)
    pc.set_clip_on(False)
    ax.add_collection3d(pc)
    for shape, rgb, alpha in overlay:
        a, b = tris_of(shape, rgb, 0.3)
        oc = Poly3DCollection(a, facecolors=[(*c, alpha) for c in b], edgecolors='none')
        oc.set_clip_on(False)
        ax.add_collection3d(oc)
    for (x, y, z), text, colour in labels:
        ax.text(x, y, z, text, fontsize=8, ha='center', zorder=10, color=colour, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.12', fc='white', ec='none', alpha=0.75), clip_on=False)
    xl, yl, zl = lim
    ax.quiver(81, yl[1] - 30, 0, 0, 22, 0, color='#0a4fa3', linewidth=1.4, arrow_length_ratio=0.35, zorder=11)
    ax.text(81, yl[1] - 4, 0, 'หัวรถ', color='#0a4fa3', fontsize=8, ha='center', fontweight='bold', zorder=11)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_zlim(*zl)
    ax.set_box_aspect((xl[1] - xl[0], yl[1] - yl[0], zl[1] - zl[0]), zoom=zoom)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.suptitle(title, fontsize=11, fontweight='bold', y=0.985)
    fig.savefig(path, dpi=230)
    plt.close(fig)


def _top(shape, DATUM, dz=3.0):
    bb = shape.BoundingBox()
    return bb.center.x - DATUM[0], bb.center.y - DATUM[1], bb.zmax - DATUM[2] + dz


def render_3d(B, ID, DATUM, path, elev, azim, title, zoom=1.3):
    """The as-welded frame alone, every tube and the big tabs labelled."""
    big_tabs = ('T01', 'T11', 'T02', 'T12', 'T23')      # small tabs are on page 3; labelling them here only clutters
    solids = [(B.AS_WELDED.get(m.name, m.shape), FRAME_RGB[m.material], 0.05) for m in B.steel]
    labels = [(_top(B.AS_WELDED.get(m.name, m.shape), DATUM), ID[m.name], '#000' if m.material == 'steel_tube' else '#6b3500')
              for m in B.steel if m.material == 'steel_tube' or ID[m.name] in big_tabs]
    _view(DATUM, path, elev, azim, title, zoom, solids, [], labels, ((-10, 172), (-5, 245), (-5, 60)))


def render_assembly(B, DATUM, path, elev, azim, title, skins=False, zoom=1.3, only=None):
    """The finished frame (holes drilled) inside the whole robot: frame in colour, the rest grey, PC skins see-through
    when asked (otherwise left out so the frame shows)."""
    solids, overlay, labels = [], [], []
    for b in B.bodies:
        if b.material == 'keepout':
            continue
        if b.material in FRAME_RGB:
            solids.append((b.shape, FRAME_RGB[b.material], 0.08))
        elif b.material == 'PC':
            if skins and b.name.startswith('Skin'):
                overlay.append((b.shape, (0.55, 0.75, 0.95), 0.16))
        else:
            solids.append((b.shape, OTHER_RGB.get(b.material, (0.6, 0.6, 0.6)), 0.15))
        if b.name in PART_TH and (only is None or PART_TH[b.name][0] in only):
            text, (dx, dy, dz) = PART_TH[b.name]
            x, y, z = _top(b.shape, DATUM, 0.0)
            labels.append(((x + dx, y + dy, z + dz), text, '#222'))
    rail = next(b for b in B.steel if b.name == 'Frame_Rail_L')
    labels.append((_top(rail.shape, DATUM, 2.0), 'โครงเหล็ก', '#8b1a0e'))
    _view(DATUM, path, elev, azim, title, zoom, solids, overlay, labels, ((-25, 187), (-20, 262), (-8, 66)))


def build(B, ID, DATUM, out_pdf, chrome):
    box = {}
    for m in B.steel:
        bb = B.AS_WELDED.get(m.name, m.shape).BoundingBox()
        box[m.name] = tuple(round(v, 2) + 0.0 for v in (bb.xmin - DATUM[0], bb.ymin - DATUM[1], bb.zmin - DATUM[2],
                                                         bb.xmax - DATUM[0], bb.ymax - DATUM[1], bb.zmax - DATUM[2]))
    n = {ID[k]: k for k in box}
    bx = lambda i: box[n[i]]                     # (x0, y0, z0, x1, y1, z1) by F/T id
    tubes = [m for m in B.steel if m.material == 'steel_tube']
    bottom = [ID[m.name] for m in tubes if bx(ID[m.name])[2] < 1]
    L = lambda i: max(bx(i)[3] - bx(i)[0], bx(i)[4] - bx(i)[1], bx(i)[5] - bx(i)[2])
    pages = []

    # ------------------------------------------------------------ page 1: bottom frame from above + cut list
    pg = Page('1) โครงล่าง มองจากด้านบน (หัวรถอยู่ด้านบน) — หน่วย มม.')
    pg.view(42, 164, 0.66)
    for i in bottom:
        x0, y0, _, x1, y1, _ = bx(i)
        pg.rect(x0, y0, x1, y1, TUBE_FILL, label=i, size=3.0)
    pg.utext(-4, -4, 'มุมอ้างอิง (0,0)', size=2.6, color=DIM, anchor='end')
    pg.circle(0, 0, 1.2)
    r_l, r_r, rear, mot, bulk, hinge = bx('F04'), bx('F08'), bx('F01'), bx('F12'), bx('F02'), bx('F05')
    pg.chain_x([rear[0], r_l[0], r_l[3], r_r[0], r_r[3], rear[3]], rear[1], -10)
    hl, hr = bx('F05'), bx('F09')
    pg.chain_x([hl[0], rear[0], hl[3], hr[0], rear[3], hr[3]], [hl[4], rear[1], hl[4], hr[4], rear[1], hr[4]], hl[4] + 12, total=False)
    pg.chain_v([rear[1], rear[4], mot[1], mot[4], bulk[1], bulk[4], hinge[1], hinge[4]], rear[0], -16)
    notes = ['รางข้าง F04 / F08 ยาวต่อเนื่องจาก F01 ถึง F05/F09 ห้ามตัดต่อ',
             'ท่อ F02, F12 อยู่ระหว่างราง · F03, F07 ต่อจากด้านนอกราง',
             'คานบานพับ F05/F09 วางชนปลายราง ยื่นเลยขอบนอก %s มม.' % fmt(rear[0] - hl[0]),
             'วัดเส้นทแยงมุมกรอบให้ต่างกันไม่เกิน 1 มม. ก่อนแต้มเชื่อม',
             'ภาพไม่ใช่สเกลจริง ให้ใช้ตัวเลข']
    for k, s in enumerate(notes):
        pg.text(175, 128 + k * 6, '• ' + s, 3.1, NOTE if k < 1 else INK, anchor='start')
    groups = {}
    for m in tubes:
        i = ID[m.name]
        wall = B.TUBE_WALL.get(m.name, B.WALL)
        groups.setdefault((round(L(i), 1), wall), []).append(i)
    rows = [('รหัส', 'ยาว', 'หนา', 'จำนวน')]
    for (length, wall), ids in sorted(groups.items(), key=lambda kv: -kv[0][0]):
        rows.append((' + '.join(ids), fmt(length), fmt(wall) + (' !' if wall != B.WALL else ''), len(ids)))
    pg.text(175, 22, 'ตัดท่อเหล็กกล่อง 10×10 (ตัดฉาก 90° ทุกท่อน)', 3.8, bold=True, anchor='start')
    yb = table(pg, 175, 25, rows, [34, 18, 18, 18])
    pg.text(175, yb + 5, '! = ท่อหนา %s มม. (รางข้าง รับแรงอาวุธ)' % fmt(B.RAIL_WALL), 3.0, NOTE, anchor='start')
    pg.text(175, yb + 10, 'รวม %d ท่อน ยาวรวม %s มม.' % (len(tubes), fmt(sum(L(ID[m.name]) for m in tubes))), 3.0, anchor='start')
    pages.append(pg)

    # ------------------------------------------------------------ page 2: rear view + drive motor section
    pg = Page('2) มองจากท้ายรถ — ซ้าย: เสาท้าย + คานบน · ขวา: หน้าตัดตรงคานรองมอเตอร์ F12')
    pg.view(22, 92, 0.62)
    for i in ('F01', 'F06', 'F10', 'F11'):
        x0, _, z0, x1, _, z1 = bx(i)
        pg.rect(x0, z0, x1, z1, TUBE_FILL, label=i, size=3.0)
    f6, f11 = bx('F06'), bx('F11')
    pg.chain_v([rear[2], rear[5], f6[5], f11[5]], rear[3], rear[3] + 14)
    pg.chain_x([rear[0], f6[3]], f6[5], f11[5] + 12, total=False)
    pg.utext(rear[0] + 81, -12, 'เสา F06/F10 ตั้งบน F01 ชิดปลาย · F11 วางบนหัวเสา', size=3.0, anchor='middle')

    pg.view(148, 92, 0.8)
    t01, t11, t23, f13 = bx('T01'), bx('T11'), bx('T23'), bx('F13')
    for i in ('F04', 'F08'):
        x0, _, z0, x1, _, z1 = bx(i)
        pg.rect(x0, z0, x1, z1, TUBE_FILL, label=i, size=2.6)
    pg.rect(mot[0], mot[2], mot[3], mot[5], TUBE_FILL, label='F12', size=2.8)
    pg.rect(f13[0], f13[2], f13[3], f13[5], TUBE_FILL, label='F13', size=2.6)
    for i in ('T01', 'T11', 'T23'):
        x0, _, z0, x1, _, z1 = bx(i)
        pg.rect(x0, z0, x1, z1, TAB_FILL)
    zc = 32.0 - DATUM[2]                               # drive axle height above the datum (z 32 in the robot)
    for x0, x1 in ((t01[3], 81.0 - 6.2), (81.0 + 6.2, t11[0])):
        pg.rect(x0, zc - 12.5, x1, zc + 12.5, 'none', dash=True)
    pg.utext((t01[3] + 74.8) / 2, zc - 1, 'มอเตอร์ / แท่ง Ø25', size=2.6, anchor='middle')
    for t in (t01, t11):                               # D8.5 hole seen edge-on: dashed lines through the plate
        for dz in (-4.25, 4.25):
            x0, y = pg.p(t[0] - 2, zc + dz); x1, _ = pg.p(t[3] + 2, zc + dz)
            pg.line(x0, y, x1, y, INK, 0.3, dash=True)
    pg.utext(t01[3] + 2, zc + 6.5, 'รู Ø8.5', size=2.6, anchor='start')
    pg.utext(t01[0] - 2, t01[5] + 3, 'T01', size=3.0, anchor='end', bold=True)
    pg.utext(t11[3] + 2, t11[5] + 3, 'T11', size=3.0, anchor='start', bold=True)
    pg.utext((t23[0] + t23[3]) / 2 + 3, (t23[2] + t23[5]) / 2 - 6, 'T23', size=3.0, anchor='start', bold=True)
    pg.chain_v([r_l[2], r_l[5], zc, t01[5]], t01[0], t01[0] - 12, total=False)
    pg.chain_v([mot[5], zc - 12.5, zc + 12.5], t11[3], t11[3] + 8, total=False)
    pg.chain_v([mot[5], f13[2]], t11[3], t11[3] + 16, total=False)          # T23 height
    pg.chain_x([f13[0], f13[3]], f13[5], f13[5] + 8, total=False)
    pg.chain_x([r_l[0], t01[0]], r_l[2], -8, total=False)
    notes = ['T01/T11 ตั้งบนราง ห่างขอบนอกราง %s มม. กว้าง (ตามแนวหน้า-หลัง) %s มม.' % (fmt(t01[0] - r_l[0]), fmt(t01[4] - t01[1])),
             'ขอบหลังของ T01/T11 ห่างขอบหลังรถ (ท้าย F01) %s มม.' % fmt(t01[1] - rear[1]),
             'รู Ø8.5 ทั้ง 2 แผ่น: ร้อยแท่ง Ø8 ยาว ≥ 130 ให้ตรงแกนเดียวกัน แล้วค่อยเชื่อม',
             'เสร็จแล้วถอดแท่ง Ø8 ก่อน (แท่งชนตำแหน่งเสา T23) แล้ววางแท่ง Ø25 แทนมอเตอร์',
             'รองใต้แท่ง Ø25 หนา %s · เว้นเหนือแท่ง %s (คือที่ของยาง) แล้วเชื่อม T23 + F13' % (fmt(zc - 12.5 - mot[5]), fmt(f13[2] - zc - 12.5)),
             'ไม่ต้องเจาะรูอื่นบน T01/T11 ผู้สั่งเจาะเอง']
    for k, s in enumerate(notes):
        pg.text(8, 128 + k * 6.5, '• ' + s, 3.3, NOTE if k in (2, 3, 5) else INK, anchor='start')
    pages.append(pg)

    # ------------------------------------------------------------ page 3: front tabs + small tabs on the rails
    pg = Page('3) แผ่นยึดด้านหน้า (มองจากด้านบน) + แผ่นเล็กบนราง')
    pg.view(22, 20 + 222 * 0.95, 0.95)
    y_lo = 126.0
    for i in ('F04', 'F08', 'F02', 'F03', 'F07', 'F05', 'F09'):
        x0, y0, _, x1, y1, _ = bx(i)
        pg.rect(x0, max(y0, y_lo), x1, y1, TUBE_FILL, label=i, size=2.8)
    for i in ('T03', 'T13', 'T04', 'T05', 'T14', 'T15', 'T02', 'T12', 'T06', 'T07', 'T08', 'T16', 'T17', 'T18', 'T21', 'T22'):
        x0, y0, _, x1, y1, _ = bx(i)
        pg.rect(x0, y0, x1, y1, TAB_FILL, lw=0.3)
    for i, du, dv, anc in (('T02', 2.5, -2, 'start'), ('T12', -2.5, -2, 'end'), ('T03', 0, 10, 'middle'), ('T13', 0, 10, 'middle'),
                           ('T21', 3, -1, 'start'), ('T22', 3, -1, 'start')):
        x0, y0, _, x1, y1, _ = bx(i)
        pg.utext((x0 + x1) / 2 + du, (y0 + y1) / 2 + dv, i, size=2.8, anchor=anc, bold=True)
    for i in ('T06', 'T07', 'T08', 'T16', 'T17', 'T18'):
        x0, y0, _, x1, y1, _ = bx(i)
        pg.utext((x0 + x1) / 2, y1 + 1.5, i, size=2.4, anchor='middle', bold=True)
    t02, t03, t04, t05 = bx('T02'), bx('T03'), bx('T04'), bx('T05')
    pg.chain_v([bulk[4], t04[1], t05[1], t03[4]], t03[3], t03[3] + 13, size=2.6, total=False)
    pg.chain_x([t03[0], t03[3]], 183.0, 183.0, size=2.6, total=False)     # flange width, drawn inside it between the rungs
    hl_t = [bx(i) for i in ('T08', 'T07', 'T06')]
    pg.chain_x([hl[0]] + [t[0] for t in hl_t], [hl[4]] + [t[4] for t in hl_t], hl[4] + 13, size=2.6, total=False)
    hr_t = [bx(i) for i in ('T16', 'T17', 'T18')]
    pg.chain_x([t[3] for t in hr_t] + [hr[3]], [t[4] for t in hr_t] + [hr[4]], hr[4] + 13, size=2.6, total=False)
    t21, t22 = bx('T21'), bx('T22')
    pg.chain_v([t21[1], t21[4], t22[1], t22[4]], t21[3], t21[3] + 9, size=2.6, total=False)
    pg.chain_v([t22[4], bulk[1]], t22[3], t22[3] + 9, size=2.6, total=False)
    h6 = bx('T06')
    notes = ['หูบานพับ T06–T08 / T16–T18 (สูง %s ยื่นหน้า %s): ร้อยแท่ง Ø3 ผ่านทั้ง 3 รูของแต่ละข้างขณะเชื่อม' % (fmt(h6[5] - h6[2]), fmt(h6[4] - h6[1])),
             'T03/T13 แผ่นพื้นวางบนโต๊ะ · T04/T05 แผ่นค้ำ สูง %s · T02/T12 แผ่นตั้ง สูง %s ตั้งบน T03' % (fmt(t04[5] - t04[2]), fmt(t02[5] - t02[2])),
             'T02/T12: ขันน็อตเข้ากับแผ่นตั้งอาวุธจริง (ผู้สั่งส่งให้) แต้มเชื่อมเท่านั้น ถอดออกก่อนเชื่อมเต็ม',
             'T02 ชิดขอบในของ T03 · ด้านขวา (T12–T15) เหมือนด้านซ้ายแต่กลับด้าน',
             'T21/T22 แผ่นฉาก (2 ชิ้นเชื่อมเป็น L) ติดด้านในรางซ้าย สูง %s / %s · T21 เริ่มที่ %s จากท้ายรถ' % (
                 fmt(t21[5] - t21[2]), fmt(t22[5] - t22[2]), fmt(t21[1] - rear[1]))]
    for k, s in enumerate(notes):
        pg.text(8, 140 + k * 6.5, '• ' + s, 3.2, NOTE if k == 2 else INK, anchor='start')
    rows = [('แผ่น', 'ติดที่', 'จากท้ายรถ', 'สูง')]
    for i in ('T09', 'T10', 'T19', 'T20'):
        x0, y0, z0, x1, y1, z1 = bx(i)
        rows.append((i, 'ด้านในราง' + ('ซ้าย F04' if x0 < 81 else 'ขวา F08'), '%s–%s' % (fmt(y0 - rear[1]), fmt(y1 - rear[1])), fmt(z1 - z0)))
    pg.text(208, 22, 'หูสายรัดแบต (แผ่นตั้งชิดด้านในราง)', 3.4, bold=True, anchor='start')
    table(pg, 208, 25, rows, [12, 25, 20, 12], size=3.0)
    pages.append(pg)

    # ------------------------------------------------------------ pages 4-5: 3D views all round
    import matplotlib
    matplotlib.rcParams['font.family'] = ['Sukhumvit Set', 'Tahoma']
    img = out_pdf.parent / 'images'
    img.mkdir(exist_ok=True)
    sets = [('4) ภาพ 3 มิติ มองจากด้านบน 4 มุม (สีแดง = ท่อ, สีส้ม = แผ่นยึด)',
             [(28, 135, 'หน้า-ซ้าย'), (28, 45, 'หน้า-ขวา'), (28, -135, 'หลัง-ซ้าย'), (28, -45, 'หลัง-ขวา')]),
            ('5) ภาพ 3 มิติ มุมต่ำ และมองจากใต้ท้อง',
             [(10, 90, 'ตรงหน้า (มุมต่ำ)'), (10, 0, 'ด้านขวา (มุมต่ำ)'), (-35, 135, 'ใต้ท้อง หน้า-ซ้าย'), (-35, -45, 'ใต้ท้อง หลัง-ขวา')])]
    for title, views in sets:
        pg = Page(title)
        for k, (elev, azim, name) in enumerate(views):
            fn = 'view3d_%s_%d.png' % ('top' if title.startswith('4') else 'low', k + 1)
            render_3d(B, ID, DATUM, img / fn, elev, azim, name)
            pg.image(2 + (k % 2) * 138, 12 + (k // 2) * 87, 135, 84.4, 'images/' + fn)
        pages.append(pg)
    pg = Page('6) โครงเหล็กเมื่อประกอบเป็นรถ (สีแดง/ส้ม = งานร้านเชื่อม · สีเทา = ชิ้นที่ผู้สั่งประกอบเอง)')
    side_labels = ('ล้อ', 'จานอาวุธ', 'แบต', 'ลิ่มหน้า')   # parts in line along X stack their labels in the side view
    for k, (elev, azim, name, skins, only) in enumerate([(28, 135, 'หน้า-ซ้าย (ไม่มีผนัง)', False, None),
                                                          (28, -45, 'หลัง-ขวา (ไม่มีผนัง)', False, None),
                                                          (10, 0, 'ด้านขวา (ไม่มีผนัง)', False, side_labels),
                                                          (28, 135, 'ประกอบเสร็จ (ผนังโปร่งใส)', True, None)]):
        fn = 'view3d_robot_%d.png' % (k + 1)
        render_assembly(B, DATUM, img / fn, elev, azim, name, skins, only=only)
        pg.image(2 + (k % 2) * 138, 12 + (k // 2) * 87, 135, 84.4, 'images/' + fn)
    pages.append(pg)

    css = ('@page{size:A4 landscape;margin:10mm}body{margin:0}.pg{page-break-after:always}.pg:last-child{page-break-after:auto}')
    doc = '<!doctype html><html lang="th"><head><meta charset="utf-8"><style>%s</style></head><body>%s</body></html>' % (
        css, ''.join('<div class="pg">%s</div>' % p.svg() for p in pages))
    tmp = out_pdf.with_suffix('.html')
    tmp.write_text(doc)
    subprocess.run([chrome, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + str(out_pdf), tmp.as_uri()],
                   check=True, capture_output=True, timeout=180)
    tmp.unlink()
    return {i: bx(i) for i in ID.values()}
