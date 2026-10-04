"""Tiny SVG engineering-drawing helper. Drawing units = robot millimetres.

Views (u right, v up on the page):
  top   : u =  x, v = y      (nose up the page)
  front : u = -x, v = z      (looking at the nose; robot's left on the page right)
  right : u =  y, v = z      (looking at the robot's right side; nose to the right)
  left  : u = -y, v = z
  rear  : u =  x, v = z
"""
import math
from html import escape

import geom as G

INK = '#111'
HID = '#8a8a8a'
SENS = '#c62828'
SIG = '#1565c0'
PWR = '#d32f2f'
MOT = '#ef6c00'
OK = '#2e7d32'
FILL_WHEEL = '#d9d9d9'
FILL_PART = '#f2f2f2'


def proj(view, x, y, z):
    return {
        'top': (x, y), 'front': (-x, z), 'right': (y, z), 'left': (-y, z), 'rear': (x, z),
    }[view]


class Fig:
    _uid = 0

    def __init__(self):
        Fig._uid += 1
        self.uid = Fig._uid
        self.markers = set()
        self.tscale = 1.0
        self.texts = []
        self.el = []
        self.umin = self.vmin = 1e9
        self.umax = self.vmax = -1e9

    def _marker(self, color):
        self.markers.add(color)
        return 'ah%d_%s' % (self.uid, color.lstrip('#'))

    def _ext(self, u, v):
        self.umin, self.umax = min(self.umin, u), max(self.umax, u)
        self.vmin, self.vmax = min(self.vmin, v), max(self.vmax, v)

    # --- primitives (page coords u, v) ---
    def line(self, a, b, color=INK, w=0.35, dash=None, arrow=None, ext=True):
        d = ' stroke-dasharray="%s"' % dash if dash else ''
        m = ''
        mid = self._marker(color)
        if arrow in ('end', 'both'):
            m += ' marker-end="url(#%s)"' % mid
        if arrow in ('start', 'both'):
            m += ' marker-start="url(#%s)"' % mid
        self.el.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="§%.3f§"%s%s style="color:%s"/>'
                       % (a[0], -a[1], b[0], -b[1], color, w, d, m, color))
        if ext:
            self._ext(*a)
            self._ext(*b)

    def poly(self, pts, color=INK, w=0.35, fill='none', dash=None, closed=True, arrow=None, ext=True, opacity=1):
        d = ' stroke-dasharray="%s"' % dash if dash else ''
        m = ' marker-end="url(#%s)"' % self._marker(color) if arrow == 'end' else ''
        tag = 'polygon' if closed else 'polyline'
        p = ' '.join('%.2f,%.2f' % (u, -v) for u, v in pts)
        self.el.append('<%s points="%s" stroke="%s" stroke-width="§%.3f§" fill="%s" fill-opacity="%.2f"%s%s '
                       'stroke-linejoin="round" style="color:%s"/>' % (tag, p, color, w, fill, opacity, d, m, color))
        if ext:
            for u, v in pts:
                self._ext(u, v)

    def rect(self, u0, u1, v0, v1, **kw):
        u0, u1 = sorted((u0, u1))
        v0, v1 = sorted((v0, v1))
        self.poly([(u0, v0), (u1, v0), (u1, v1), (u0, v1)], **kw)

    def circle(self, c, r, color=INK, w=0.35, fill='none', dash=None, opacity=1, ext=True):
        d = ' stroke-dasharray="%s"' % dash if dash else ''
        self.el.append('<circle cx="%.2f" cy="%.2f" r="%.2f" stroke="%s" stroke-width="§%.3f§" fill="%s" fill-opacity="%.2f"%s/>'
                       % (c[0], -c[1], r, color, w, fill, opacity, d))
        if ext:
            self._ext(c[0] - r, c[1] - r)
            self._ext(c[0] + r, c[1] + r)

    def arc_path(self, c, r, a0, a1, color=INK, w=0.35, fill='none', opacity=1, dash=None, wedge=False):
        """Arc from angle a0 to a1 (deg, CCW, page frame)."""
        p0 = (c[0] + r * math.cos(math.radians(a0)), c[1] + r * math.sin(math.radians(a0)))
        p1 = (c[0] + r * math.cos(math.radians(a1)), c[1] + r * math.sin(math.radians(a1)))
        large = 1 if (a1 - a0) % 360 > 180 else 0
        d = 'M %.2f %.2f A %.2f %.2f 0 %d 0 %.2f %.2f' % (p0[0], -p0[1], r, r, large, p1[0], -p1[1])
        if wedge:
            d = 'M %.2f %.2f L %.2f %.2f ' % (c[0], -c[1], p0[0], -p0[1]) + d[d.index('A'):] + ' Z'
        ds = ' stroke-dasharray="%s"' % dash if dash else ''
        self.el.append('<path d="%s" stroke="%s" stroke-width="§%.3f§" fill="%s" fill-opacity="%.2f"%s/>'
                       % (d, color, w, fill, opacity, ds))
        self._ext(c[0] - r, c[1] - r)
        self._ext(c[0] + r, c[1] + r)

    def text(self, p, s, size=4.2, color=INK, anchor='middle', weight='normal', rot=0, ext=True, italic=False):
        tr = ' transform="rotate(%.1f %.2f %.2f)"' % (-rot, p[0], -p[1]) if rot else ''
        st = ' font-style="italic"' if italic else ''
        self.el.append('<text x="%.2f" y="%.2f" font-size="§%.3f§" fill="%s" text-anchor="%s" font-weight="%s"%s%s>%s</text>'
                       % (p[0], -p[1], size, color, anchor, weight, tr, st, escape(s)))
        if ext:
            self.texts.append((p, size, anchor, len(s), rot))

    # --- engineering helpers ---
    def dim(self, a, b, off, label, size=3.6, color=INK, ext_lines=True, tpos=0.5):
        """Aligned dimension between a and b, offset `off` to the left of a->b."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1
        nx, ny = -dy / L, dx / L
        a2 = (a[0] + nx * off, a[1] + ny * off)
        b2 = (b[0] + nx * off, b[1] + ny * off)
        if ext_lines:
            s = 1 if off >= 0 else -1
            self.line(a, (a2[0] + nx * 1.5 * s, a2[1] + ny * 1.5 * s), color=color, w=0.18)
            self.line(b, (b2[0] + nx * 1.5 * s, b2[1] + ny * 1.5 * s), color=color, w=0.18)
        self.line(a2, b2, color=color, w=0.22, arrow='both')
        ang = math.degrees(math.atan2(dy, dx))
        if ang > 90 or ang < -90:
            ang += 180
        m = (a2[0] + (b2[0] - a2[0]) * tpos, a2[1] + (b2[1] - a2[1]) * tpos)
        s = 1 if off >= 0 else -1
        tp = (m[0] + nx * 1.0 * s, m[1] + ny * 1.0 * s)
        if s < 0:
            tp = (m[0] + nx * (size + 0.6) * s, m[1] + ny * (size + 0.6) * s)
        self.el.append('<text x="%.2f" y="%.2f" font-size="§%.3f§" fill="%s" text-anchor="middle" '
                       'transform="rotate(%.1f %.2f %.2f)" paint-order="stroke" stroke="#fff" stroke-width="§1.2§">%s</text>'
                       % (tp[0], -tp[1], size, color, -ang, tp[0], -tp[1], escape(label)))
        self._ext(*a2); self._ext(*b2); self._ext(tp[0], tp[1] + size)

    def badge(self, p, label, color=SENS, r=3.6):
        self.circle(p, r, color=color, w=0.4, fill='#fff')
        self.text((p[0], p[1] - 1.35), label, size=3.4, color=color, weight='bold')

    def leader(self, p, q, label, color=INK, size=3.6, anchor=None, badge=None):
        self.line(p, q, color=color, w=0.25)
        self.circle(p, 0.6, color=color, fill=color)
        if badge:
            self.badge(q, badge, color=color)
            if label:
                a = anchor or ('start' if q[0] >= p[0] else 'end')
                off = 5 if a == 'start' else -5
                self.text((q[0] + off, q[1] - 1.3), label, size=size, color=color, anchor=a)
        else:
            a = anchor or ('start' if q[0] >= p[0] else 'end')
            off = 1 if a == 'start' else -1
            self.text((q[0] + off, q[1] - 1.2), label, size=size, color=color, anchor=a)

    def angle_arc(self, c, r, a0, a1, label, color=INK, size=3.6):
        self.arc_path(c, r, a0, a1, color=color, w=0.25)
        am = math.radians((a0 + a1) / 2)
        self.text((c[0] + (r + 4) * math.cos(am), c[1] + (r + 4) * math.sin(am) - 1.2), label, size=size, color=color)

    def scalebar(self, p, length=50, label=None):
        u, v = p
        for i in range(5):
            self.rect(u + i * length / 5, u + (i + 1) * length / 5, v, v + 1.6,
                      color=INK, w=0.2, fill=INK if i % 2 == 0 else '#fff')
        self.text((u, v - 4.4), '0', size=3.0)
        self.text((u + length, v - 4.4), '%d mm' % length, size=3.0)
        if label:
            self.text((u + length / 2, v + 2.6), label, size=3.0)

    def _text_ext(self, k):
        e = [self.umin, self.umax, self.vmin, self.vmax]
        for p, size, anchor, n, rot in self.texts:
            sz = size * k
            w = 0.52 * sz * n
            if rot:
                us, vs = [p[0] - sz, p[0] + sz], [p[1] - w / 2, p[1] + w / 2]
            elif anchor == 'middle':
                us, vs = [p[0] - w / 2, p[0] + w / 2], [p[1] - sz * 0.3, p[1] + sz]
            elif anchor == 'start':
                us, vs = [p[0], p[0] + w], [p[1] - sz * 0.3, p[1] + sz]
            else:
                us, vs = [p[0] - w, p[0]], [p[1] - sz * 0.3, p[1] + sz]
            e = [min(e[0], *us), max(e[1], *us), min(e[2], *vs), max(e[3], *vs)]
        return e

    def svg(self, width_mm=None, scale=None, pad=4, extra=''):
        k = 1.0
        for _ in range(4):
            a, b, c, d = self._text_ext(k)
            W = b - a + 2 * pad
            if not width_mm:
                break
            k = max(0.3, min(2.4, 0.85 / (width_mm / W))) * self.tscale
        a, b, c, d = self._text_ext(k)
        u0, u1, v0, v1 = a - pad, b + pad, c - pad, d + pad
        W, H = u1 - u0, v1 - v0
        style = 'width:%.1fmm;max-width:100%%;' % width_mm if width_mm else 'width:100%;'
        mk = ''.join('<marker id="ah%d_%s" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="§3.2§" markerHeight="§3.2§" '
                     'orient="auto-start-reverse" markerUnits="userSpaceOnUse"><path d="M0,1.5 L10,5 L0,8.5 z" fill="%s"/></marker>'
                     % (self.uid, c.lstrip('#'), c) for c in sorted(self.markers))
        defs = ('<defs>' + mk +
                '<pattern id="hatch" width="2.5" height="2.5" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
                '<line x1="0" y1="0" x2="0" y2="2.5" stroke="#999" stroke-width="0.4"/></pattern>'
                '</defs>')
        out = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="%.2f %.2f %.2f %.2f" style="%s" '
                'font-family="Sukhumvit Set, Thonburi, sans-serif">%s%s%s</svg>'
                % (u0, -v1, W, H, style, defs, ''.join(self.el), extra))
        import re
        return re.sub('§([0-9.]+)§', lambda m: '%.3f' % (float(m.group(1)) * k), out)


# ---------------------------------------------------------------- robot body
def P(view, x, y, z):
    return proj(view, x, y, z)


def box(f, view, b, **kw):
    x0, x1, y0, y1, z0, z1 = b
    pts = [P(view, x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    f.rect(min(us), max(us), min(vs), max(vs), **kw)


def cyl(f, view, c, **kw):
    x0, x1, y, z, r = c
    if view in ('right', 'left'):
        f.circle(P(view, 0, y, z), r, **kw)
    else:
        box(f, view, (x0, x1, y - r, y + r, z - r, z + r), **kw)


def profile_x(f, view, pts_yz, xr, **kw):
    if view in ('right', 'left'):
        f.poly([P(view, 0, y, z) for y, z in pts_yz], **kw)
    else:
        ys = [p[0] for p in pts_yz]
        zs = [p[1] for p in pts_yz]
        box(f, view, (xr[0], xr[1], min(ys), max(ys), min(zs), max(zs)), **kw)


def skirts(f, view, **kw):
    s = [G.SKIRT_P_TOP, G.SKIRT_P_BOT,
         (G.SKIRT_P_BOT[0] + G.SKIRT_T * G.C35, G.SKIRT_P_BOT[1] + G.SKIRT_T * G.S35),
         (G.SKIRT_P_TOP[0] + G.SKIRT_T * G.C35, G.SKIRT_P_TOP[1] + G.SKIRT_T * G.S35)]
    us = [p[0] for p in s]
    zs = [p[1] for p in s]
    if view in ('front', 'rear'):
        for sg in (-1, 1):
            f.poly([P(view, sg * u, 0, z) for u, z in s], **kw)
        if view == 'rear':
            box(f, view, (-88, 88, 0, 0, min(zs), max(zs)), **kw)
    elif view == 'top':
        for sg in (-1, 1):
            box(f, view, (sg * min(us), sg * max(us), -110, 97, 0, 0), **kw)
        box(f, view, (-88, 88, -(max(us) + 22), -(min(us) + 22), 0, 0), **kw)
    else:  # side
        f.poly([P(view, 0, -(u + 22), z) for u, z in s], **kw)
        box(f, view, (0, 0, -110, 97, min(zs), max(zs)), **kw)


def robot(f, view, internals=True, light=False):
    ink = HID if light else INK
    w = 0.3 if light else 0.4
    # tub + lid + side walls + front floors
    box(f, view, (G.TUB['x0'], G.TUB['x1'], G.TUB['y0'], G.TUB['y1'], G.TUB['z0'], G.TUB['z1']), color=ink, w=w)
    for sg in (-1, 1):
        box(f, view, tuple(sorted((sg * 80, sg * 88))) + (64, 97, 7, 58), color=ink, w=w * 0.8)
        box(f, view, tuple(sorted((sg * 34, sg * 88))) + (64, 100, 4, 7), color=ink, w=w * 0.6)
    skirts(f, view, color=ink, w=0.3)
    # weapon
    for xr in G.WEDGELET_X:
        profile_x(f, view, G.WEDGELET_PROFILE, xr, color=ink, w=w, fill='#cfcfcf' if not light else 'none', opacity=0.35)
    for xr in G.UPRIGHT_X:
        profile_x(f, view, G.UPRIGHT_PROFILE, xr, color=ink, w=w, fill=FILL_PART if not light else 'none', opacity=0.6)
    for k in ('disc_l', 'disc_r', 'hub'):
        cyl(f, view, G.CYL[k], color=ink, w=w, dash=None)
    if view in ('right', 'left'):
        # tooth tip circle + base circle
        f.circle(P(view, 0, G.ROTOR_Y, G.AXLE_Z), G.BASE_R, color=ink, w=0.2, dash='1,1')
    box(f, view, G.BOX['top_brace'], color=ink, w=0.3)
    box(f, view, G.BOX['bot_brace'], color=ink, w=0.3)
    box(f, view, G.BOX['skid'], color=ink, w=0.3)
    # wheels
    for k, c in G.CYL.items():
        if k.startswith('wheel'):
            cyl(f, view, c, color=ink, w=w, fill=FILL_WHEEL if not light else 'none', opacity=0.55)
    if internals:
        for k, c in G.CYL.items():
            if k.startswith('motor_') or k in ('weapon_motor', 'pulley'):
                cyl(f, view, c, color=HID, w=0.25, dash='2,1')
        for k in ('battery', 'esc', 'esp32', 'drv_l', 'drv_r', 'switch', 'motor_mount'):
            box(f, view, G.BOX[k], color=HID, w=0.25, dash='2,1')


def sensors(f, view, color=SENS):
    box(f, view, G.BOX['imu'], color=color, w=0.5, fill=color, opacity=0.25)
    box(f, view, G.BOX['hall_post'], color=color, w=0.3, dash='1,0.6')
    box(f, view, G.BOX['hall'], color=color, w=0.5, fill=color, opacity=0.6)
