"""Document framework: section/figure/table numbering, tags, HTML helpers."""
from html import escape

TAGS = {
    'CAD': ('CAD', '#1565c0', 'ค่าจาก build_robot_v4.py / ROBOT_V4_Beater.f3d'),
    'FW': ('FW', '#6a1b9a', 'ค่าจากเฟิร์มแวร์ v4_controller.ino'),
    'ST': ('STATUS', '#00695c', 'ค่าจาก robot_v4/STATUS.md หรือ manufacturing/README.md'),
    'REC': ('แนะนำ', '#ef6c00', 'ข้อแนะนำของคู่มือนี้ ยังไม่อยู่ในแบบ CAD'),
    'MEAS': ('วัดจริง', '#c62828', 'ต้องวัดจาก Robot V4 จริงแล้วกรอก'),
    'CHK': ('ตรวจบอร์ด', '#5d4037', 'ข้อมูลทั่วไปของโมดูล ต้องตรวจกับป้าย/datasheet ของชิ้นจริง'),
}


def tag(k):
    t, c, _ = TAGS[k]
    return '<span class="tag" style="border-color:%s;color:%s">%s</span>' % (c, c, t)


def blank(unit='mm', w=18):
    return '<span class="blank" style="min-width:%dmm"></span> %s' % (w, unit)


class Doc:
    def __init__(self):
        self.parts = []
        self.toc = []          # (level, number, title)
        self.sec = 0
        self.sub = 0
        self.fig_n = {}
        self.tab_n = {}

    def add(self, html):
        self.parts.append(html)

    def h1(self, title, newpage=True, num=None):
        if num is None:
            self.sec += 1
            num = str(self.sec)
        self.sub = 0
        self.toc.append((1, num, title))
        cls = ' class="newpage"' if newpage else ''
        self.add('<h1%s><span class="mk">@@H%s@@</span><span class="num">%s</span> %s</h1>' % (cls, num, num, escape(title)))

    def h2(self, title):
        self.sub += 1
        num = '%d.%d' % (self.sec, self.sub)
        self.toc.append((2, num, title))
        self.add('<h2><span class="mk">@@H%s@@</span><span class="num">%s</span> %s</h2>' % (num, num, escape(title)))

    def h3(self, title):
        self.add('<h3>%s</h3>' % title)

    def p(self, html):
        self.add('<p>%s</p>' % html)

    def ul(self, items, cls=''):
        self.add('<ul class="%s">%s</ul>' % (cls, ''.join('<li>%s</li>' % i for i in items)))

    def ol(self, items):
        self.add('<ol>%s</ol>' % ''.join('<li>%s</li>' % i for i in items))

    def note(self, html, kind='info'):
        self.add('<div class="note %s">%s</div>' % (kind, html))

    def figure(self, svg_or_html, caption, cls=''):
        n = self.fig_n.get(self.sec, 0) + 1
        self.fig_n[self.sec] = n
        num = '%d.%d' % (self.sec, n)
        self.add('<figure class="%s">%s<figcaption><b>Figure %s</b> %s</figcaption></figure>' % (cls, svg_or_html, num, caption))
        return num

    def figgrid(self, items, caption, cols=2):
        cells = ''.join('<div class="cell">%s<div class="sub">%s</div></div>' % (s, c) for s, c in items)
        return self.figure('<div class="grid g%d">%s</div>' % (cols, cells), caption)

    def table(self, header, rows, caption, widths=None, cls=''):
        n = self.tab_n.get(self.sec, 0) + 1
        self.tab_n[self.sec] = n
        num = '%d.%d' % (self.sec, n)
        cg = ''
        if widths:
            cg = '<colgroup>%s</colgroup>' % ''.join('<col style="width:%s">' % w for w in widths)
        th = '<tr>%s</tr>' % ''.join('<th>%s</th>' % h for h in header)
        tb = ''.join('<tr>%s</tr>' % ''.join('<td>%s</td>' % c for c in r) for r in rows)
        self.add('<div class="tblwrap %s"><div class="tcap"><b>Table %s</b> %s</div><table>%s<thead>%s</thead><tbody>%s</tbody></table></div>'
                 % (cls, num, caption, cg, th, tb))
        return num

    def pagebreak(self):
        self.add('<div class="newpage"></div>')


CSS = r"""
@page {
  size: A4;
  margin: 16mm 15mm 17mm 15mm;
  @bottom-left { content: "Robot V4 — คู่มือการติดตั้ง Sensor · Rev A (CAD 9321d02)"; font-family: "Sukhumvit Set", Thonburi, sans-serif; font-size: 7.5pt; color: #666; }
  @bottom-right { content: "หน้า " counter(page) " / " counter(pages); font-family: "Sukhumvit Set", Thonburi, sans-serif; font-size: 8pt; color: #333; }
  @top-right { content: "V4-SNS-MAN-001"; font-family: "Sukhumvit Set", Thonburi, sans-serif; font-size: 7.5pt; color: #888; }
}
@page cover { @bottom-left { content: none; } @bottom-right { content: none; } @top-right { content: none; } }
html { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
body { font-family: "Sukhumvit Set", Thonburi, sans-serif; font-size: 10pt; line-height: 1.55; color: #111; background: #fff; margin: 0; }
.cover { page: cover; }
.cover h1 { font-size: 26pt; line-height: 1.25; margin: 0 0 4mm 0; border: none; padding: 0; }
.cover .kicker { font-size: 11pt; color: #c62828; font-weight: bold; letter-spacing: 0.04em; margin-top: 8mm; }
.cover .sub { font-size: 13pt; color: #333; margin-bottom: 6mm; }
.cover img { width: 100%; max-height: 120mm; object-fit: cover; border: 1px solid #ccc; margin: 3mm 0; }
.cover table { font-size: 9pt; }
.newpage { break-before: page; }
h1 { font-size: 17pt; margin: 0 0 4mm 0; padding: 0 0 2mm 0; border-bottom: 2px solid #111; }
h2 { font-size: 12.5pt; margin: 6mm 0 2mm 0; padding-bottom: 1mm; border-bottom: 0.5px solid #999; break-after: avoid; }
h3 { font-size: 10.5pt; margin: 4mm 0 1.5mm 0; break-after: avoid; }
h1 .num, h2 .num { color: #c62828; margin-right: 2mm; }
.mk { color: #fff; font-size: 1px; }
p { margin: 1.5mm 0 2mm 0; }
ul, ol { margin: 1mm 0 2mm 0; padding-left: 6mm; }
li { margin: 0.6mm 0; }
figure { margin: 3mm 0 4mm 0; text-align: center; break-inside: avoid; }
figure svg { display: block; margin: 0 auto; background: #fff; }
figcaption { font-size: 9pt; margin-top: 1.5mm; color: #222; }
.grid { display: grid; gap: 3mm; align-items: end; }
.g2 { grid-template-columns: 1fr 1fr; } .g3 { grid-template-columns: 1fr 1fr 1fr; } .g1 { grid-template-columns: 1fr; }
.grid .cell svg { max-width: 100%; }
.grid .sub { font-size: 8.5pt; color: #444; margin-top: 1mm; }
.tblwrap { margin: 3mm 0 4mm 0; break-inside: auto; }
.tblwrap.keep { break-inside: avoid; }
.tcap { font-size: 9pt; margin-bottom: 1mm; }
table { width: 100%; border-collapse: collapse; font-size: 8.8pt; line-height: 1.4; }
th, td { border: 0.6px solid #777; padding: 1.1mm 1.6mm; vertical-align: top; text-align: left; }
th { background: #eceff1; font-weight: bold; }
tr { break-inside: avoid; }
td.c, th.c { text-align: center; }
.tag { display: inline-block; border: 0.8px solid; border-radius: 2px; font-size: 6.8pt; line-height: 1.2; padding: 0 1mm; margin-left: 0.8mm; vertical-align: 1px; font-weight: bold; white-space: nowrap; }
.blank { display: inline-block; border-bottom: 0.8px solid #c62828; height: 3.6mm; vertical-align: -0.5mm; }
.box { display: inline-block; width: 3.4mm; height: 3.4mm; border: 0.9px solid #111; vertical-align: -0.6mm; margin-right: 1.5mm; }
.note { border-left: 3px solid #1565c0; background: #e3f2fd; padding: 2mm 3mm; margin: 3mm 0; break-inside: avoid; }
.note.warn { border-color: #c62828; background: #ffebee; }
.note.ok { border-color: #2e7d32; background: #e8f5e9; }
.note.adapt { border-color: #ef6c00; background: #fff3e0; }
.toc { font-size: 10pt; }
.toc .l1 { font-weight: bold; margin-top: 2mm; }
.toc .l2 { padding-left: 8mm; }
.toc div { display: flex; }
.toc .t { flex: 0 1 auto; } .toc .n { flex: 0 0 11mm; color: #c62828; }
.toc .dots { flex: 1 1 auto; border-bottom: 1px dotted #999; margin: 0 2mm 1.4mm 2mm; }
.toc .pg { flex: 0 0 auto; width: 10mm; text-align: right; }
.step { border: 0.8px solid #999; border-radius: 2px; padding: 2mm 3mm; margin: 3mm 0; break-inside: avoid; display: grid; grid-template-columns: 1fr 72mm; gap: 3mm; align-items: center; }
.step h3 { margin-top: 0; color: #c62828; }
.step figure { margin: 0; }
.step svg { max-width: 100%; }
.step.wide { grid-template-columns: 1fr; }
.step.wide svg { max-height: 120mm; }
.small { font-size: 8.5pt; color: #444; }
.mono { font-family: Menlo, monospace; font-size: 8.5pt; }
"""
