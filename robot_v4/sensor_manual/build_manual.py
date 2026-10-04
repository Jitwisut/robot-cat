"""Build robot_v4/V4_Sensor_Installation_Manual_TH.pdf (HTML + inline SVG -> headless Chrome).

Two passes: the first PDF gives the page of every heading (hidden @@H<n>@@ markers),
the second fills the table of contents. Needs Google Chrome and pypdf.
"""
import os
import re
import subprocess
import sys
import tempfile
from html import escape

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import geom
import content_a as A
import content_b as Bm
import content_c as C
from doc import Doc, CSS

OUT = os.path.join(os.path.dirname(HERE), 'V4_Sensor_Installation_Manual_TH.pdf')
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'


def build_doc():
    d = Doc()
    A.cover(d)
    A.toc_placeholder(d)
    A.how_to_read(d)
    A.section1(d); A.section2(d); A.section3(d); A.section4(d)
    Bm.section5(d); Bm.section6(d); Bm.section7(d); Bm.section8(d); Bm.section9(d); Bm.section10(d)
    C.section11(d); C.section12(d); C.section13(d); C.section14(d); C.section15(d); C.section16(d)
    C.appendix(d)
    return d


def toc_html(d, pages):
    out = []
    for lvl, num, title in d.toc:
        pg = pages.get(num, '')
        out.append('<div class="l%d"><span class="n">%s</span><span class="t">%s</span><span class="dots"></span><span class="pg">%s</span></div>'
                   % (lvl, num, escape(title), pg))
    return ''.join(out)


def render(html, pdf):
    with tempfile.NamedTemporaryFile('w', suffix='.html', delete=False, encoding='utf-8') as f:
        f.write(html)
        src = f.name
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                    '--print-to-pdf=' + pdf, 'file://' + src], check=True, capture_output=True)
    os.unlink(src)


def heading_pages(pdf):
    from pypdf import PdfReader
    pages = {}
    for i, p in enumerate(PdfReader(pdf).pages):
        for m in re.findall(r'@@H([0-9A-Z.]+)@@', (p.extract_text() or '').replace(' ', '')):
            pages.setdefault(m, i + 1)
    return pages


def main():
    geom.selfcheck()
    d = build_doc()
    body = ''.join(d.parts)
    page = ('<!doctype html><html lang="th"><head><meta charset="utf-8"><title>Robot V4 Sensor Manual</title>'
            '<style>%s</style></head><body>%s</body></html>')
    tmp = OUT + '.pass1.pdf'
    render(page % (CSS, body.replace('@@TOC@@', toc_html(d, {}))), tmp)
    pages = heading_pages(tmp)
    os.unlink(tmp)
    missing = [n for _, n, _ in d.toc if n not in pages]
    render(page % (CSS, body.replace('@@TOC@@', toc_html(d, pages))), OUT)
    final = heading_pages(OUT)
    drift = {k: (pages[k], final.get(k)) for k in pages if final.get(k) != pages[k]}
    from pypdf import PdfReader
    print('pages:', len(PdfReader(OUT).pages), '| headings:', len(d.toc), '| missing:', missing, '| drift:', drift)
    print(OUT)


if __name__ == '__main__':
    main()
