# -*- coding: utf-8 -*-
"""markdown -> docx，自己拼 WordprocessingML，不依赖 Word/WPS 的 HTML 导入。

为什么自己拼：WPS 导入 HTML 时会给每个 <p> 塞约 55pt 的段高（样式改不动、也不是文档网格），
导致目录 50 条排 6 页、全文被撑到 77 页。自己写 XML 后排版完全可控：
  * 封面单独一页（空段+显式段前距控制位置，居中）；
  * 每个一级标题 pageBreakBefore（章、附录各自起页）；
  * 目录用 TOC 域（含纯文本回退内容），打开后按 F9 即带页码；
  * 页脚"第 X 页 / 共 Y 页"（PAGE / NUMPAGES 域）；
  * 表格统一样式（表头行加粗 + 跨页重复表头）；
  * 正文宋体 10.5pt、单倍行距、段后 3pt；标题黑体；代码 Consolas 9pt。

用法: python md2docx.py <in.md> <out.docx>
"""
import html
import io
import re
import sys
import zipfile

W = 'xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"'
R = 'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"'

BODY_SZ = "21"        # 10.5pt = 21 half-points
H1, H2, H3, H4 = "32", "28", "24", "22"
CODE_SZ = "18"        # 9pt
TBL_SZ = "18"


def esc(s):
    return html.escape(s, quote=False).replace("\u00a0", "&#160;")


def runs(text, base=None):
    """处理 **粗体** 与 `代码`，生成多个 run"""
    out = []
    # 先按粗体切
    for i, seg in enumerate(re.split(r"\*\*(.+?)\*\*", text)):
        bold = (i % 2 == 1)
        for j, part in enumerate(re.split(r"`([^`]+)`", seg)):
            if part == "":
                continue
            code = (j % 2 == 1)
            rpr = ""
            if bold or code or base:
                rpr = "<w:rPr>" + ("<w:b/>" if bold else "")
                if code:
                    rpr += '<w:rFonts w:ascii="Consolas" w:hAnsi="Consolas" w:eastAsia="宋体"/>'
                rpr += base or ""
                rpr += "</w:rPr>"
            out.append('<w:r>%s<w:t xml:space="preserve">%s</w:t></w:r>' % (rpr, esc(part)))
    return "".join(out) or '<w:r><w:t xml:space="preserve"></w:t></w:r>'


def para(text, style=None, page_break=False, align=None, rpr=None, spacing=None):
    ppr = "<w:pPr>"
    if style:
        ppr += '<w:pStyle w:val="%s"/>' % style
    if page_break:
        ppr += "<w:pageBreakBefore/>"
    if spacing:
        ppr += spacing
    if align:
        ppr += '<w:jc w:val="%s"/>' % align
    ppr += "</w:pPr>"
    return "<w:p>%s%s</w:p>" % (ppr, runs(text, rpr))


def table(rows):
    if not rows:
        return ""
    ncol = max(len(r) for r in rows)
    grid = "".join('<w:gridCol w:w="%d"/>' % (9000 // ncol) for _ in range(ncol))
    borders = ("<w:tblBorders>"
               + "".join('<w:%s w:val="single" w:sz="4" w:space="0" w:color="000000"/>' % k
                         for k in ("top", "left", "bottom", "right", "insideH", "insideV"))
               + "</w:tblBorders>")
    out = ['<w:tbl><w:tblPr><w:tblW w:w="5000" w:type="pct"/>' + borders +
           '<w:tblLayout w:type="autofit"/><w:tblCellMar>'
           '<w:top w:w="20" w:type="dxa"/><w:left w:w="60" w:type="dxa"/>'
           '<w:bottom w:w="20" w:type="dxa"/><w:right w:w="60" w:type="dxa"/>'
           '</w:tblCellMar></w:tblPr><w:tblGrid>' + grid + "</w:tblGrid>"]
    for i, row in enumerate(rows):
        row = row + [""] * (ncol - len(row))
        trpr = "<w:trPr><w:tblHeader/></w:trPr>" if i == 0 else ""
        cells = []
        for c in row:
            body = para(c, style="TblText", rpr=('<w:b/>' if i == 0 else None))
            cells.append('<w:tc><w:tcPr><w:tcW w:w="0" w:type="auto"/></w:tcPr>%s</w:tc>' % body)
        out.append("<w:tr>%s%s</w:tr>" % (trpr, "".join(cells)))
    out.append("</w:tbl>")
    # 表格后补一个空段，避免两张表贴在一起
    out.append(para("", spacing='<w:spacing w:before="0" w:after="0" w:line="120" w:lineRule="auto"/>'))
    return "".join(out)


def parse(md):
    """切成块：cover / h / p / table / code / toc(纯文本目录占位)"""
    lines = md.replace("\r\n", "\n").split("\n")
    blocks = []
    i = 0
    in_code = False
    code = []
    cover = []
    started = False
    while i < len(lines):
        ln = lines[i]
        s = ln.rstrip()
        if s.startswith("```"):
            if in_code:
                blocks.append(("code", code))
                code, in_code = [], False
            else:
                in_code = True
            i += 1
            continue
        if in_code:
            code.append(ln)
            i += 1
            continue
        m = re.match(r"^(#{1,6})\s+(.*)$", s.strip())
        if m:
            started = True
            blocks.append(("h", (len(m.group(1)), m.group(2).strip())))
            i += 1
            continue
        if s.strip().startswith("|"):
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                cells = [c.strip() for c in lines[i].strip().strip("|").split("|")]
                if not set("".join(cells)) <= set("-: "):
                    rows.append(cells)
                i += 1
            blocks.append(("table", rows))
            continue
        if s.strip() == "":
            i += 1
            continue
        if not started:
            cover.append(s.strip())
            i += 1
            continue
        blocks.append(("p", s.strip()))
        i += 1
    return cover, blocks


def styles_xml():
    def hstyle(idx, name, sz, outline):
        return ('<w:style w:type="paragraph" w:styleId="Heading%d"><w:name w:val="heading %d"/>'
                '<w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>'
                '<w:pPr><w:keepNext/><w:spacing w:before="240" w:after="120" w:line="240" w:lineRule="auto"/>'
                '<w:outlineLvl w:val="%d"/></w:pPr>'
                '<w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="黑体" w:hAnsi="Arial"/><w:b/>'
                '<w:sz w:val="%s"/><w:szCs w:val="%s"/></w:rPr></w:style>'
                % (idx, idx, outline, sz, sz))

    return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<w:styles %s>'
            '<w:docDefaults><w:rPrDefault><w:rPr>'
            '<w:rFonts w:ascii="Times New Roman" w:eastAsia="宋体" w:hAnsi="Times New Roman"/>'
            '<w:sz w:val="%s"/><w:szCs w:val="%s"/></w:rPr></w:rPrDefault>'
            '<w:pPrDefault><w:pPr><w:spacing w:before="0" w:after="60" w:line="240" w:lineRule="auto"/>'
            '</w:pPr></w:pPrDefault></w:docDefaults>'
            '<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/>'
            '<w:qFormat/></w:style>'
            '%s%s%s%s'
            '<w:style w:type="paragraph" w:styleId="Code"><w:name w:val="Code"/><w:basedOn w:val="Normal"/>'
            '<w:pPr><w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
            '<w:rPr><w:rFonts w:ascii="Consolas" w:hAnsi="Consolas" w:eastAsia="宋体"/>'
            '<w:sz w:val="%s"/><w:szCs w:val="%s"/></w:rPr></w:style>'
            '<w:style w:type="paragraph" w:styleId="TblText"><w:name w:val="Table Text"/>'
            '<w:basedOn w:val="Normal"/>'
            '<w:pPr><w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
            '<w:rPr><w:sz w:val="%s"/><w:szCs w:val="%s"/></w:rPr></w:style>'
            '<w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/></w:style>'
            '</w:styles>') % (W, BODY_SZ, BODY_SZ,
                              hstyle(1, "Heading 1", H1, 0), hstyle(2, "Heading 2", H2, 1),
                              hstyle(3, "Heading 3", H3, 2), hstyle(4, "Heading 4", H4, 3),
                              CODE_SZ, CODE_SZ, TBL_SZ, TBL_SZ)


def build(md, out, numpages=1):
    cover, blocks = parse(md)

    body = []
    # ---- 封面：空段撑版（显式行高，避免被样式影响） ----
    def blank(n=1, line=360):
        return "".join(para("", spacing='<w:spacing w:before="0" w:after="0" w:line="%d" w:lineRule="auto"/>' % line)
                       for _ in range(n))

    body.append(blank(8))
    if cover:
        body.append('<w:p><w:pPr><w:jc w:val="center"/><w:spacing w:before="0" w:after="240" '
                    'w:line="480" w:lineRule="auto"/></w:pPr>'
                    '<w:r><w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="黑体" w:hAnsi="Arial"/><w:b/>'
                    '<w:sz w:val="52"/><w:szCs w:val="52"/></w:rPr>'
                    '<w:t xml:space="preserve">%s</w:t></w:r></w:p>' % esc(cover[0]))
        body.append(blank(4, 360))
        for c in cover[1:-1]:
            if c:
                body.append(para(c, align="center", rpr='<w:sz w:val="28"/><w:szCs w:val="28"/>',
                                 spacing='<w:spacing w:before="0" w:after="120" w:line="360" w:lineRule="auto"/>'))
        if len(cover) > 1:
            body.append(blank(10, 360))
            body.append(para(cover[-1], align="center",
                             rpr='<w:rFonts w:eastAsia="黑体"/><w:b/><w:sz w:val="32"/><w:szCs w:val="32"/>',
                             spacing='<w:spacing w:before="0" w:after="0" w:line="360" w:lineRule="auto"/>'))

    # ---- 正文；目录处插入 TOC 域（回退内容用纯文本目录） ----
    heads = [(lvl, txt) for (k, v) in blocks if k == "h" for (lvl, txt) in [v]]
    for kind, val in blocks:
        if kind == "h":
            lvl, txt = val
            body.append(para(txt, style="Heading%d" % min(lvl, 4), page_break=(lvl == 1)))
            if txt == "目录":
                toc_items = [(l, t) for (l, t) in heads if t != "目录" and l <= 3]
                body.append('<w:p><w:pPr><w:spacing w:before="120" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
                            '<w:r><w:fldChar w:fldCharType="begin"/></w:r>'
                            '<w:r><w:instrText xml:space="preserve"> TOC \\o "1-3" \\h \\z \\u </w:instrText></w:r>'
                            '<w:r><w:fldChar w:fldCharType="separate"/></w:r></w:p>')
                for (l, t) in toc_items:
                    body.append(para(t, spacing='<w:spacing w:before="0" w:after="0" w:line="240" '
                                                'w:lineRule="auto"/><w:ind w:left="%d"/>' % ((l - 1) * 220)))
                body.append('<w:p><w:pPr><w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
                            '<w:r><w:fldChar w:fldCharType="end"/></w:r></w:p>')
        elif kind == "p":
            t = val
            if t.startswith(">"):
                t = t.lstrip("> ").strip()
                body.append(para(t, rpr='<w:i/>'))
            else:
                body.append(para(t))
        elif kind == "code":
            for ln in val:
                body.append(para(ln if ln.strip() else " ", style="Code"))
        elif kind == "table":
            body.append(table(val))

    # ---- 节属性 + 页脚 ----
    body.append('<w:sectPr><w:footerReference w:type="default" r:id="rId1"/>'
                '<w:pgSz w:w="11906" w:h="16838"/>'
                '<w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" '
                'w:header="851" w:footer="992" w:gutter="0"/></w:sectPr>')

    document = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<w:document %s %s><w:body>%s</w:body></w:document>' % (W, R, "".join(body)))

    footer = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
              '<w:ftr %s><w:p><w:pPr><w:jc w:val="center"/></w:pPr>'
              '<w:r><w:t xml:space="preserve">第 </w:t></w:r>'
              '<w:fldSimple w:instr=" PAGE "><w:r><w:t>1</w:t></w:r></w:fldSimple>'
              '<w:r><w:t xml:space="preserve"> 页 / 共 </w:t></w:r>'
              '<w:fldSimple w:instr=" NUMPAGES "><w:r><w:t>%d</w:t></w:r></w:fldSimple>'
              '<w:r><w:t xml:space="preserve"> 页</w:t></w:r></w:p></w:ftr>' % (W, numpages))

    content_types = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                     '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                     '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                     '<Default Extension="xml" ContentType="application/xml"/>'
                     '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
                     '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
                     '<Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>'
                     '</Types>')

    rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>'
            '</Relationships>')

    drels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
             '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
             '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/>'
             '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
             '</Relationships>')

    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        z.writestr("_rels/.rels", rels)
        z.writestr("word/document.xml", document)
        z.writestr("word/styles.xml", styles_xml())
        z.writestr("word/footer1.xml", footer)
        z.writestr("word/_rels/document.xml.rels", drels)

    nh = sum(1 for k, _ in blocks if k == "h")
    nt = sum(1 for k, _ in blocks if k == "table")
    nc = sum(1 for k, _ in blocks if k == "code")
    print("wrote %s   标题 %d / 表格 %d / 代码块 %d / 正文段 %d"
          % (out, nh, nt, nc, sum(1 for k, _ in blocks if k == "p")))


if __name__ == "__main__":
    md_path, out_path = sys.argv[1], sys.argv[2]
    npages = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    build(io.open(md_path, encoding="utf-8").read(), out_path, npages)
