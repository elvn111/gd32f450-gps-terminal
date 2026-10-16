# -*- coding: utf-8 -*-
"""协议 markdown -> HTML（给 WPS/Word 导入用）。

特性：
  * 首个标题之前的内容当作封面，居中放大；
  * 标题 == "目录" 的位置自动插入由各级标题生成的目录；
  * 管道表格转 <table>（带边框，供 WPS 转 docx 后仍是表格）；
  * ``` 代码块转 <pre>；**粗体** 转 <b>；转义 HTML 实体与表格里的 \\| 。
用法: python md2html.py <in.md> <out.html>
"""
import html
import io
import re
import sys

CSS = """
body   { font-family: "宋体", SimSun, serif; font-size: 10.5pt; line-height: 1.5; }
h1     { font-family: "黑体", SimHei, sans-serif; font-size: 16pt; margin: 18pt 0 8pt 0; }
h2     { font-family: "黑体", SimHei, sans-serif; font-size: 14pt; margin: 14pt 0 6pt 0; }
h3     { font-family: "黑体", SimHei, sans-serif; font-size: 12pt; margin: 12pt 0 6pt 0; }
h4     { font-family: "黑体", SimHei, sans-serif; font-size: 11pt; margin: 10pt 0 4pt 0; }
table  { border-collapse: collapse; margin: 6pt 0; }
td, th { border: 1px solid #000000; padding: 2pt 5pt; font-size: 9pt; vertical-align: top; }
th     { background: #f2f2f2; font-weight: bold; }
pre    { font-family: Consolas, "Courier New", monospace; font-size: 9pt;
         background: #f7f7f7; border: 1px solid #cccccc; padding: 4pt 6pt; }
.cover { text-align: center; margin-top: 220pt; }
.cover .t { font-size: 26pt; font-weight: bold; font-family: "黑体", SimHei, sans-serif; }
.cover .s { font-size: 14pt; margin-top: 28pt; }
.cover .c { font-size: 16pt; margin-top: 90pt; font-family: "黑体", SimHei, sans-serif; }
.toc   { margin: 6pt 0 12pt 0; }
.toc div { font-size: 10.5pt; line-height: 1.6; }
"""


def inline(s):
    s = html.escape(s, quote=False)
    s = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", s)
    s = s.replace("\\|", "|")
    s = s.replace("`(.+?)`", r"\1")
    s = re.sub(r"`([^`]+)`", r"<span style=\"font-family:Consolas,monospace\">\1</span>", s)
    return s


def main():
    src, dst = sys.argv[1], sys.argv[2]
    lines = io.open(src, encoding="utf-8").read().split("\n")

    heads = []          # (level, text)
    body = []
    i = 0
    started = False
    in_code = False
    in_table = False
    cover = []

    while i < len(lines):
        ln = lines[i]
        s = ln.strip()

        if s.startswith("```"):
            if not in_code:
                body.append("<pre>")
                in_code = True
            else:
                body.append("</pre>")
                in_code = False
            i += 1
            continue

        if in_code:
            body.append(html.escape(ln))
            i += 1
            continue

        m = re.match(r"^(#{1,6})\s+(.*)$", s)
        if m:
            lvl, txt = len(m.group(1)), m.group(2).strip()
            if not started:
                started = True          # 封面结束
            heads.append((lvl, txt))
            if in_table:
                body.append("</table>")
                in_table = False
            if txt == "目录":
                body.append("<h%d>%s</h%d>" % (lvl, inline(txt), lvl))
                body.append("<!--TOC-->")
            else:
                body.append("<h%d>%s</h%d>" % (lvl, inline(txt), lvl))
            i += 1
            continue

        if s.startswith("|"):
            cells = [c.strip() for c in s.strip("|").split("|")]
            if set("".join(cells)) <= set("-: "):
                i += 1
                continue                # 分隔行
            if not in_table:
                body.append("<table>")
                in_table = True
                row_idx = 0
            tag = "th" if row_idx == 0 else "td"
            row_idx += 1
            body.append("<tr>" + "".join("<%s>%s</%s>" % (tag, inline(c), tag) for c in cells) + "</tr>")
            i += 1
            continue

        if in_table:
            body.append("</table>")
            in_table = False

        if not started:
            if s == "":
                if cover and cover[-1] != "":
                    cover.append("")
            else:
                cover.append(s)
            i += 1
            continue

        if s == "":
            i += 1
            continue

        if s.startswith(">"):
            body.append("<p><i>%s</i></p>" % inline(s.lstrip("> ").strip()))
        else:
            body.append("<p>%s</p>" % inline(s))
        i += 1

    if in_code:
        body.append("</pre>")
    if in_table:
        body.append("</table>")

    # ---- 目录 ----
    toc_items = []
    for lvl, txt in heads:
        if txt == "目录":
            continue
        toc_items.append('<div style="margin-left:%dpt">%s</div>' % ((lvl - 1) * 14, inline(txt)))
    toc = '<div class="toc">' + "\n".join(toc_items) + "</div>"
    body_html = "\n".join(body).replace("<!--TOC-->", toc)

    # ---- 封面 ----
    cov = []
    if cover:
        cov.append('<div class="cover">')
        for k, c in enumerate(cover):
            if k == 0:
                cov.append('<p class="t">%s</p>' % inline(c))
            elif c == "":
                cov.append("<p>&nbsp;</p>")
            elif k == len(cover) - 1:
                cov.append('<p class="c">%s</p>' % inline(c))
            else:
                cov.append('<p class="s">%s</p>' % inline(c))
        cov.append("</div><br clear=all style='page-break-before:always'>")

    out = ("""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>%s</title><style>%s</style></head><body>
%s
%s
</body></html>""" % (inline(cover[0]) if cover else "protocol", CSS, "\n".join(cov), body_html))

    io.open(dst, "w", encoding="utf-8").write(out)
    print("wrote %s (%d 标题, %d 字符)" % (dst, len(heads), len(out)))


if __name__ == "__main__":
    main()
