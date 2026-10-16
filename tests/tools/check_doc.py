# -*- coding: utf-8 -*-
"""校验协议文档：
1) 代码块里按帧头+N 定帧，逐帧检查帧头/长度/CRC/帧尾，并打印解码结果；
2) 检查表号（只看表题行）是否从 1 连续编号；
3) 列出正文里“校验码 0xXXXX”的引用，便于人工对帧。
用法: python check_doc.py <doc.md>
"""
import io
import re
import struct
import sys


HEX_RE = re.compile(r"(?<![0-9A-Fa-f])[0-9A-Fa-f]{2}(?![0-9A-Fa-f])")


def hex_tokens(text):
    """抽十六进制字节。先去掉 recv:/send: 这类行标签，否则 'recv' 里的 'ec'
    会被当成一个字节，整块样例就被误判成"不是帧"。"""
    text = re.sub(r"(?i)\b(?:recv|send|tx|rx)\s*:", " ", text)
    return [int(x, 16) for x in HEX_RE.findall(text)]

def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def deframe(tokens):
    """按 A6 6A + 长度 N(高字节先发) 把字节流切成帧"""
    frames = []
    i = 0
    while i < len(tokens):
        if tokens[i] == 0xA6 and i + 5 < len(tokens) and tokens[i + 1] == 0x6A:
            n = (tokens[i + 4] << 8) | tokens[i + 5]
            end = i + 10 + n
            if end <= len(tokens):
                frames.append(tokens[i:end])
                i = end
                continue
        i += 1
    return frames


def main():
    path = sys.argv[1]
    t = io.open(path, encoding="utf-8").read()

    blocks = re.findall(r"```(.*?)```", t, re.S)
    nframe = nbad = nlost = 0
    for bi, b in enumerate(blocks, 1):
        toks = hex_tokens(b)
        if not toks or toks[0] != 0xA6:
            continue
        got = deframe(toks)
        if not got and (len(toks) >= 8) and ("+" not in b) and ("|" not in b):
            # 像一串真帧却一帧都切不出来：长度字段字节序或定帧逻辑有问题
            # (带 +/| 的块是 ASCII 示意图，不算)
            print("LOST 块%-2d 以 A6 6A 开头但切不出帧（检查长度字段字节序）: %s"
                  % (bi, " ".join("%02X" % x for x in toks[:12])))
            nlost += 1
        for raw in got:
            raw = bytes(raw)
            nframe += 1
            n = struct.unpack_from(">H", raw, 4)[0]
            c = struct.unpack_from("<H", raw, 6 + n)[0]
            calc = crc16_modbus(raw[:6 + n])
            ok = (len(raw) == 10 + n) and (raw[-2:] == b"\xb7\x7b") and (c == calc)
            print("%-4s 块%-2d cmd=0x%02X func=0x%02X N=%2d 总长=%2d CRC=0x%04X%s"
                  % ("OK" if ok else "FAIL", bi, raw[2], raw[3], n, len(raw), c,
                     "" if ok else "  (算得 0x%04X)" % calc))
            if not ok:
                nbad += 1

    caps = [int(m.group(1)) for m in re.finditer(r"^表\s*(\d+)\s", t, re.M)]
    print("\n表题编号:", caps)
    good = caps == list(range(1, len(caps) + 1))
    print("表号从 1 连续编号:", "OK" if good else "FAIL")

    print("\n正文里的校验码引用:")
    for m in re.finditer(r"校验码\s*0x([0-9A-F]{4})", t):
        print("   ", m.group(0))

    print("\n共 %d 帧，失败 %d，切不出帧的块 %d" % (nframe, nbad, nlost))
    return 0 if (0 == nbad and 0 == nlost and good) else 1


if __name__ == "__main__":
    sys.exit(main())
