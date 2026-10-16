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


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def deframe(tokens):
    """按 A6 6A + 长度 N 把字节流切成帧，返回 (frame, leftover) 列表"""
    frames = []
    i = 0
    while i < len(tokens):
        if tokens[i] == 0xA6 and i + 5 < len(tokens) and tokens[i + 1] == 0x6A:
            n = tokens[i + 4] | (tokens[i + 5] << 8)
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
    nframe = nbad = 0
    for bi, b in enumerate(blocks, 1):
        toks = [int(x, 16) for x in re.findall(r"(?<![0-9A-Fa-f])[0-9A-Fa-f]{2}(?![0-9A-Fa-f])", b)]
        if not toks or toks[0] != 0xA6:
            continue
        for raw in deframe(toks):
            raw = bytes(raw)
            nframe += 1
            n = struct.unpack_from("<H", raw, 4)[0]
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

    print("\n共 %d 帧，失败 %d" % (nframe, nbad))
    return 0 if (0 == nbad and good) else 1


if __name__ == "__main__":
    sys.exit(main())
