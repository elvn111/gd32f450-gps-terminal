"""Protocol conformance checks for the GD32F450 GPS terminal (frame format A6 6A … B7 7B).

Two jobs:
  1. rebuild the sample frames of GD32F450_串行接口协议规范.md from their field
     values and verify 帧头/帧尾/长度/CRC-16/MODBUS, so the document cannot drift
     away from the firmware;
  2. parse a document and validate every hex frame block it contains.

Usage:
    python check_frames.py                 # rebuild + self-check the samples
    python check_frames.py <doc.md> [...]  # also validate the document's frames

Only the standard library is used.
"""
import re
import struct
import sys
from pathlib import Path

HEAD = bytes([0xA6, 0x6A])
TAIL = bytes([0xB7, 0x7B])
UP_DATA = 0x90
SUB_FIX = 0x01
SUB_INFO = 0x02

N_FIX = 26
N_INFO = 34
MAX_N = 256


def crc16_modbus(data):
    """poly=0x8005 (reflected 0xA001), init=0xFFFF, refin/refout=true, xorout=0"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(cmd, func, data):
    body = HEAD + bytes([cmd, func]) + struct.pack(">H", len(data)) + data
    return body + struct.pack("<H", crc16_modbus(body)) + TAIL


def fix_frame(seq, lat_e7, lon_e7, h_mm, head_cd, base_mm, sat, flag, t_ms):
    data = (struct.pack(">H", seq) + struct.pack(">iii", lat_e7, lon_e7, h_mm) +
            struct.pack(">H", head_cd) + struct.pack(">I", base_mm) +
            bytes([sat, flag]) + struct.pack(">I", t_ms))
    assert len(data) == N_FIX, len(data)
    return frame(UP_DATA, SUB_FIX, data)


def info_frame(seq, uptime_ms, version, tx_frames, tx_err, drop_latest,
               gps_ok, gps_err, out, auto_mode, rate, sat_min):
    data = (struct.pack(">H", seq) +
            struct.pack(">IIIIIII", uptime_ms, version, tx_frames, tx_err,
                        drop_latest, gps_ok, gps_err) +
            bytes([out, auto_mode, rate, sat_min]))
    assert len(data) == N_INFO, len(data)
    return frame(UP_DATA, SUB_INFO, data)


SAMPLES = [
    ("S1 一般定位（$GPYBM，VALID）",
     fix_frame(1, 301234567, 1207654321, 12345, 8721, 1234, 18, 0x03, 12345)),
    ("S2 未定位（$GPYBM）",
     fix_frame(2, 0, 0, 0, 0, 0, 3, 0x02, 20000)),
    ("S3 南半球、序号回绕（$GPNAV）",
     fix_frame(0xFFFF, -338688000, 1512093000, -5499, 35999, 0, 25, 0x01, 999999)),
    ("S4 极值边界（$GPNAV）",
     fix_frame(0, -338688000, -1512093000, -5499, 0, 0xFFFFFFFF, 0, 0x00, 0xFFFFFFFF)),
    ("I1 终端信息：正常运行",
     info_frame(101, 754321, 0x00010000, 1204, 0, 37, 2411, 3, 2, 1, 1, 4)),
    ("I2 终端信息：下行关闭",
     info_frame(0, 1500, 0x00010000, 0, 0, 0, 0, 0, 0, 0, 1, 4)),
]


def hexs(b, per_line=16):
    hx = [f"{x:02X}" for x in b]
    return "\n".join(" ".join(hx[i:i + per_line]) for i in range(0, len(hx), per_line))


def decode(b):
    """Human readable dump of one frame."""
    n = struct.unpack_from(">H", b, 4)[0]
    cmd, func = b[2], b[3]
    out = [f"cmd=0x{cmd:02X} func=0x{func:02X} N={n} total={len(b)}"]
    data = b[6:6 + n]
    if (cmd, func) == (UP_DATA, SUB_FIX):
        seq, = struct.unpack_from(">H", data, 0)
        lat, lon, h = struct.unpack_from(">iii", data, 2)
        head, = struct.unpack_from(">H", data, 14)
        base, = struct.unpack_from(">I", data, 16)
        t, = struct.unpack_from(">I", data, 22)
        out.append(f"seq={seq} lat={lat * 1e-7:.7f} lon={lon * 1e-7:.7f} h={h / 1000:.3f}m "
                   f"head={head / 100:.2f}deg base={base / 1000:.3f}m "
                   f"sat={data[20]} flag=0x{data[21]:02X} t_ms={t}")
    elif (cmd, func) == (UP_DATA, SUB_INFO):
        seq, = struct.unpack_from(">H", data, 0)
        up, ver, txf, txe, dr, ok, err = struct.unpack_from(">IIIIIII", data, 2)
        out.append(f"seq={seq} uptime={up}ms version={ver >> 16}.{(ver >> 8) & 0xFF}.{ver & 0xFF} "
                   f"tx={txf} tx_err={txe} drop={dr} gps_ok={ok} gps_err={err} "
                   f"out={data[30]} auto={data[31]} rate={data[32]} sat_min={data[33]}")
    return " | ".join(out)


def validate(b):
    """Return a list of problems for one frame."""
    problems = []
    if len(b) < 10:
        return ["过短（最短 10 字节）"]
    if b[:2] != HEAD:
        problems.append("帧头不是 A6 6A")
    if b[-2:] != TAIL:
        problems.append("帧尾不是 B7 7B")
    n = struct.unpack_from(">H", b, 4)[0]
    if n > MAX_N:
        problems.append(f"N={n} 超限")
    if len(b) != n + 10:
        problems.append(f"长度不符: {len(b)} != N+10 ({n + 10})")
    else:
        want = struct.unpack_from("<H", b, 6 + n)[0]
        if crc16_modbus(b[:6 + n]) != want:
            problems.append(f"CRC 错: 0x{want:04X}")
    return problems


def main():
    rc = 0
    got = crc16_modbus(b"123456789")
    print("== 自检：CRC-16/MODBUS 标准向量 ==")
    print(f"   crc16('123456789') = 0x{got:04X} (期望 0x4B37) {'OK' if got == 0x4B37 else 'FAIL'}")
    if got != 0x4B37:
        rc = 1

    print("\n== 样例帧（可按此回归接收端）==")
    known = {}
    for name, f in SAMPLES:
        problems = validate(f)
        known[f] = name
        print(f"-- {name}: {len(f)} 字节, {'OK' if not problems else 'FAIL ' + '; '.join(problems)}")
        print(hexs(f))
        print("   " + decode(f))
        if problems:
            rc = 1

    docs = sys.argv[1:]
    if not docs:
        here = Path(__file__).resolve().parent
        for cand in sorted((here / ".." / ".." / "Template").glob("*协议规范*.md")):
            docs.append(str(cand))

    for doc in docs:
        path = Path(doc)
        text = path.read_text(encoding="utf-8")
        blocks = re.findall(r"```\n((?:[0-9A-F]{2}[ \n])+[0-9A-F]{2})\n```", text)
        print(f"\n== 文档校验 {path.name}：{len(blocks)} 个帧块 ==")
        for i, blk in enumerate(blocks, 1):
            b = bytes(int(x, 16) for x in blk.split())
            problems = validate(b)
            name = known.get(b, "(不在样例集中)")
            print(f"   {i:2d}. {len(b):3d} B  {'OK' if not problems else 'FAIL ' + '; '.join(problems)}   {name}")
            if problems:
                rc = 1

    print("\nresult:", "PASS" if rc == 0 else "FAIL")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
