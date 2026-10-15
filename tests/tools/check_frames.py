"""Protocol conformance checks for the GD32F450 GPS terminal.

Two jobs:
  1. rebuild every sample frame of GD32F450_串行接口协议规范.md from its field
     values and verify length + CRC-16/CCITT-FALSE, so the document cannot drift
     away from the firmware;
  2. parse the document and validate every hex block it contains.

Usage:
    python check_frames.py                 # rebuild + self-check the samples
    python check_frames.py <doc.md>        # also validate the document's blocks

Only the standard library is used.
"""
import re
import struct
import sys
from pathlib import Path

SOF = bytes([0xA5, 0x5A])
TYPE_FIX = 0x01
TYPE_INFO = 0x02
PAYLOAD_FIX = 24
PAYLOAD_INFO = 32


def crc16_ccitt_false(data):
    """poly=0x1021 init=0xFFFF refin=false refout=false xorout=0x0000"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(body):
    """SOF | LEN | body | CRC16(little endian)"""
    assert len(body) == 3 + (PAYLOAD_FIX if body[0] == TYPE_FIX else PAYLOAD_INFO)
    crc = crc16_ccitt_false(body)
    return SOF + bytes([len(body)]) + body + struct.pack("<H", crc)


def fix_frame(seq, lat_e7, lon_e7, h_mm, head_cd, base_mm, sat, flag, t_ms):
    body = (bytes([TYPE_FIX]) + struct.pack("<H", seq) +
            struct.pack("<iii", lat_e7, lon_e7, h_mm) +
            struct.pack("<H", head_cd) + struct.pack("<I", base_mm) +
            bytes([sat, flag]) + struct.pack("<I", t_ms))
    return frame(body)


def info_frame(seq, uptime_ms, version, tx_frames, tx_err, drop_latest,
               gps_ok, gps_err, out, auto_mode, rate, sat_min):
    body = (bytes([TYPE_INFO]) + struct.pack("<H", seq) +
            struct.pack("<IIIIIII", uptime_ms, version, tx_frames, tx_err,
                        drop_latest, gps_ok, gps_err) +
            bytes([out, auto_mode, rate, sat_min]))
    return frame(body)


# ---------------------------------------------------------------- sample set
SAMPLES = [
    ("S1 一般定位（$GPYBM，VALID）",
     fix_frame(1, 301234567, 1207654321, 12345, 8721, 1234, 18, 0x03, 12345)),
    ("S2 未定位（$GPYBM）",
     fix_frame(2, 0, 0, 0, 0, 0, 3, 0x02, 20000)),
    ("S3 南半球、序号回绕（$GPNAV）",
     fix_frame(0xFFFF, -338688000, 1512093000, -5499, 35999, 0, 25, 0x01, 999999)),
    ("S4 极值边界（$GPNAV）",
     fix_frame(0, -338688000, -1512093000, -5499, 0, 4294967295, 0, 0x00, 4294967295)),
    ("I1 终端信息：正常运行",
     info_frame(101, 754321, 0x010200, 1204, 0, 37, 2411, 3, 2, 1, 1, 4)),
    ("I2 终端信息：下行关闭",
     info_frame(0, 1500, 0x010200, 0, 0, 0, 0, 0, 0, 0, 1, 4)),
]


def hexs(b, per_line=16):
    hx = [f"{x:02X}" for x in b]
    return "\n".join(" ".join(hx[i:i + per_line]) for i in range(0, len(hx), per_line))


def decode(b):
    """Human readable dump of one frame."""
    ln, typ = b[2], b[3]
    seq, = struct.unpack_from("<H", b, 4)
    out = [f"LEN={ln} TYPE=0x{typ:02X} SEQ={seq} total={len(b)}"]
    if typ == TYPE_FIX:
        lat, lon, h = struct.unpack_from("<iii", b, 6)
        head, = struct.unpack_from("<H", b, 18)
        base, = struct.unpack_from("<I", b, 20)
        t, = struct.unpack_from("<I", b, 26)
        out.append(f"lat={lat * 1e-7:.7f} lon={lon * 1e-7:.7f} h={h / 1000:.3f}m "
                   f"head={head / 100:.2f}deg base={base / 1000:.3f}m "
                   f"sat={b[24]} flag=0x{b[25]:02X} t_ms={t}")
    elif typ == TYPE_INFO:
        up, ver, txf, txe, dr, ok, err = struct.unpack_from("<IIIIIII", b, 6)
        out.append(f"uptime={up}ms version={ver >> 16}.{(ver >> 8) & 0xFF}.{ver & 0xFF} "
                   f"tx={txf} tx_err={txe} drop={dr} gps_ok={ok} gps_err={err} "
                   f"out={b[34]} auto={b[35]} rate={b[36]} sat_min={b[37]}")
    return " | ".join(out)


def main():
    rc = 0
    print("== 自检：CRC 标准值 ==")
    got = crc16_ccitt_false(b"123456789")
    print(f"   crc16('123456789') = 0x{got:04X} (期望 0x29B1) {'OK' if got == 0x29B1 else 'FAIL'}")
    if got != 0x29B1:
        rc = 1

    print("\n== 样例帧（可按此回归接收端）==")
    known = {}
    for name, f in SAMPLES:
        body = f[3:3 + f[2]]
        ok = (len(f) == f[2] + 5) and (crc16_ccitt_false(body) ==
                                       struct.unpack_from("<H", f, 3 + f[2])[0])
        known[f] = name
        print(f"-- {name}: {len(f)} 字节, CRC {'OK' if ok else 'FAIL'}")
        print(hexs(f))
        print("   " + decode(f))
        if not ok:
            rc = 1

    docs = sys.argv[1:]
    if not docs:
        # 默认：在 ../../Template 下找协议规范文档
        here = Path(__file__).resolve().parent
        for cand in sorted((here / ".." / ".." / "Template").glob("*协议规范*.md")):
            docs.append(str(cand))

    for doc in docs:
        path = Path(doc)
        text = path.read_text(encoding="utf-8")
        blocks = re.findall(r"```\n((?:[0-9A-F]{2}[ \n])+[0-9A-F]{2})\n```", text)
        print(f"\n== 文档校验：{path.name}（{len(blocks)} 个二进制块）==")
        for i, blk in enumerate(blocks, 1):
            b = bytes(int(x, 16) for x in blk.split())
            ln = b[2]
            crc_ok = crc16_ccitt_false(b[3:3 + ln]) == struct.unpack_from("<H", b, 3 + ln)[0]
            len_ok = len(b) == ln + 5
            name = known.get(b, "(不在样例集中)")
            print(f"   block {i}: {len(b):3d} B  len={'OK' if len_ok else 'FAIL'}  "
                  f"crc={'OK' if crc_ok else 'FAIL'}  {name}")
            if not (len_ok and crc_ok):
                rc = 1

    print("\nresult:", "PASS" if rc == 0 else "FAIL")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
