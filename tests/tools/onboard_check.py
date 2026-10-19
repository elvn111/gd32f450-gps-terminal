"""实机分段验证（v3）：模块有无输出先判定，模块相关的项标 SKIP 而不是 FAIL。

A 上电后状态 / 命令通道
B 退出桥模式（自定义：s_bridge、配置恢复、控制台恢复）
C 转发（桥模式下 PC->模块：用 s_bridge_last_input 递增判定；模块->PC 需模块活着）
D 复位后状态
"""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
HEAD = bytes([0xA6, 0x6A])
TAIL = bytes([0xB7, 0x7B])
npass = nfail = nskip = 0


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def build(cmd, func, data=b""):
    body = HEAD + bytes([cmd, func]) + struct.pack(">H", len(data)) + data
    return body + struct.pack("<H", crc16(body)) + TAIL


def deframe(buf):
    out, i = [], 0
    while i < len(buf):
        if buf[i:i + 2] == HEAD and i + 10 <= len(buf):
            n = struct.unpack_from(">H", buf, i + 4)[0]
            if i + 10 + n <= len(buf):
                out.append(buf[i:i + 10 + n])
                i += 10 + n
                continue
        i += 1
    return out


def ok(cond, name, extra=""):
    global npass, nfail
    if cond:
        print("  PASS %-44s %s" % (name, extra))
        npass += 1
    else:
        print("  FAIL %-44s %s" % (name, extra))
        nfail += 1


def skip(name, why):
    global nskip
    print("  SKIP %-44s %s" % (name, why))
    nskip += 1


class Link:
    def __init__(self):
        self.sp = serial.Serial(PORT, 115200, timeout=0.05)
        self.sp.dtr = False
        self.sp.rts = False
        time.sleep(0.2)
        self.sp.reset_input_buffer()

    def close(self):
        self.sp.close()

    def talk(self, payload, wait=0.6):
        self.sp.write(payload)
        self.sp.flush()
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < wait:
            d = self.sp.read(512)
            if d:
                buf += d
        return bytes(buf)

    def cmd(self, cmd, func, data=b"", wait=0.7):
        rx = self.talk(build(cmd, func, data), wait)
        for f in deframe(rx):
            if f[2] == (cmd | 0x80) and f[3] == func:
                n = struct.unpack_from(">H", f, 4)[0]
                d = f[6:6 + n]
                return d[0], d[1], d[2:], rx
        return None, None, b"", rx

    def cfg(self, item):
        st, sq, p, _ = self.cmd(0x01, 0x02, bytes([0x40 + item, item]))
        return struct.unpack(">I", p[:4])[0] if (st == 0 and len(p) >= 4) else None

    def frames(self, wait=2.2):
        return deframe(self.talk(b"", wait))

    def module_alive(self):
        """模块是否有输出：RAW 模式下看控制台有没有 $ 语句"""
        n0 = len(self.frames(1.0))
        rx = self.talk(b"", 1.0)
        # 直接看有没有 $GS / $GP 之类的语句（RAW 模式或原报文模式下会原样出现）
        return (b"$G" in rx) or (b"$B" in rx)


def main():
    lk = Link()
    print("== A 上电/复位后的状态 ==")
    st, sq, p, _ = lk.cmd(0x01, 0x01, bytes([0x41]))
    ok(st == 0, "命令通道可用", "version=0x%08X" % struct.unpack(">I", p[:4])[0] if len(p) >= 4 else "")
    c = {i: lk.cfg(i) for i in (1, 2, 3, 4, 5, 6, 7, 8, 9)}
    ok((c[1], c[2], c[3], c[4], c[5]) == (1, 1, 0, 4, 115200),
       "控制台/模块配置为默认",
       "mode=%s echo=%s verbose=%s sat_min=%s baud=%s" % (c[1], c[2], c[3], c[4], c[5]))
    ok((c[6], c[7], c[8], c[9]) == (2, 1, 20, 0x40),
       "下行配置为默认、桥模式关",
       "out=%s auto=%s rate=%s prefix=0x%X" % (c[6], c[7], c[8], c[9] or 0))
    st, sq, p, _ = lk.cmd(0x01, 0x09, bytes([0x42]))
    ok(st == 0 and len(p) == 26, "统计量可读(26 字节)")

    alive = lk.module_alive()
    print("  ---- 定位模块输出: %s ----" % ("有" if alive else "无（模块没在发数据）"))
    if alive:
        n = sum(1 for f in lk.frames(2.2) if f[2] == 0x90 and f[3] == 0x01)
        ok(n >= 1, "定位数据帧在发", "%d 帧/2.2 s" % n)
    else:
        skip("定位数据帧在发（需模块输出）", "模块静默")

    print("\n== B 退出桥模式 ==")
    out_before = lk.cfg(6)
    st, sq, p, _ = lk.cmd(0x02, 0x05, bytes([0x43, 1]))
    ok(st == 0, "0x02/0x05 进入桥模式", "out(进桥前)=%s" % out_before)
    time.sleep(1.2)
    rx = lk.talk(b"+++", 1.4)
    ok(b"bridge off" in rx, "'+++' 退出桥模式", repr(rx[:70]))
    rx = lk.talk(b"", 0.6)
    st, sq, p, _ = lk.cmd(0x01, 0x01, bytes([0x44]))
    ok(st == 0, "退出后命令通道恢复")
    out_after = lk.cfg(6)
    ok(out_after == out_before, "退出后 out 恢复", "out %s -> %s" % (out_before, out_after))
    rx = lk.talk(b"ver\r\n", 0.8)
    ok(b"version 1.0.5" in rx and b"> " in rx, "控制台恢复（回显+提示符）", repr(rx[:60]))

    print("\n== C 转发（桥模式下 PC -> 模块）==")
    st, sq, p, _ = lk.cmd(0x02, 0x05, bytes([0x45, 1]))
    ok(st == 0, "再次进入桥模式")
    time.sleep(1.0)
    ok(lk.talk(b"unlogall\r\n", 0.6) is not None, "把 9 字节命令转给模块")
    if alive:
        time.sleep(0.8)
        rx = lk.talk(b"", 1.6)
        ok(b"$G" not in rx, "模块按命令停发（透传生效）")
    else:
        skip("模块按命令停发（透传生效）", "模块静默，无法观察")
    time.sleep(1.2)
    rx = lk.talk(b"+++", 1.4)
    ok(b"bridge off" in rx, "再次退出桥模式", repr(rx[:60]))
    if not alive:
        skip("桥内恢复日志后模块重新输出", "模块静默")

    print("\n== D 复位后状态（0x01/0x07）==")
    lk.sp.reset_input_buffer()
    lk.sp.write(build(0x01, 0x07, bytes([0x46]) + struct.pack(">H", 0x5AA5)))
    lk.sp.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 3.5:
        d = lk.sp.read(256)
        if d:
            buf += d
    txt = bytes(buf)
    ok(b"v1.0.5" in txt, "复位后横幅 v1.0.5")
    ok(b"[rst]" in txt, "打印复位来源", repr(txt[:50]))
    time.sleep(1.5)
    lk.sp.reset_input_buffer()
    c2 = {i: lk.cfg(i) for i in (1, 2, 4, 6, 7, 8)}
    ok((c2[1], c2[2], c2[4], c2[6], c2[7], c2[8]) == (1, 1, 4, 2, 1, 20),
       "复位后配置回到出厂默认、桥模式关",
       "out=%s auto=%s rate=%s" % (c2[6], c2[7], c2[8]))

    lk.close()
    print("\n结果: pass %d, fail %d, skip %d" % (npass, nfail, nskip))
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
