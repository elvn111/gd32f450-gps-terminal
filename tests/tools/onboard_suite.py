"""实机全方位测试套件：命令通道 + 收帧健壮性 + 桥模式 + 上电/复位状态。

用法: python onboard_suite.py [COM7]

分段：
  A 上电后状态：默认配置、桥模式关、命令通道、统计可读、定位帧（模块有输出才判）
  B 19 条命令逐一验证：状态码 / 回带序号 / 载荷长度与关键字段
  C 收帧健壮性：帧+文本同包、文本+帧同包、半帧分片、半帧超时重同步、坏 CRC 后正常帧
  D 桥模式：进入 / 帧在桥模式下不生效 / '+++' 退出 / 配置恢复 / 控制台恢复
  E 复位后状态：0x01/0x07 横幅 + [rst] + 配置回默认
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
        print("  PASS %-46s %s" % (name, extra))
        npass += 1
    else:
        print("  FAIL %-46s %s" % (name, extra))
        nfail += 1


def skip(name, why):
    global nskip
    print("  SKIP %-46s %s" % (name, why))
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

    def find(self, rx, cmd, func):
        for f in deframe(rx):
            if f[2] == (cmd | 0x80) and f[3] == func:
                n = struct.unpack_from(">H", f, 4)[0]
                d = f[6:6 + n]
                return d[0], d[1], d[2:]
        return None, None, b""

    def cmd(self, cmd, func, data=b"", wait=0.7):
        rx = self.talk(build(cmd, func, data), wait)
        st, sq, p = self.find(rx, cmd, func)
        return st, sq, p, rx

    def cfg(self, item):
        st, sq, p, _ = self.cmd(0x01, 0x02, bytes([0x40 + item, item]))
        return struct.unpack(">I", p[:4])[0] if (st == 0 and len(p) >= 4) else None

    def frames(self, wait=2.2):
        return deframe(self.talk(b"", wait))


def main():
    lk = Link()
    print("== A 上电后状态 ==")
    st, sq, p, _ = lk.cmd(0x01, 0x01, bytes([0x41]))
    ok(st == 0, "命令通道可用（0x01/0x01）",
       "version=0x%08X" % struct.unpack(">I", p[:4])[0] if len(p) >= 4 else "")
    ok(sq == 0x41, "回带序号正确")
    c = {i: lk.cfg(i) for i in range(1, 10)}
    ok((c[1], c[2], c[3], c[4], c[5]) == (1, 1, 0, 4, 115200), "控制台/模块配置默认",
       "mode=%s echo=%s verbose=%s sat_min=%s baud=%s" % (c[1], c[2], c[3], c[4], c[5]))
    ok((c[6], c[7], c[8], c[9]) == (2, 1, 20, 0x40), "下行配置默认、桥模式关",
       "out=%s auto=%s rate=%s prefix=0x%X" % (c[6], c[7], c[8], c[9] or 0))
    st, sq, p, _ = lk.cmd(0x01, 0x09, bytes([0x42]))
    ok(st == 0 and len(p) == 26, "统计量 26 字节")

    fr = lk.frames(2.2)
    nfix = sum(1 for f in fr if f[2] == 0x90 and f[3] == 0x01)
    ninfo = sum(1 for f in fr if f[2] == 0x90 and f[3] == 0x02)
    ok(ninfo >= 1, "终端信息帧在发（1 Hz 保活）", "%d 帧" % ninfo)
    alive = nfix >= 1
    if alive:
        ok(True, "定位数据帧在发", "%d 帧/2.2 s" % nfix)
    else:
        skip("定位数据帧在发", "定位模块无输出（硬件）")

    print("\n== B 19 条命令 ==")
    # 0x01/0x01 版本
    st, sq, p, _ = lk.cmd(0x01, 0x01, bytes([0x01]))
    ok(st == 0 and len(p) == 4 and struct.unpack(">I", p)[0] >= 0x00010000, "0x01/0x01 读取版本")
    # 0x01/0x02 读配置项
    st, sq, p, _ = lk.cmd(0x01, 0x02, bytes([0x02, 6]))
    ok(st == 0 and struct.unpack(">I", p[:4])[0] == c[6], "0x01/0x02 读取配置项")
    st, sq, p, _ = lk.cmd(0x01, 0x02, bytes([0x03, 99]))
    ok(st == 0x02, "0x01/0x02 不存在的配置项 -> BAD_PARAM")
    # 0x01/0x03 设置配置项
    st, sq, p, _ = lk.cmd(0x01, 0x03, bytes([0x04, 8]) + struct.pack(">I", 15))
    ok(st == 0 and lk.cfg(8) == 15, "0x01/0x03 设置 rate=15")
    st, sq, p, _ = lk.cmd(0x01, 0x03, bytes([0x05, 8]) + struct.pack(">I", 0))
    ok(st == 0x02, "0x01/0x03 rate=0 -> BAD_PARAM")
    st, sq, p, _ = lk.cmd(0x01, 0x03, bytes([0x06, 8]) + struct.pack(">I", 20))
    ok(st == 0, "0x01/0x03 恢复 rate=20")
    # 0x01/0x04 保存（本板 Flash 容量寄存器仅 512 KB，固件拒绝写 bank1）
    st, sq, p, _ = lk.cmd(0x01, 0x04, bytes([0x07]), 1.2)
    ok(st == 0x04, "0x01/0x04 保存 -> UNSUPPORTED（Flash 容量保护）", "status=0x%02X" % (st if st is not None else 0xFF))
    # 0x01/0x05 读取已保存
    st, sq, p, _ = lk.cmd(0x01, 0x05, bytes([0x08]), 1.0)
    ok(st in (0x08, 0x00), "0x01/0x05 读取已保存配置", "status=0x%02X" % (st if st is not None else 0xFF))
    # 0x01/0x06 恢复默认
    st, sq, p, _ = lk.cmd(0x01, 0x06, bytes([0x09]) + struct.pack(">H", 0x5AA5))
    ok(st == 0, "0x01/0x06 恢复默认（带确认字）")
    st, sq, p, _ = lk.cmd(0x01, 0x06, bytes([0x0A]) + b"\x00\x00")
    ok(st == 0x06, "0x01/0x06 无确认字 -> NEED_CONFIRM")
    st, sq, p, _ = lk.cmd(0x01, 0x03, bytes([0x0B, 6]) + struct.pack(">I", 2))
    # 0x01/0x08 清统计
    st, sq, p, _ = lk.cmd(0x01, 0x08, bytes([0x0C]))
    ok(st == 0, "0x01/0x08 清除统计")
    # 0x01/0x09 读统计
    st, sq, p, _ = lk.cmd(0x01, 0x09, bytes([0x0D]))
    ok(st == 0 and len(p) == 26 and struct.unpack(">I", p[0:4])[0] < 10000, "0x01/0x09 读统计（清后归零附近）")
    # 0x02/0x01 设置下行参数
    st, sq, p, _ = lk.cmd(0x02, 0x01, bytes([0x0E, 2, 1, 20]))
    ok(st == 0, "0x02/0x01 设置下行参数")
    st, sq, p, _ = lk.cmd(0x02, 0x01, bytes([0x0F, 2, 9, 20]))
    ok(st == 0x02, "0x02/0x01 auto=9 -> BAD_PARAM")
    # 0x02/0x02 立即发一帧
    st, sq, p, rx = lk.cmd(0x02, 0x02, bytes([0x10]))
    ok(st in (0x00, 0x08), "0x02/0x02 立即发送一帧", "status=0x%02X" % (st if st is not None else 0xFF))
    # 0x02/0x03 发终端信息帧
    st, sq, p, rx = lk.cmd(0x02, 0x03, bytes([0x11]))
    ok(st == 0, "0x02/0x03 发终端信息帧")
    ok(any(f[2] == 0x90 and f[3] == 0x02 for f in deframe(rx)), "  并伴随 0x90/0x02 帧")
    # 0x02/0x04 读最近定位
    st, sq, p, _ = lk.cmd(0x02, 0x04, bytes([0x12]))
    ok(st in (0x00, 0x08) and (len(p) == 24 or st == 0x08), "0x02/0x04 读取最近定位",
       "status=0x%02X len=%d" % (st if st is not None else 0xFF, len(p)))
    # 0x02/0x06 恢复默认发送
    st, sq, p, _ = lk.cmd(0x02, 0x06, bytes([0x13]))
    ok(st == 0 and (lk.cfg(6), lk.cfg(7), lk.cfg(8)) == (2, 1, 20), "0x02/0x06 恢复默认发送")
    # 0x03/0x01 模块透传
    st, sq, p, _ = lk.cmd(0x03, 0x01, bytes([0x14]) + b"version", 0.9)
    ok(st == 0, "0x03/0x01 模块命令透传（应答 OK）")
    # 0x03/0x02 硬复位
    st, sq, p, _ = lk.cmd(0x03, 0x02, bytes([0x15]), 1.2)
    ok(st == 0, "0x03/0x02 硬复位模块（应答 OK）")
    # 0x03/0x03 重发预设
    st, sq, p, _ = lk.cmd(0x03, 0x03, bytes([0x16]), 0.9)
    ok(st == 0, "0x03/0x03 重发模块预设")
    # 0x03/0x04 工厂复位（确认字）
    st, sq, p, _ = lk.cmd(0x03, 0x04, bytes([0x17]) + b"\x12\x34")
    ok(st == 0x06, "0x03/0x04 错误确认字 -> NEED_CONFIRM")
    # 未知功能码 / 未知命令集
    st, sq, p, _ = lk.cmd(0x01, 0x7F, bytes([0x18]))
    ok(st == 0x01, "0x01/0x7F 未知功能码 -> UNKNOWN_FUNC")
    rx = lk.talk(build(0x05, 0x01, bytes([0x19])), 0.7)
    hit = [f for f in deframe(rx) if f[2] == 0x81 and f[3] == 0x01]
    stt = hit[0][6] if hit else None
    ok(stt == 0x01, "0x05/0x01 未知命令集 -> 0x81/0x01 UNKNOWN_FUNC",
       ("status=0x%02X" % stt) if stt is not None else "no reply")

    print("\n== C 收帧健壮性 ==")
    # C1: 帧 + 文本同包
    lk.sp.reset_input_buffer()
    lk.sp.write(build(0x01, 0x01, bytes([0x20, ])) + b"ver\r\n")
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    st, sq, p = lk.find(rx, 0x01, 0x01)
    ok(st == 0 and sq == 0x20, "帧+文本同一包：帧被正确解析")
    ok(b"version 1.0.5" in rx or b"[cli]" in rx, "  同一包的文本也进了控制台")
    # C2: 文本 + 帧同包
    lk.sp.reset_input_buffer()
    lk.sp.write(b"ver\r\n" + build(0x01, 0x01, bytes([0x21])))
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    st, sq, p = lk.find(rx, 0x01, 0x01)
    ok(st == 0 and sq == 0x21, "文本+帧同一包：帧也被正确解析")
    # C3: 半帧分片（两段写，间隔 40 ms < 100 ms 超时）
    lk.sp.reset_input_buffer()
    whole = build(0x01, 0x01, bytes([0x22]))
    lk.sp.write(whole[:5])
    lk.sp.flush()
    time.sleep(0.04)
    lk.sp.write(whole[5:])
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    st, sq, p = lk.find(rx, 0x01, 0x01)
    ok(st == 0 and sq == 0x22, "半帧跨包拼接（40 ms）")
    # C4: 半帧后超时（200 ms），再发整帧 -> 超时复位后仍能收
    lk.sp.reset_input_buffer()
    lk.sp.write(whole[:6])
    lk.sp.flush()
    time.sleep(0.25)
    lk.sp.write(build(0x01, 0x01, bytes([0x23])))
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    st, sq, p = lk.find(rx, 0x01, 0x01)
    ok(st == 0 and sq == 0x23, "半帧超时(200 ms)后重同步，后续帧正常")
    # C5: 坏 CRC 后紧跟正常帧
    lk.sp.reset_input_buffer()
    bad = bytearray(build(0x01, 0x01, bytes([0x24])))
    bad[7] ^= 0x5A
    lk.sp.write(bytes(bad) + build(0x01, 0x01, bytes([0x25])))
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    st, sq, p = lk.find(rx, 0x01, 0x01)
    ok(st == 0 and sq == 0x25, "坏 CRC 帧被丢弃，紧跟的正常帧照常执行")

    print("\n== D 桥模式 ==")
    out_before = lk.cfg(6)
    st, sq, p, _ = lk.cmd(0x02, 0x05, bytes([0x26, 1]))
    ok(st == 0, "0x02/0x05 进入桥模式", "out(进桥前)=%s" % out_before)
    time.sleep(1.0)
    # 桥模式下二进制帧不生效：发一帧，应没有任何应答
    lk.sp.reset_input_buffer()
    lk.sp.write(build(0x01, 0x01, bytes([0x27])))
    lk.sp.flush()
    time.sleep(0.8)
    rx = lk.sp.read(2048)
    ok(not any(f[2] == 0x81 for f in deframe(rx)), "桥模式下二进制命令不生效（按设计）")
    time.sleep(1.2)
    rx = lk.talk(b"+++", 1.4)
    ok(b"bridge off" in rx, "'+++' 退出桥模式")
    st, sq, p, _ = lk.cmd(0x01, 0x01, bytes([0x28]))
    ok(st == 0, "退出后命令通道恢复")
    ok(lk.cfg(6) == out_before, "退出后 out 恢复", "out %s -> %s" % (out_before, lk.cfg(6)))
    rx = lk.talk(b"ver\r\n", 0.8)
    ok(b"version 1.0.5" in rx, "控制台恢复（ver 有回显）")

    print("\n== E 复位后状态 ==")
    lk.sp.reset_input_buffer()
    lk.sp.write(build(0x01, 0x07, bytes([0x29]) + struct.pack(">H", 0x5AA5)))
    lk.sp.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 3.5:
        d = lk.sp.read(256)
        if d:
            buf += d
    txt = bytes(buf)
    ok(b"v1.0.5" in txt, "复位后横幅 v1.0.5")
    ok(b"[rst]" in txt, "打印复位来源")
    time.sleep(1.5)
    lk.sp.reset_input_buffer()
    c2 = {i: lk.cfg(i) for i in (1, 2, 4, 6, 7, 8)}
    ok((c2[1], c2[2], c2[4], c2[6], c2[7], c2[8]) == (1, 1, 4, 2, 1, 20),
       "复位后配置回默认、桥模式关",
       "out=%s auto=%s rate=%s" % (c2[6], c2[7], c2[8]))

    lk.close()
    print("\n结果: pass %d, fail %d, skip %d" % (npass, nfail, nskip))
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
