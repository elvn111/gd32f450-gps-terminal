# -*- coding: utf-8 -*-
"""生成/校验协议文档里要用的所有样例帧。

规则同固件：A6 6A | CMD | FUNC | N(小端) | DATA | CRC16/MODBUS(小端) | B7 7B
所有字段（长度、数据段数值）都是小端，CRC 计算范围 = 帧头起到数据段结束（6+N）。
"""
import struct
import sys

HEAD = bytes([0xA6, 0x6A])
TAIL = bytes([0xB7, 0x7B])


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build(cmd, func, data=b""):
    body = HEAD + bytes([cmd, func]) + struct.pack(">H", len(data)) + data
    return body + struct.pack("<H", crc16_modbus(body)) + TAIL


def check(raw):
    """返回 (ok, 说明)"""
    if len(raw) < 10:
        return False, "长度不足"
    if raw[0:2] != HEAD:
        return False, "帧头错"
    if raw[-2:] != TAIL:
        return False, "帧尾错"
    n = struct.unpack_from(">H", raw, 4)[0]
    if len(raw) != 10 + n:
        return False, "N=%d 与实长 %d 不符" % (n, len(raw))
    c = struct.unpack_from("<H", raw, 6 + n)[0]
    if c != crc16_modbus(raw[:6 + n]):
        return False, "CRC 错(帧内 0x%04X, 算得 0x%04X)" % (c, crc16_modbus(raw[:6 + n]))
    return True, "cmd=0x%02X func=0x%02X N=%d CRC=0x%04X" % (raw[2], raw[3], n, c)


def hx(raw, per=16):
    s = " ".join("%02X" % b for b in raw)
    return " ".join(s.split(" ")[i:i + per] for i in range(0, len(s.split(" ")), per))
    # 占位，实际下面用 wrap()


def wrap(raw, per=16):
    parts = ["%02X" % b for b in raw]
    return "\n".join(" ".join(parts[i:i + per]) for i in range(0, len(parts), per))


def u16(v):
    return struct.pack(">H", v)


def u32(v):
    return struct.pack(">I", v)


def i32(v):
    return struct.pack(">i", v)


FIX24 = (i32(int(30.1234567 * 1e7)) + i32(int(120.7654321 * 1e7)) + i32(12345)
         + u16(8721) + u32(1234) + bytes([18, 0x03]) + u32(754321))
FIX24_NOFIX = (i32(0) + i32(0) + i32(0) + u16(0) + u32(0) + bytes([0, 0x00]) + u32(754322))
INFO32 = (u32(754321) + u32(0x00010000) + u32(1204) + u32(0) + u32(37)
          + u32(2411) + u32(3) + bytes([2, 1, 20, 4]))
STAT26 = (u32(48641) + u16(2103) + u16(96) + u16(4)
          + u32(2411) + u32(3) + u32(1204) + u16(0) + u16(37))

CASES = [
    # ---- 2.1 数据上报 ----
    ("R1 定位数据上报(0x90/0x01)", 0x90, 0x01, u16(1204) + FIX24),
    ("R2 终端信息上报(0x90/0x02)", 0x90, 0x02, u16(1205) + INFO32),
    ("R3 定位数据上报-尚未定位(0x90/0x01)", 0x90, 0x01, u16(1206) + FIX24_NOFIX),
    ("R4 统计信息上报(0x90/0x03)", 0x90, 0x03, u16(1207) + STAT26),
    # ---- 2.2 系统信息 ----
    ("Q1 读取版本请求(0x01/0x01)", 0x01, 0x01, bytes([0x07])),
    ("A1 读取版本应答(0x81/0x01)", 0x81, 0x01, bytes([0x00, 0x07]) + u32(0x00010000)),
    ("Q2 读取配置项请求(0x01/0x02)", 0x01, 0x02, bytes([0x08, 0x06])),
    ("A2 读取配置项应答(0x81/0x02)", 0x81, 0x02, bytes([0x00, 0x08]) + u32(2)),
    ("Q3 设置配置项请求(0x01/0x03)", 0x01, 0x03, bytes([0x09, 0x06]) + u32(2)),
    ("A3 设置配置项应答(0x81/0x03)", 0x81, 0x03, bytes([0x00, 0x09])),
    ("Q4 保存配置请求(0x01/0x04)", 0x01, 0x04, bytes([0x0B])),
    ("A4 保存配置应答(0x81/0x04)", 0x81, 0x04, bytes([0x00, 0x0B])),
    ("Q5 读取已存配置请求(0x01/0x05)", 0x01, 0x05, bytes([0x0C])),
    ("A5 读取已存配置应答(0x81/0x05)", 0x81, 0x05, bytes([0x00, 0x0C])),
    ("Q6 恢复默认配置请求(0x01/0x06)", 0x01, 0x06, bytes([0x0D, 0xA5, 0x5A])),
    ("A6 恢复默认配置应答(0x81/0x06)", 0x81, 0x06, bytes([0x00, 0x0D])),
    ("Q7 系统重启请求(0x01/0x07)", 0x01, 0x07, bytes([0x0C, 0xA5, 0x5A])),
    ("Q8 清除统计请求(0x01/0x08)", 0x01, 0x08, bytes([0x0E])),
    ("A8 清除统计应答(0x81/0x08)", 0x81, 0x08, bytes([0x00, 0x0E])),
    ("Q9 读取统计请求(0x01/0x09)", 0x01, 0x09, bytes([0x0F])),
    ("A9 读取统计应答(0x81/0x09)", 0x81, 0x09, bytes([0x00, 0x0F]) + STAT26),
    # ---- 2.3 数据链路 ----
    ("Q10 设置下行请求(0x02/0x01)", 0x02, 0x01, bytes([0x0A, 2, 1, 20])),
    ("A10 设置下行应答(0x82/0x01)", 0x82, 0x01, bytes([0x00, 0x0A])),
    ("Q11 立即发送一帧请求(0x02/0x02)", 0x02, 0x02, bytes([0x10])),
    ("A11 立即发送一帧应答(0x82/0x02)", 0x82, 0x02, bytes([0x00, 0x10])),
    ("Q12 发送终端信息帧请求(0x02/0x03)", 0x02, 0x03, bytes([0x11])),
    ("A12 发送终端信息帧应答(0x82/0x03)", 0x82, 0x03, bytes([0x00, 0x11])),
    ("Q13 读取最近定位请求(0x02/0x04)", 0x02, 0x04, bytes([0x12])),
    ("A13 读取最近定位应答(0x82/0x04)", 0x82, 0x04, bytes([0x00, 0x12]) + FIX24),
    ("Q14 桥模式请求(0x02/0x05)", 0x02, 0x05, bytes([0x13, 1])),
    ("A14 桥模式应答(0x82/0x05)", 0x82, 0x05, bytes([0x00, 0x13])),
    ("Q15 恢复默认发送请求(0x02/0x06)", 0x02, 0x06, bytes([0x01])),
    ("A15 恢复默认发送应答(0x82/0x06)", 0x82, 0x06, bytes([0x00, 0x01])),
    # ---- 2.4 模块 ----
    ("Q16 模块命令透传请求(0x03/0x01)", 0x03, 0x01,
     bytes([0x0D]) + b"log com1 gpnav ontime 1"),
    ("A16 模块命令透传应答(0x83/0x01)", 0x83, 0x01, bytes([0x00, 0x0D])),
    ("Q17 硬复位模块请求(0x03/0x02)", 0x03, 0x02, bytes([0x0E])),
    ("A17 硬复位模块应答(0x83/0x02)", 0x83, 0x02, bytes([0x00, 0x0E])),
    ("Q18 重发预设请求(0x03/0x03)", 0x03, 0x03, bytes([0x0F])),
    ("A18 重发预设应答(0x83/0x03)", 0x83, 0x03, bytes([0x00, 0x0F])),
    ("Q19 模块工厂复位请求(0x03/0x04)", 0x03, 0x04, bytes([0x10, 0xA5, 0x5A])),
    ("A19 模块工厂复位应答(0x83/0x04)", 0x83, 0x04, bytes([0x00, 0x10])),
    # ---- 2.5 事件 ----
    ("E1 定位丢失事件(0xA0/0x01)", 0xA0, 0x01, u16(1300) + u32(900000)),
    ("E2 模块无响应事件(0xA0/0x02)", 0xA0, 0x02, u16(1301) + u32(906000)),
    ("E3 配置变更事件(0xA0/0x03)", 0xA0, 0x03, u16(1302) + bytes([0x06]) + u32(2)),
    ("E4 错误应答-未知功能码(0x81/0x7F)", 0x81, 0x7F, bytes([0x01, 0x0F])),
    ("E5 错误应答-参数越界(0x81/0x03)", 0x81, 0x03, bytes([0x02, 0x09])),
    ("E6 错误应答-缺确认字(0x81/0x07)", 0x81, 0x07, bytes([0x06, 0x0C])),
]

out = []
for name, cmd, func, data in CASES:
    raw = build(cmd, func, data)
    ok, info = check(raw)
    out.append("== %s  (%d B)%s" % (name, len(raw), "" if ok else "   <<< 校验失败: " + info))
    out.append(wrap(raw))
print("\n".join(out))
