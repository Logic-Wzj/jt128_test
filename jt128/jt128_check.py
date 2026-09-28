#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JT128（禾赛 128 线机械式激光雷达）裸 UDP 点云验证工具

不依赖 ROS、不依赖厂商上位机，直接收 192.168.1.201 -> 255.255.255.255:2368 的
点云数据包，校验包结构并按秒统计：包率 / 带宽 / 丢包 / 转速 / 帧率 / 最近点距离。

字段偏移依据《JT128 用户手册 J01-zh-260720》3.1 数据格式：
  UDP 数据 1100 字节 = 包头 6 + 数据头 6 + 数据主体 1032 + 数据尾 56
  数据主体 = Azimuth1(2) + Block1(512, 128通道×4字节) + Azimuth2(2) + Block2(512) + CRC(4)
  每通道 4 字节 = Distance(u16, ×Dis Unit) + Reflectivity(u8, 1%) + Confidence(u8)

用法:
  自检(不需要雷达):      python3 jt128_check.py --selftest
  实时抓包统计:          python3 jt128_check.py                # 需要能收广播包，普通用户即可
  指定端口/时长:          python3 jt128_check.py --port 2368 --seconds 30
  保存原始包(供 PandarView 之外离线分析): python3 jt128_check.py --dump /tmp/jt128.bin
"""

import argparse
import socket
import struct
import sys
import time

PKT_LEN = 1100           # UDP 数据长度
ETH_TAIL = 4             # 以太网帧尾 FCS，tcpdump/Wireshark 一般不计入
DIS_UNIT_MM = 4          # 数据头 Dis Unit 固定 0x04 = 4 mm

RETURN_MODE = {
    0x33: "第一 First",
    0x37: "最强 Strongest",
    0x38: "最后 Last",
    0x39: "最后及最强 Last&Strongest(默认)",
    0x3B: "最后及第一 Last&First",
    0x3C: "第一及最强 First&Strongest",
}
WORKING_MODE = {0: "运行", 1: "待机"}

TAIL_OFF = 1044          # 数据尾起始偏移（UDP 内）


def parse_packet(pkt: bytes) -> dict:
    """解析一个 1100 字节的点云 UDP 数据，返回字段字典（含首块距离样本）。"""
    if len(pkt) != PKT_LEN:
        raise ValueError("包长 %d != %d" % (len(pkt), PKT_LEN))
    if pkt[0] != 0xEE or pkt[1] != 0xFF:
        raise ValueError("包头标志错误: %02X %02X (应为 EE FF)" % (pkt[0], pkt[1]))

    d = {
        "proto": "%d.%d" % (pkt[2], pkt[3]),
        "channels": pkt[6],
        "blocks": pkt[7],
        "dis_unit": pkt[9],
        "return_num": pkt[10],
        "flags": pkt[11],
        "azimuth1": struct.unpack_from("<H", pkt, 12)[0] * 0.01,
        "azimuth2": struct.unpack_from("<H", pkt, 526)[0] * 0.01,
    }
    t = TAIL_OFF
    d["working"] = pkt[t + 11]
    d["return_mode"] = pkt[t + 12]
    d["motor_rpm"] = struct.unpack_from("<H", pkt, t + 13)[0] * 0.1
    d["utc"] = struct.unpack_from("<6B", pkt, t + 15)
    d["utc_us"] = struct.unpack_from("<I", pkt, t + 21)[0]
    d["factory"] = pkt[t + 25]
    d["seq"] = struct.unpack_from("<I", pkt, t + 26)[0]
    d["imu_temp"] = struct.unpack_from("<h", pkt, t + 30)[0] * 0.01
    d["imu_acc_x"] = struct.unpack_from("<h", pkt, t + 40)[0]
    d["imu_gyro_x"] = struct.unpack_from("<h", pkt, t + 46)[0]

    # 取数据块 1 的通道距离（用于粗看最近/最远点，单位 mm）
    unit = d["dis_unit"] or DIS_UNIT_MM
    dists = []
    for ch in range(128):
        off = 14 + ch * 4
        raw = struct.unpack_from("<H", pkt, off)[0]
        if raw:
            dists.append(raw * unit)
    d["dist_min_mm"] = min(dists) if dists else 0
    d["dist_max_mm"] = max(dists) if dists else 0
    d["valid_pts_block1"] = len(dists)
    return d


def build_selftest_packet(seq=1, azimuth=123.45, motor_rpm=600.0, ret_mode=0x39):
    """构造一个合法包，用于无硬件时验证解析逻辑。"""
    pkt = bytearray(PKT_LEN)
    pkt[0], pkt[1], pkt[2], pkt[3] = 0xEE, 0xFF, 0x01, 0x04
    pkt[6], pkt[7], pkt[8], pkt[9] = 0x80, 0x02, 0x00, DIS_UNIT_MM
    pkt[10] = 0x02 if ret_mode != 0x38 else 0x01   # Return Num
    pkt[11] = 0x06                                  # IMU + UDP Sequence
    struct.pack_into("<H", pkt, 12, int(azimuth / 0.01))
    struct.pack_into("<H", pkt, 526, int(azimuth / 0.01))
    for ch in range(128):                            # 每通道给个 1~2 m 的假距离
        off = 14 + ch * 4
        struct.pack_into("<H", pkt, off, 250 + ch)   # ×4mm => 1.0m 起
        pkt[off + 2] = 100                           # 反射率 100%
        pkt[off + 3] = 0
    b1 = pkt[14:14 + 512]
    b2 = pkt[528:528 + 512]
    struct.pack_into("<I", pkt, 1040, 0)             # 主体 CRC（自检不校验）
    t = TAIL_OFF
    pkt[t + 11] = 0                                  # 运行
    pkt[t + 12] = ret_mode
    struct.pack_into("<H", pkt, t + 13, int(motor_rpm / 0.1))
    struct.pack_into("<6B", pkt, t + 15, 126, 9, 21, 5, 8, 0)
    struct.pack_into("<I", pkt, t + 21, 123456)
    pkt[t + 25] = 0x42
    struct.pack_into("<I", pkt, t + 26, seq)
    struct.pack_into("<h", pkt, t + 30, 4212)        # 42.12 °C
    return bytes(pkt), b1, b2


def selftest():
    pkt, _, _ = build_selftest_packet()
    d = parse_packet(pkt)
    print("[selftest] 解析结果:")
    for k in ("proto", "channels", "blocks", "dis_unit", "return_num",
              "azimuth1", "azimuth2", "working", "motor_rpm", "seq",
              "factory", "imu_temp", "dist_min_mm", "dist_max_mm",
              "valid_pts_block1"):
        print("   %-16s = %s" % (k, d[k]))
    assert d["channels"] == 128 and d["blocks"] == 2
    assert abs(d["azimuth1"] - 123.45) < 0.02
    assert abs(d["motor_rpm"] - 600.0) < 0.05
    assert d["return_mode"] in RETURN_MODE
    print("[selftest] 字段偏移与解析逻辑 OK -> %s / %s" %
          (WORKING_MODE.get(d["working"], "?"), RETURN_MODE[d["return_mode"]]))
    return 0


def run(port, iface, seconds, dump_path, quiet):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    except OSError:
        pass
    if iface:
        s.setsockopt(socket.SOL_SOCKET, 25, iface.encode() + b"\0")  # SO_BINDTODEVICE
    s.bind(("", port))
    s.settimeout(2.0)

    print("监听 UDP :%d 中…（10 Hz 单回波约 4500 包/s，双回波（默认）约 9000 包/s，"
          "Ctrl+C 结束）" % port)
    dump = open(dump_path, "wb") if dump_path else None

    t0 = time.time()
    win_start = t0
    pkts = bytes_ = bad = 0
    last_seq = None
    lost = 0
    revs = 0
    last_az = None
    first = None
    frames = 0
    got_any = False

    try:
        while True:
            now = time.time()
            if seconds and now - t0 >= seconds:
                break
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                if not got_any:
                    print("!! 2 秒内没收到任何包：检查 IP/网线/供电，"
                          "或用 sudo tcpdump -i %s -n udp port %d 看物理链路" % (iface or "网卡", port))
                continue
            got_any = True
            if dump:
                dump.write(struct.pack("<I", len(data)) + data)
            pkts += 1
            bytes_ += len(data) + 42 + ETH_TAIL
            try:
                d = parse_packet(data)
            except ValueError as e:
                bad += 1
                if bad <= 5:
                    print("!! 非法包来自 %s: %s" % (addr[0], e))
                continue
            if first is None:
                first = (addr[0], d)
            # 丢包（UDP 序号）
            if last_seq is not None:
                gap = (d["seq"] - last_seq - 1) & 0xFFFFFFFF
                if 0 < gap < 100000:
                    lost += gap
            last_seq = d["seq"]
            # 圈数（方位角回绕）
            if last_az is not None and d["azimuth1"] < last_az - 180:
                revs += 1
                frames += 1
            last_az = d["azimuth1"]
            # 每秒打印
            now = time.time()
            if now - win_start >= 1.0:
                dt = now - win_start
                pps = pkts / dt
                mbps = bytes_ * 8 / dt / 1e6
                loss_pct = 100.0 * lost / max(lost + pkts, 1)
                print("%s | %7.0f 包/s | %6.2f Mbps | 帧率 %4.1f Hz | 转速 %6.1f RPM | "
                      "丢包 %5.2f%% | %s | %s | 最近点 %.2f m | IMU %.1f°C" % (
                          time.strftime("%H:%M:%S"), pps, mbps, revs / dt, d["motor_rpm"],
                          loss_pct, WORKING_MODE.get(d["working"], d["working"]),
                          RETURN_MODE.get(d["return_mode"], hex(d["return_mode"])),
                          d["dist_min_mm"] / 1000.0, d["imu_temp"]))
                if bad:
                    print("   （本秒累计非法包 %d 个）" % bad)
                pkts = bytes_ = 0
                revs = 0
                win_start = now
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        if dump:
            dump.close()
            print("原始包已保存到 %s（格式：4 字节长度 + 包内容）" % dump_path)

    if first:
        print("\n首包来源 IP: %s，协议版本 %s，通道数 %d，块数 %d，距离单位 %d mm，"
              "回波数 %d，Flags 0x%02X，工厂信息 0x%02X" % (
                  first[0], first[1]["proto"], first[1]["channels"], first[1]["blocks"],
                  first[1]["dis_unit"], first[1]["return_num"], first[1]["flags"],
                  first[1]["factory"]))
    return 0 if got_any else 1


def main():
    ap = argparse.ArgumentParser(description="JT128 裸 UDP 点云验证工具")
    ap.add_argument("--port", type=int, default=2368, help="点云 UDP 端口（默认 2368）")
    ap.add_argument("--iface", default=None, help="只在该网卡收包，如 enp131s0")
    ap.add_argument("--seconds", type=int, default=0, help="统计多少秒后退出，0=一直跑")
    ap.add_argument("--dump", default=None, help="把收到的原始包保存到文件")
    ap.add_argument("--quiet", action="store_true", help="安静模式")
    ap.add_argument("--selftest", action="store_true", help="构造假包验证解析逻辑，不需要雷达")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    return run(args.port, args.iface, args.seconds, args.dump, args.quiet)


if __name__ == "__main__":
    sys.exit(main())
