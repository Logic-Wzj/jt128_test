#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JT128 内存/资源占用观测工具

观测对象：接收雷达数据的进程（上位机 LidarUtilities、ROS 驱动节点、或测试脚本）
以及内核侧 UDP 收包缓冲区的堆积与丢包，用来回答两个问题：
  1. 跑 JT128 到底吃多少内存/CPU？
  2. 长时间跑会不会持续增长（泄漏）？

用法:
  python3 memwatch.py --name LidarUtilities --seconds 600
  python3 memwatch.py --pid 12345 --seconds 300
  python3 memwatch.py --name jt128_check --interval 5 --seconds 1800
  python3 memwatch.py --list          # 看当前有哪些候选进程

指标：
  RSS     进程实际占用物理内存；HWM 是进程生命周期内 RSS 峰值
  CPU     (utime+stime) 差分，单核百分比
  Threads / FDs     线程数与打开文件数（驱动类程序常见泄漏点）
  Recv-Q  该 UDP socket 内核收包队列里还没被应用读走的字节数
  UdpErr  内核 UDP 收包错误 / RcvbufErrors（应用读得太慢会涨）
"""

import argparse
import os
import re
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")


def _own_pids():
    """自己 + 所有祖先进程。避免把 memwatch 本身、或调用它的 shell（命令行里往往含关键字）
    当成观测目标 —— 这会让读数看起来"只有 11 MB"之类，结论完全错。"""
    pids, pid = set(), os.getpid()
    for _ in range(16):
        if pid <= 1:
            break
        pids.add(pid)
        try:
            stat = open("/proc/%d/stat" % pid).read()
            pid = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return pids


def find_procs(name):
    """按关键字匹配进程，返回 [(pid, rss_kb, comm, cmd)]，最可能是目标的排在最后。

    排序依据：可执行名(comm)命中优先于仅命令行命中；同档按 RSS 从大到小。
    """
    kw = name.lower()
    skip = _own_pids()
    hits = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) in skip:
            continue
        try:
            cmd = open("/proc/%s/cmdline" % pid, "rb").read().replace(b"\0", b" ").decode(errors="replace").strip()
            comm = open("/proc/%s/comm" % pid).read().strip()
        except OSError:
            continue
        if kw not in cmd.lower() and kw not in comm.lower():
            continue
        rss = 0
        try:
            for line in open("/proc/%s/status" % pid):
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])
                    break
        except OSError:
            pass
        hits.append((int(pid), rss, comm, cmd[:90], kw in comm.lower()))
    hits.sort(key=lambda h: (h[4], h[1]))
    return [(pid, rss, comm, cmd) for pid, rss, comm, cmd, _ in hits]


def proc_snapshot(pid):
    st = {}
    with open("/proc/%d/status" % pid) as f:
        for line in f:
            if line.startswith(("VmRSS:", "VmHWM:", "Threads:")):
                k, v = line.split(":", 1)
                st[k] = v.strip()
    with open("/proc/%d/stat" % pid) as f:
        fields = f.read().rsplit(") ", 1)[1].split()
    # rsplit 后第 11、12 个字段是 utime、stime（原 stat 的 14、15）
    st["utime"] = int(fields[11])
    st["stime"] = int(fields[12])
    try:
        st["fds"] = len(os.listdir("/proc/%d/fd" % pid))
    except OSError:
        st["fds"] = -1
    return st


def meminfo():
    d = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":", 1)
            d[k] = int(v.strip().split()[0])
    return d


def udp_snmp():
    """返回 (InDatagrams, InErrors, RcvbufErrors)"""
    out = {}
    section = None
    with open("/proc/net/snmp") as f:
        for line in f:
            k, _, rest = line.partition(":")
            if k.strip() == "Udp":
                vals = rest.split()
                if line.startswith("Udp:") and "InDatagrams" in line:
                    out["hdr"] = vals
                else:
                    out["val"] = vals
    try:
        i = out["hdr"].index("InDatagrams")
        return (int(out["val"][i]),
                int(out["val"][out["hdr"].index("InErrors")]),
                int(out["val"][out["hdr"].index("RcvbufErrors")]))
    except Exception:
        return (0, 0, 0)


def udp_recvq(port):
    """内核中该 UDP 端口 socket 的接收队列字节数（十六进制 -> 十进制）"""
    hexport = ":%04X" % port
    total = 0
    try:
        with open("/proc/net/udp") as f:
            next(f)
            for line in f:
                p = line.split()
                if len(p) > 5 and p[1].endswith(hexport):
                    tx, rx = p[4].split(":")
                    total += int(rx, 16)
    except OSError:
        pass
    return total


def fmt_mb(kb):
    return kb / 1024.0


def main():
    ap = argparse.ArgumentParser(description="进程内存/CPU 与内核 UDP 缓冲观测")
    ap.add_argument("--pid", type=int, help="直接指定进程号")
    ap.add_argument("--name", help="按命令行关键字匹配进程（取最新的一个）")
    ap.add_argument("--port", type=int, default=2368, help="观测的 UDP 端口，默认 2368")
    ap.add_argument("--interval", type=float, default=5.0, help="采样间隔秒，默认 5")
    ap.add_argument("--seconds", type=int, default=600, help="观测总时长，0=一直跑")
    ap.add_argument("--list", action="store_true", help="列出可能相关的进程后退出")
    args = ap.parse_args()

    if args.list:
        for kw in ("LidarUtilities", "hesai", "lidar", "jt128_check", "ros2", "rviz"):
            hits = find_procs(kw)
            if hits:
                print("[%s]" % kw)
                for pid, rss, comm, cmd in hits:
                    print("   %6d  %6.1f MB  %-28s %s" % (pid, rss / 1024.0, comm, cmd))
        return 0

    pid = args.pid
    if pid is None:
        if not args.name:
            print("需要 --pid 或 --name（或 --list 看候选）")
            return 1
        hits = find_procs(args.name)
        if not hits:
            print("没找到匹配 '%s' 的进程；用 --list 看当前候选" % args.name)
            return 1
        if len(hits) > 1:
            print("匹配到 %d 个进程（已排除 memwatch 自身及父进程）：" % len(hits))
            for p, r, c, cm in hits:
                print("   %6d  %6.1f MB  %-24s %s" % (p, r / 1024.0, c, cm))
        pid, rss, comm, cmd = hits[-1]
        print("观测进程: %d  %s  (RSS %.1f MB)  %s" % (pid, comm, rss / 1024.0, cmd))
    else:
        print("观测进程: %d" % pid)

    try:
        first = proc_snapshot(pid)
    except OSError as e:
        print("读不到进程 %d: %s" % (pid, e))
        return 1

    in_dgrams0, in_err0, rcvbuf0 = udp_snmp()
    samples = []
    t0 = time.time()
    last = first
    last_t = t0
    print("%-10s %8s %8s %7s %6s %6s %9s %9s" %
          ("时间", "RSS/MB", "峰值/MB", "CPU%", "线程", "FD", "Recv-Q/KB", "UdpErr"))
    try:
        while True:
            now = time.time()
            if args.seconds and now - t0 >= args.seconds:
                break
            time.sleep(args.interval)
            now = time.time()
            try:
                s = proc_snapshot(pid)
            except OSError:
                print("进程 %d 已退出" % pid)
                break
            rss = int(s["VmRSS"].split()[0])
            hwm = int(s["VmHWM"].split()[0])
            dt = max(now - last_t, 1e-6)
            cpu = 100.0 * ((s["utime"] - last["utime"]) + (s["stime"] - last["stime"])) / CLK_TCK / dt
            rq = udp_recvq(args.port)
            in_d, in_e, rc_e = udp_snmp()
            print("%-10s %8.1f %8.1f %7.1f %6s %6s %9.1f %9d" %
                  (time.strftime("%H:%M:%S"), fmt_mb(rss), fmt_mb(hwm), cpu,
                   s["Threads"], s["fds"], rq / 1024.0, rc_e - rcvbuf0))
            samples.append((now - t0, rss, cpu, s["fds"], rq))
            last, last_t = s, now
    except KeyboardInterrupt:
        pass

    if len(samples) >= 3:
        dur = samples[-1][0]
        rss0, rss1 = samples[0][1], samples[-1][1]
        slope = (rss1 - rss0) / max(dur, 1) / 1024.0 * 60.0   # MB/min
        peak = max(s[1] for s in samples)
        cpu_avg = sum(s[2] for s in samples) / len(samples)
        fd0, fd1 = samples[0][3], samples[-1][3]
        rq_peak = max(s[4] for s in samples)
        in_d, in_e, rc_e = udp_snmp()
        print("\n===== 汇总（观测 %.0f 秒，%d 个采样点）=====" % (dur, len(samples)))
        print("RSS 起/止      : %.1f -> %.1f MB   峰值 %.1f MB" % (fmt_mb(rss0), fmt_mb(rss1), fmt_mb(peak)))
        print("内存增长斜率   : %+.3f MB/min" % slope)
        print("平均 CPU       : %.1f %% (单核百分比)" % cpu_avg)
        print("FD 起/止       : %s -> %s" % (fd0, fd1))
        print("内核 Recv-Q 峰值: %.1f KB   （持续不降 = 应用读得不够快）" % (rq_peak / 1024.0))
        print("内核 UDP 增量  : InDatagrams +%d, InErrors +%d, RcvbufErrors +%d" %
              (in_d - in_dgrams0, in_e - in_err0, rc_e - rcvbuf0))
        print("\n判定:")
        print("  %s 内存：%s" % ("✅" if abs(slope) < 1.0 else "⚠️",
              "稳定，无泄漏迹象" if abs(slope) < 1.0 else "持续增长 %.2f MB/min，疑似泄漏或缓存无上限" % slope))
        print("  %s FD  ：%s" % ("✅" if fd1 <= fd0 + 2 else "⚠️",
              "稳定" if fd1 <= fd0 + 2 else "增长 %d 个，疑似句柄泄漏" % (fd1 - fd0)))
        print("  %s 内核缓冲：%s" % ("✅" if (rc_e - rcvbuf0) == 0 else "⚠️",
              "无 RcvbufErrors" if (rc_e - rcvbuf0) == 0 else
              "出现 %d 次 RcvbufErrors → 应用读取偏慢，需要加大 SO_RCVBUF 或降低点云带宽" % (rc_e - rcvbuf0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
