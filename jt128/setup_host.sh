#!/usr/bin/env bash
# JT128 接收主机侧一键准备脚本（Ubuntu 22.04）
#
# 用法：
#   sudo ./setup_host.sh enp131s0              # 用默认 192.168.1.100
#   sudo ./setup_host.sh enp131s0 192.168.1.77 # 指定主机 IP（2~200 / 202~254）
#   sudo ./setup_host.sh enp131s0 --tcpdump    # 配好 IP 后直接抓包看有没有数据
#
# 雷达默认：源 IP 192.168.1.201，掩码 255.255.255.0，点云广播到 255.255.255.255:2368

set -u
IFACE="${1:-}"
HOST_IP="${2:-192.168.1.100}"
RADAR_IP=192.168.1.201
PORT=2368
DO_TCPDUMP=0
[[ "${2:-}" == "--tcpdump" ]] && { DO_TCPDUMP=1; HOST_IP=192.168.1.100; }
[[ "${3:-}" == "--tcpdump" ]] && DO_TCPDUMP=1

if [[ -z "$IFACE" ]]; then
  echo "用法: sudo $0 <网卡名> [主机IP] [--tcpdump]"
  echo "当前网卡:"; ip -br addr
  exit 1
fi
if [[ $EUID -ne 0 ]]; then echo "需要 root：请用 sudo 运行"; exit 1; fi
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "找不到网卡 $IFACE，现有网卡:"; ip -br addr; exit 1
fi

echo "== 1) 拉起链路 =="
ip link set "$IFACE" up
if command -v ethtool >/dev/null 2>&1; then
  ethtool "$IFACE" 2>/dev/null | grep -E "Speed|Duplex|Link detected" || true
else
  echo "(没装 ethtool，跳过链路速率显示；看 /sys/class/net/$IFACE/carrier 也能判断链路)"
fi

echo
echo "== 2) 添加 $HOST_IP/24 到 $IFACE（不影响原有 IP，可随时 ip addr del 删除）=="
if ip -4 addr show dev "$IFACE" | grep -q "inet $HOST_IP/"; then
  echo "已存在 $HOST_IP/24，跳过"
else
  ip addr add "$HOST_IP/24" dev "$IFACE"
  echo "已添加"
fi
ip -br addr show "$IFACE"

echo
echo "== 3) ping 雷达 $RADAR_IP =="
if ping -c 3 -W 1 "$RADAR_IP" >/dev/null 2>&1; then
  echo "ping 通 ✅（雷达在网、IP 正常）"
else
  echo "ping 不通 ⚠️  —— 雷达可能不回复 ICMP（部分固件如此），先别下结论，继续第 4 步看广播包。"
  echo "   若 Wireshark/tcpdump 也完全没包，再查：供电(9~32V/≥2.6A)、M8 线缆针脚、网线收发是否交叉。"
fi

echo
if [[ $DO_TCPDUMP -eq 1 ]]; then
  echo "== 4) 抓包 5 秒（看是否有 1146 字节、来自 $RADAR_IP 的 UDP 包）=="
  timeout 5 tcpdump -i "$IFACE" -n -c 5 "udp port $PORT" || true
  echo
  echo "接下来跑统计脚本："
  echo "  python3 $(dirname "$0")/jt128_check.py --iface $IFACE --seconds 20"
else
  echo "== 4) 下一步 =="
  echo "  抓包:  sudo tcpdump -i $IFACE -n -c 5 udp port $PORT"
  echo "  统计:  python3 $(dirname "$0")/jt128_check.py --iface $IFACE --seconds 20"
fi
