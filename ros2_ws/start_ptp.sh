#!/bin/bash
# =============================================================================
# PTP 时间同步启动脚本 — 多 LiDAR 传感器
# =============================================================================
# 功能:
#   1. 检查 linuxptp 并安装
#   2. 为 LiDAR 网口启动 ptp4l (软件时间戳, enp3s0 无硬件 PTP 时钟)
#   3. 等待 PTP 锁定
#
# 当前状态:
#   - 宿主机为 PTP Grand Master（LiDAR 固件暂未开 PTP slave）
#   - LiDAR 固件开启 PTP 后，ptp4l 会自动协商切换为 slave
#
# 用法:
#   ./start_ptp.sh
#   PTP_IFACE=enp3s0 ./start_ptp.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PTP_CFG="${PTP_CFG:-${SCRIPT_DIR}/src/mapless_nav/config/ptp_lidar.cfg}"
PTP_IFACE="${PTP_IFACE:-enp3s0}"

run_sudo() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo -n "$@" 2>/dev/null || {
            echo "[PTP] ERROR: Need passwordless sudo for ptp4l."
            echo "  Run: sudo visudo -f /etc/sudoers.d/ptp4l"
            echo "  Add: $USER ALL=(ALL) NOPASSWD: /usr/sbin/ptp4l, /usr/bin/pkill"
            exit 1
        }
    fi
}

log()   { echo "[PTP] $*"; }
warn()  { echo "[PTP] WARN: $*" >&2; }
err()   { echo "[PTP] ERROR: $*" >&2; }

# ---------------------------------------------------------------------------
# 1. 检查 linuxptp
# ---------------------------------------------------------------------------
if ! command -v ptp4l &>/dev/null; then
    err "linuxptp 未安装，正在安装..."
    run_sudo apt-get update -qq && run_sudo apt-get install -y linuxptp
fi

# ---------------------------------------------------------------------------
# 2. 检查接口
# ---------------------------------------------------------------------------
if ! ip link show dev "$PTP_IFACE" &>/dev/null; then
    err "接口 $PTP_IFACE 不存在"
    exit 1
fi

if ! ip link show dev "$PTP_IFACE" | grep -q 'LOWER_UP'; then
    warn "$PTP_IFACE 无载波（网线未插？），仍尝试启动 PTP..."
fi

# ---------------------------------------------------------------------------
# 3. 清理旧 ptp4l
# ---------------------------------------------------------------------------
if pgrep -f "ptp4l.*-i $PTP_IFACE" &>/dev/null; then
    log "停止 $PTP_IFACE 上的旧 ptp4l 实例..."
    run_sudo pkill -f "ptp4l.*-i $PTP_IFACE" 2>/dev/null || true
    sleep 1
fi

# ---------------------------------------------------------------------------
# 4. 启动 ptp4l
# ---------------------------------------------------------------------------
log "启动 ptp4l on $PTP_IFACE (software timestamp, L2 transport)..."
log "配置: $PTP_CFG"

run_sudo ptp4l -f "$PTP_CFG" -i "$PTP_IFACE" -S -m &
PTP_PID=$!
log "ptp4l pid=$PTP_PID"

# ---------------------------------------------------------------------------
# 5. 等待 PTP 稳定
# ---------------------------------------------------------------------------
log "等待 PTP 协商（最多 30 秒）..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    # 检查 ptp4l 进程是否存活
    if ! kill -0 "$PTP_PID" 2>/dev/null; then
        err "ptp4l 进程异常退出"
        exit 1
    fi
    # slave 模式 = LiDAR 是 master（最理想情况）
    if pmc -u -b 0 "GET CURRENT_DATA_SET" 2>/dev/null | grep -q 'slave'; then
        log "PTP 已锁定为 SLAVE 模式（LiDAR 为 master）"
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    warn "PTP 锁超时 — 当前为 Grand Master 模式（LiDAR 未开 PTP slave）"
    warn ""
    warn "  需要操作（按优先级）："
    warn "  1. Unitree L2: AT 指令或 SDK 开启 PTP slave"
    warn "  2. Livox Mid-70: Livox Viewer → 设备设置 → 开启 PTP/gPTP"
    warn "  3. 开启后重启本脚本，ptp4l 会自动切换为 slave"
fi

log "PTP 运行中。使用 'sudo pmc -u -b 0 GET CURRENT_DATA_SET' 检查状态。"
echo "$PTP_PID"
