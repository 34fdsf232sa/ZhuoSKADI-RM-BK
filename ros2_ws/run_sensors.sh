#!/bin/bash
set -e

SCRIPT_DIR=$(cd $(dirname "$0") && pwd)

L2_LIDAR_IP=${L2_LIDAR_IP:-192.168.1.62}
L2_LOCAL_IP=${L2_LOCAL_IP:-192.168.1.2}
L2_LOCAL_CIDR=${L2_LOCAL_CIDR:-192.168.1.2/24}
L2_IFACE=${L2_IFACE:-}
TIME_SYNC_MODE=${TIME_SYNC_MODE:-ptp}
PTP_CFG="${PTP_CFG:-${SCRIPT_DIR}/src/mapless_nav/config/ptp_lidar.cfg}"
PTP_IFACE="${PTP_IFACE:-}"
run_sudo() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo -n "$@" 2>/dev/null || {
            echo "[run_sensors] Need passwordless sudo. Configure:"
            echo "  sudo visudo -f /etc/sudoers.d/lidar"
            echo "  Add: $USER ALL=(ALL) NOPASSWD: /usr/sbin/ptp4l, /usr/bin/systemctl, /usr/sbin/ip"
            return 1
        }
    fi
}

run_ip_admin() {
    if [ "$(id -u)" -eq 0 ]; then
        ip "$@"
    else
        sudo -n ip "$@" 2>/dev/null || run_sudo ip "$@"
    fi
}

route_l2_in_table() {
    local iface=$1
    local table=$2

    run_ip_admin route replace "$L2_LIDAR_IP/32" dev "$iface" src "$L2_LOCAL_IP" table "$table"
}

find_l2_iface() {
    local iface

    if [ -n "$L2_IFACE" ]; then
        if ip link show dev "$L2_IFACE" >/dev/null 2>&1; then
            echo "$L2_IFACE"
            return 0
        fi
        echo "ERROR: L2_IFACE=$L2_IFACE does not exist." >&2
        return 1
    fi

    iface=$(ip -o -4 addr show | awk -v ip="$L2_LOCAL_IP" '$4 ~ "^" ip "/" {print $2; exit}')
    if [ -n "$iface" ]; then
        echo "$iface"
        return 0
    fi

    iface=$(ip -o link show up | awk -F': ' '
        $2 !~ /^(lo|docker|br-|veth|wl|FlClash)/ && $3 ~ /LOWER_UP/ {
            print $2
            exit
        }')
    if [ -n "$iface" ]; then
        echo "$iface"
        return 0
    fi

    iface=$(ip -o -4 addr show | awk -F'[: ]+' '
        $2 !~ /^(lo|docker|br-|veth|wl|FlClash)/ {
            print $2
            exit
        }')
    if [ -n "$iface" ]; then
        echo "$iface"
        return 0
    fi

    return 1
}

ensure_l2_network() {
    local iface

    if ! iface=$(find_l2_iface); then
        echo "ERROR: No wired interface found for Unitree L2 network." >&2
        echo "       Connect the L2 Ethernet cable or set L2_IFACE manually." >&2
        exit 1
    fi

    if ! ip -o link show dev "$iface" | grep -q 'LOWER_UP'; then
        echo "Warning: $iface has no carrier. Check the L2 Ethernet cable/link if pointcloud stays empty." >&2
    fi

    if ! ip -o -4 addr show dev "$iface" | awk -v ip="$L2_LOCAL_IP" '$4 ~ "^" ip "/" {found=1} END {exit !found}'; then
        echo "Configuring Unitree L2 local IP $L2_LOCAL_CIDR on $iface ..."
        if ! run_ip_admin addr add "$L2_LOCAL_CIDR" dev "$iface" 2>/dev/null; then
            echo "ERROR: Failed to configure $L2_LOCAL_CIDR on $iface." >&2
            echo "       Run 'sudo -v' before this launcher, or run:" >&2
            echo "       sudo ip addr add $L2_LOCAL_CIDR dev $iface" >&2
            exit 1
        fi
    fi

    if ! ip -o -4 addr show dev "$iface" | awk -v ip="$L2_LOCAL_IP" '$4 ~ "^" ip "/" {found=1} END {exit !found}'; then
        echo "ERROR: Failed to configure $L2_LOCAL_CIDR on $iface." >&2
        exit 1
    fi

    if ! route_l2_in_table "$iface" main; then
        echo "ERROR: Failed to route $L2_LIDAR_IP via $iface." >&2
        echo "       Run 'sudo -v' before this launcher, or run:" >&2
        echo "       sudo ip route replace $L2_LIDAR_IP/32 dev $iface src $L2_LOCAL_IP table main" >&2
        exit 1
    fi

    if ip rule show | grep -q 'lookup 2022'; then
        if ! route_l2_in_table "$iface" 2022; then
            echo "ERROR: Failed to bypass policy routing for $L2_LIDAR_IP." >&2
            echo "       Run 'sudo -v' before this launcher, or run:" >&2
            echo "       sudo ip route replace $L2_LIDAR_IP/32 dev $iface src $L2_LOCAL_IP table 2022" >&2
            exit 1
        fi
    fi

    echo "Unitree L2 network ready on $iface: local=$L2_LOCAL_IP lidar=$L2_LIDAR_IP"
}

ensure_l2_network

# =============================================================================
# PTP 时间同步 — 使用 systemd 服务 (开机自启)
# =============================================================================
check_ptp() {
    if [ "$TIME_SYNC_MODE" != "ptp" ]; then
        echo "[run_sensors] TIME_SYNC_MODE=$TIME_SYNC_MODE, skip PTP check"
        return 0
    fi

    if systemctl is-active --quiet ptp4l-lidar 2>/dev/null; then
        echo "[run_sensors] ptp4l-lidar systemd service is active ✓"
        journalctl -u ptp4l-lidar --no-pager -n 1 2>/dev/null || true
    else
        echo "[run_sensors] WARNING: ptp4l-lidar service not running! Starting it..."
        run_sudo systemctl start ptp4l-lidar 2>/dev/null || echo "  Failed. Run: sudo systemctl start ptp4l-lidar"
    fi
}

check_ptp

# =============================================================================
# ROS2 启动
# =============================================================================
source /opt/ros/humble/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "Warning: install/setup.bash not found in $SCRIPT_DIR"
fi
ros2 launch "$SCRIPT_DIR/launch_sensors.py" time_sync_mode:="$TIME_SYNC_MODE"
