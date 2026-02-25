#!/bin/bash
# =============================================================================
# VX-01 Container Entrypoint
# =============================================================================

set -e

echo "╔══════════════════════════════════════════════╗"
echo "║       VX-01 Hybrid Robot — Docker Shell       ║"
echo "╚══════════════════════════════════════════════╝"

# ─── Source ROS2 ──────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

if [ -f "/vx01_ws/install/setup.bash" ]; then
    source /vx01_ws/install/setup.bash
    echo "[✓] Workspace sourced"
else
    echo "[!] Workspace not built — run: cd /vx01_ws && colcon build --symlink-install"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ─── X11 (GUI apps: RViz, Gazebo) ─────────────────────────────────────────────
if [ -n "${DISPLAY}" ]; then
    export QT_X11_NO_MITSHM=1
    echo "[✓] GUI available (DISPLAY=${DISPLAY})"
else
    echo "[i] No DISPLAY — RViz/Gazebo won't open (hardware shell is fine)"
fi

# ─── Device detection using STABLE udev symlinks ─────────────────────────────
# udev rules create these fixed names regardless of which USB port you plug into
echo ""
echo "── Device Status ───────────────────────────────────────────────────────"

check_device() {
    local name="$1"
    local symlink="$2"
    local fallback="$3"
    if [ -e "${symlink}" ]; then
        echo "[✓] ${name}: ${symlink}"
    elif [ -n "${fallback}" ] && ls ${fallback} 2>/dev/null | head -1 > /dev/null 2>&1; then
        echo "[!] ${name}: symlink not found, fallback $(ls ${fallback} 2>/dev/null | head -1)"
        echo "    → udev rules not active on host. Run: ./build.sh udev-install"
    else
        echo "[ ] ${name}: NOT connected (${symlink})"
    fi
}

check_device "Radilink PIX6 (ArduPilot)" "/dev/ttyPIXHAWK"  "/dev/ttyACM*"
check_device "Pololu Maestro 18ch"       "/dev/ttyMAESTRO"  "/dev/ttyACM*"
check_device "TFmini-S LiDAR"           "/dev/ttyTFMINI"   "/dev/ttyUSB*"
check_device "IM10A IMU"                "/dev/ttyIMU"      "/dev/ttyUSB*"

# HP60C is a UVC video device
if ls /dev/video* 2>/dev/null | head -1 > /dev/null 2>&1; then
    echo "[✓] HP60C Depth Camera: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
else
    echo "[ ] HP60C Depth Camera: NOT connected (/dev/video*)"
fi

echo "────────────────────────────────────────────────────────────────────────"
echo ""

# ─── GitHub SSH ───────────────────────────────────────────────────────────────
# Each contributor mounts their own ~/.ssh from their laptop.
# No usernames are stored inside this image.
if [ -d "/root/.ssh" ] && ls /root/.ssh/id_* 2>/dev/null | head -1 > /dev/null 2>&1; then
    ssh-keyscan -t rsa github.com >> /root/.ssh/known_hosts 2>/dev/null || true
    echo "[✓] SSH key found — git push to GitHub enabled"
    echo "    Repo: git@github.com:Gouri-Shankar-85/VX-01.git"
else
    echo "[i] No SSH key mounted — git pull works, git push disabled"
    echo "    To enable push: ensure ~/.ssh/id_* exists on your host machine"
fi

# Set git remote to SSH (in case it was cloned via HTTPS)
if [ -d "/vx01_ws/.git" ]; then
    git -C /vx01_ws remote set-url origin \
        git@github.com:Gouri-Shankar-85/VX-01.git 2>/dev/null || true
fi

echo ""
echo "[✓] Ready. Workspace: /vx01_ws | ROS: humble | Domain: ${ROS_DOMAIN_ID}"
echo ""

echo 'export PS1="\[\033[1;32m\]vx01-docker\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]# "' >> ~/.bashrc

exec "$@"