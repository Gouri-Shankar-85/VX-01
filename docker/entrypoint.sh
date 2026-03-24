#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f "/vx01_ws/install/setup.bash" ]; then
    source /vx01_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -n "${DISPLAY}" ]; then
    export QT_X11_NO_MITSHM=1
fi

if [ -d "/root/.ssh" ] && ls /root/.ssh/id_* 2>/dev/null | head -1 > /dev/null 2>&1; then
    ssh-keyscan -t rsa github.com >> /root/.ssh/known_hosts 2>/dev/null || true
fi

if [ -d "/vx01_ws/.git" ]; then
    git -C /vx01_ws remote set-url origin \
        git@github.com:Gouri-Shankar-85/VX-01.git 2>/dev/null || true
fi

echo 'export PS1="\[\033[1;32m\]vx01\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]# "' >> ~/.bashrc

exec "$@"