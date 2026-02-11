#!/bin/bash
# Patch the VIM4 section in rcS to include uxrce_dds_client restart

RCFILE=~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS

# Backup
cp "$RCFILE" "$RCFILE.backup.$(date +%Y%m%d_%H%M%S)"

# Replace the VIM4 section
python3 << 'PYEOF'
import re

with open("/home/humiro/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS", "r") as f:
    content = f.read()

old_section = """# ============ VIM4 SITL 연결 (인스턴스별 설정) ============
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set EKF2_EV_CTRL 0
param set EKF2_HGT_REF 1
mavlink stop-all

if [ "$px4_instance" = "0" ]; then
    # 드론 1 -> VIM4 192.168.100.11, MAV_SYS_ID=1
    param set MAV_SYS_ID 1
    mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000
elif [ "$px4_instance" = "1" ]; then
    # 드론 2 -> VIM4 192.168.100.21, MAV_SYS_ID=2
    param set MAV_SYS_ID 2
    mavlink start -u 14541 -o 18002 -t 192.168.100.21 -r 4000000
elif [ "$px4_instance" = "2" ]; then
    # 드론 3 -> VIM4 192.168.100.31, MAV_SYS_ID=3
    param set MAV_SYS_ID 3
    mavlink start -u 14542 -o 18003 -t 192.168.100.31 -r 4000000
fi
# ========================================================="""

new_section = """# ============ VIM4 SITL 연결 (인스턴스별 설정) ============
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set EKF2_EV_CTRL 0
param set EKF2_HGT_REF 1
mavlink stop-all
uxrce_dds_client stop

if [ "$px4_instance" = "0" ]; then
    # 드론 1 -> VIM4 192.168.100.11, MAV_SYS_ID=1
    param set MAV_SYS_ID 1
    mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.11 -p 8888
elif [ "$px4_instance" = "1" ]; then
    # 드론 2 -> VIM4 192.168.100.21, MAV_SYS_ID=2
    param set MAV_SYS_ID 2
    mavlink start -u 14541 -o 18002 -t 192.168.100.21 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.21 -p 8888 -n px4_1
elif [ "$px4_instance" = "2" ]; then
    # 드론 3 -> VIM4 192.168.100.31, MAV_SYS_ID=3
    param set MAV_SYS_ID 3
    mavlink start -u 14542 -o 18003 -t 192.168.100.31 -r 4000000
    uxrce_dds_client start -t udp -h 192.168.100.31 -p 8888 -n px4_2
fi
# ========================================================="""

if old_section in content:
    content = content.replace(old_section, new_section)
    with open("/home/humiro/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS", "w") as f:
        f.write(content)
    print("SUCCESS: VIM4 section updated with uxrce_dds_client")
else:
    print("ERROR: Could not find old VIM4 section")
PYEOF
