# 🖥️ PX4 SITL 시뮬레이션 PC 공통 설정 가이드

**작성일:** 2026-01-23  
**대상 OS:** Ubuntu 22.04

> ⚠️ **이 가이드는 모든 시뮬레이션 PC에서 공통으로 수행해야 하는 설정입니다.**  
> 이 가이드를 완료한 후, 각 드론별 가이드(DRONE1, DRONE2, DRONE3)를 참고하세요.

---

## 📋 공통 설치 순서

1. 기본 패키지 설치
2. PX4 설치 및 빌드
3. ROS2 Humble 설치
4. px4_msgs 빌드
5. Micro XRCE-DDS Agent 설치
6. rcS 파일에 VIM4 연결 설정 추가

---

## 1. 기본 패키지 설치

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
    git wget curl cmake build-essential \
    python3-pip python3-venv \
    openjdk-11-jdk ninja-build \
    exiftool astyle \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    net-tools
```

---

## 2. PX4 설치 및 빌드

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 빌드 도구 설치 (약 20분)
bash ./Tools/setup/ubuntu.sh

# 재부팅
sudo reboot
```

재부팅 후:

```bash
cd ~/PX4-Autopilot

# 첫 빌드 (약 30분)
make px4_sitl gz_x500

# 빌드 완료 후 Ctrl+C로 종료
```

---

## 3. ROS2 Humble 설치

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. px4_msgs 빌드

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs

# colcon 명령어가 없다는 오류 발생 시:
# sudo apt install python3-colcon-common-extensions -y

echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Micro XRCE-DDS Agent 설치

```bash
# 의존성
sudo apt install -y libasio-dev libtinyxml2-dev libp11-kit-dev libssl-dev

# Micro-CDR
cd ~
git clone https://github.com/eProsima/Micro-CDR.git
cd Micro-CDR && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# Micro-XRCE-DDS-Client
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Client.git
cd Micro-XRCE-DDS-Client && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# Micro-XRCE-DDS-Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# 확인
MicroXRCEAgent --version
```

---

## 6. rcS에 인스턴스별 VIM4 연결 설정 추가 (한 번만 실행)

> **참고:** 이 설정은 드론 1, 2, 3 모두에 적용됩니다. 한 번만 설정하면 됩니다.

### rcS 파일 끝에 추가

```bash
nano ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS
```

**파일 맨 끝 (`replay trystart` 다음)에 추가:**

```bash
# ============ VIM4 SITL 연결 (인스턴스별 설정) ============
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set EKF2_EV_CTRL 0
param set EKF2_HGT_REF 1
mavlink stop-all

if [ "$px4_instance" = "0" ]; then
    # 드론 1 -> VIM4 192.168.100.11, MAV_SYS_ID=4
    param set MAV_SYS_ID 4
    mavlink start -u 14540 -o 18001 -t 192.168.100.11 -r 4000000
elif [ "$px4_instance" = "1" ]; then
    # 드론 2 -> VIM4 192.168.100.21, MAV_SYS_ID=5
    param set MAV_SYS_ID 5
    mavlink start -u 14541 -o 18002 -t 192.168.100.21 -r 4000000
elif [ "$px4_instance" = "2" ]; then
    # 드론 3 -> VIM4 192.168.100.31, MAV_SYS_ID=6
    param set MAV_SYS_ID 6
    mavlink start -u 14542 -o 18003 -t 192.168.100.31 -r 4000000
fi
# =========================================================
```

### PX4 다시 빌드

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

---

## ✅ 공통 설정 완료 체크리스트

- [ ] PX4 빌드 완료
- [ ] ROS2 Humble 설치 완료
- [ ] px4_msgs 빌드 완료
- [ ] MicroXRCEAgent 설치 완료
- [ ] rcS 파일 수정 완료
- [ ] PX4 재빌드 완료

---

## 🔢 드론별 설정 정보

| 드론 | VIM4 IP | SITL 포트 | MAVLink 로컬 포트 | 인스턴스 | MAV_SYS_ID |
|------|---------|-----------|-------------------|----------|------------|
| 드론 1 | 192.168.100.11 | 18001 | 14540 | -i 0 | 4 |
| 드론 2 | 192.168.100.21 | 18002 | 14541 | -i 1 | 5 |
| 드론 3 | 192.168.100.31 | 18003 | 14542 | -i 2 | 6 |

---

## ➡️ 다음 단계

공통 설정을 완료한 후, 아래 드론별 가이드를 참고하세요:

- **드론 1:** `SIMULATION_PC_SETUP_GUIDE_DRONE1.md`
- **드론 2:** `SIMULATION_PC_SETUP_GUIDE_DRONE2.md`
- **드론 3:** `SIMULATION_PC_SETUP_GUIDE_DRONE3.md`
