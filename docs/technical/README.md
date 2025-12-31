# Cluster_Drone

PX4 + ROS2 기반 VIM4 군집 드론 시스템

## 개요

이 프로젝트는 Khadas VIM4 (Ubuntu 22.04 ARM64)에서 PX4와 ROS2를 XRCE-DDS로 연결하고, 
MAVLink Router를 통해 QGroundControl과 통신하는 군집 드론 시스템입니다.

## 📚 문서

| 문서 | 설명 |
|------|------|
| [1000-Project_Summery.html](1000-Project_Summery.html) | 프로젝트 전체 아키텍처 및 요약 |
| [3000-INSTALLATION_GUIDE.html](3000-INSTALLATION_GUIDE.html) | **신규 설치 가이드** (빈 OS → 완전 설치) |
| [4000-POST_INSTALL_GUIDE.html](4000-POST_INSTALL_GUIDE.html) | **이미지 복원 후 설정 가이드** |
| [5000-IMAGE_BACKUP_RESTORE.html](5000-IMAGE_BACKUP_RESTORE.html) | **OOWOW 이미지 백업/복원 가이드** |

## 빠른 시작

### 방법 1: 커스텀 이미지 사용 (권장)
```bash
# 1. OOWOW로 이미지 복원

# 2. 기체 설정 수정
cd ~/projects/Cluster_Drone
nano device_config.env

# 3. 설정 적용
sudo ./003-apply_config.sh

# 4. 검증 (sudo 없이!)
bash 103-verify_installation.sh

# 5. 재부팅
sudo reboot
```

### 방법 2: 신규 설치
```bash
# 1. 프로젝트 다운로드
cd ~/projects
git clone https://github.com/your-repo/Cluster_Drone.git
cd Cluster_Drone

# 2. 기체 설정 수정
nano device_config.env

# 3. 설치 (30-60분 소요)
sudo ./001-install_all.sh

# 4. 재부팅
sudo reboot

# 5. 검증 (sudo 없이!)
bash 103-verify_installation.sh
```

## 파일 구조

```
Cluster_Drone/
├── [문서]
│   ├── 1000-Project_Summery.html         # 프로젝트 요약
│   ├── 3000-INSTALLATION_GUIDE.html      # 설치 가이드
│   ├── 4000-POST_INSTALL_GUIDE.html      # 복원 후 가이드
│   └── 5000-IMAGE_BACKUP_RESTORE.html    # 백업/복원 가이드
│
├── [설치 스크립트]
│   ├── 001-install_all.sh                # 통합 설치 (신규 설치용)
│   ├── 001-install_px4_ros2_complete.sh  # ROS2 + micro-ROS Agent
│   ├── 002-install_mavlink_router.sh     # MAVLink Router
│   └── 003-apply_config.sh               # 설정 적용 (복원 후 실행)
│
├── [검증/유틸리티]
│   ├── 101-check_px4_connection.sh       # PX4 연결 확인
│   ├── 102-ip_to_decimal.sh              # IP 변환 도구
│   ├── 103-verify_installation.sh        # 전체 시스템 검증
│   └── 104-save_versions.sh              # 버전 저장
│
├── [설정]
│   ├── device_config.env                 # ★ 기체별 설정 파일
│   └── versions.env                      # 패키지 버전 정보
│
└── [빌드 디렉토리]
    ├── micro_ros_ws/                     # Micro-ROS Agent
    ├── px4_ros2_ws/                      # PX4 ROS2 메시지
    └── mavlink-router/                   # MAVLink Router 소스
```

## 기체별 설정

### device_config.env 수정

| 드론 | DRONE_ID | ETH0_IP | FC_IP | WIFI_IP |
|------|----------|---------|-------|---------|
| #1 | 1 | 10.0.0.11 | 10.0.0.12 | 192.168.100.11 |
| #2 | 2 | 10.0.0.21 | 10.0.0.22 | 192.168.100.21 |
| #3 | 3 | 10.0.0.31 | 10.0.0.32 | 192.168.100.31 |

## 주요 명령어

```bash
# 서비스 상태 확인
sudo systemctl status dnsmasq-px4
sudo systemctl status mavlink-router
sudo systemctl status micro-ros-agent

# ROS2 토픽 확인
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_attitude --once

# 설정 재적용
sudo ./003-apply_config.sh

# 시스템 검증
bash 103-verify_installation.sh  # sudo 없이!
```

## 시스템 요구사항

- **SBC**: Khadas VIM4 (ARM64)
- **OS**: Ubuntu 22.04 LTS
- **FC**: Pixhawk (PX4 v1.16.0)
- **ROS**: ROS2 Humble

## 라이센스

MIT License
