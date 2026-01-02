# px4_msgs 버전 정보

## 현재 사용 중인 버전

**브랜치**: `release/1.16`  
**커밋**: 392e831 - "Update message definitions Thu Aug 7 19:15:05 UTC 2025"  
**저장소**: https://github.com/PX4/px4_msgs.git

## 설치 위치

`~/humiro_fire_suppression/workspaces/px4_ros2_ws/src/px4_msgs`

## 버전 변경 방법

```bash
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws/src/px4_msgs
git checkout release/1.16
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
```

## 버전 확인 방법

```bash
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws/src/px4_msgs
git branch
git log -1 --oneline
```

## 중요 사항

⚠️ **px4_msgs 버전은 PX4 펌웨어 버전과 반드시 일치해야 합니다!**

- PX4 v1.16 펌웨어 → px4_msgs release/1.16 브랜치
- 버전 불일치 시 payload 크기 에러 발생

## 호환성

| PX4 펌웨어 버전 | px4_msgs 브랜치 | 상태 |
|----------------|-----------------|------|
| v1.16.x        | release/1.16    | ✅ 사용 중 |
| v1.15.x        | release/1.15    | - |
| main (개발)    | main            | ⚠️ 불안정 |

## 관련 문서

- [작업 로그: 20260102_px4_msgs_fix.md](../work-plan/20260102_px4_msgs_fix.md)
- [PX4 uXRCE-DDS 설정](PX4_16_UXRCE_DDS_SETUP.md)
