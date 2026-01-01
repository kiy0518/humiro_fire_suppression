# Phase 3 테스트 가이드

## 빌드 모드

### 1. ROS2 비활성화 모드 (기본, 현재 상태)
**목적**: 기존 기능 정상 동작 확인

```bash
cd thermal/src/build
cmake ..
make -j$(nproc)
```

**결과**: 
- 바이너리 크기: ~397KB
- 기능: 기존 기능만 (열화상, 라이다, 스트리밍)
- ROS2: 없음

**테스트 항목**:
- [ ] 열화상 카메라 동작 확인
- [ ] 라이다 데이터 수집 확인
- [ ] RTSP 스트리밍 확인
- [ ] HTTP 스트리밍 확인
- [ ] 타겟팅 오버레이 표시 확인
- [ ] 성능 측정 (FPS)

### 2. ROS2 활성화 모드 (선택적)
**목적**: ROS2 토픽 발행 기능 확인

**사전 요구사항**:
```bash
# ROS2 패키지 설치 확인 필요
# (현재 시스템에 설치되어 있지 않을 수 있음)
```

```bash
cd thermal/src/build
cmake .. -DENABLE_ROS2=ON
make -j$(nproc)
```

**결과**: 
- 바이너리 크기: ~더 큼 (ROS2 라이브러리 포함)
- 기능: 기존 기능 + ROS2 토픽 발행
- ROS2 토픽:
  - `/thermal/image` - 열화상 이미지
  - `/thermal/max_temperature` - 최대 온도
  - `/thermal/min_temperature` - 최소 온도
  - `/thermal/center` - 중심점
  - `/thermal/hotspot` - Hotspot 좌표
  - `/lidar/points` - 포인트 클라우드
  - `/lidar/front_distance` - 전방 거리

**테스트 항목**:
- [ ] ROS2 노드 실행 확인
- [ ] 토픽 발행 확인
  ```bash
  ros2 topic list
  ros2 topic echo /thermal/max_temperature
  ros2 topic echo /lidar/front_distance
  ```
- [ ] 기존 기능 정상 동작 확인 (ROS2 활성화 시에도)
- [ ] 성능 영향 측정

## 현재 권장 사항

**현재로서는 ROS2 비활성화 모드로 테스트하는 것을 권장합니다:**

1. **기존 기능 검증**: Phase 1, 2 변경사항이 정상 작동하는지 확인
2. **안정화**: 현재 구조에서 안정화 먼저 진행
3. **ROS2는 필요시**: 실제로 ROS2 토픽이 필요할 때 활성화

## 다음 단계

1. **현재 빌드로 기능 테스트** (ROS2 비활성화)
   - 열화상/라이다/스트리밍 동작 확인
   - Phase 1, 2 리팩토링 검증

2. **필요시 ROS2 활성화 테스트**
   - ROS2 환경 확인 및 설치
   - ROS2 토픽 발행 테스트

3. **Phase 4: 테스트 및 검증**
   - 단위 테스트 작성
   - 통합 테스트
   - 성능 측정

