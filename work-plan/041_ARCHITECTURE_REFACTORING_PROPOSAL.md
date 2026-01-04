# 아키텍처 리팩토링 제안서

## 현재 구조의 문제점

### 현재 구조
```
thermal/
  ├── 데이터 취득 (카메라, 열화상)
  ├── 데이터 처리 (thermal_processor)
  ├── 오버레이 및 합성 (frame_compositor) ← 문제!
  │   ├── 열화상 오버레이
  │   ├── 라이다 오버레이
  │   ├── 거리 표시
  │   ├── 조준 표시
  │   └── hotspot 추적 표시
  └── 영상 전송 (RTSP, HTTP)

lidar/
  └── 데이터 취득만 (lidar_interface)
```

### 문제점
1. **단일 책임 원칙 위반**: `frame_compositor`가 너무 많은 책임을 가짐
   - 데이터 취득 결과 표시
   - 타겟팅 정보 표시
   - 영상 전송과 강하게 결합

2. **재사용성 부족**: 
   - 타겟팅 로직이 영상 전송과 결합되어 독립적으로 사용 불가
   - 다른 출력 방식(ROS2 토픽 등)으로 확장 어려움

3. **테스트 어려움**:
   - 각 계층이 강하게 결합되어 단위 테스트 어려움
   - 모의(mock) 객체 사용이 복잡

4. **유지보수성 저하**:
   - 변경 시 영향 범위가 큼
   - 코드 추적이 어려움

## 제안된 아키텍처

### 계층 구조

```
┌─────────────────────────────────────────────────────────┐
│                    영상 전송 계층                        │
│  (streaming/)                                           │
│  - RTSP 서버                                            │
│  - HTTP 서버                                            │
│  - ROS2 이미지 토픽 발행                                │
│  - 통합 프레임 버퍼 관리                                │
└─────────────────────────────────────────────────────────┘
                          ↑
                          │ 최종 합성된 프레임
┌─────────────────────────────────────────────────────────┐
│                  타겟팅/표시 계층                        │
│  (targeting/)                                           │
│  - 거리 표시 (라이다 기반)                              │
│  - 조준 표시 (타겟 위치)                                │
│  - Hotspot 추적 표시                                    │
│  - 타겟팅 정보 오버레이                                 │
│  - 탄도 계산 결과 표시                                  │
└─────────────────────────────────────────────────────────┘
                          ↑
                          │ 타겟팅 데이터 + 원본 프레임
┌─────────────────────────────────────────────────────────┐
│                   데이터 취득 계층                       │
│                                                          │
│  thermal/              lidar/                           │
│  - 카메라 취득         - 라이다 데이터 취득              │
│  - 열화상 처리         - 거리 데이터 파싱                │
│  - 기본 오버레이       - 포인트 클라우드 생성            │
│    (온도, 좌표 등)                                       │
│  - 데이터 구조화       - 데이터 구조화                   │
│  - ROS2 토픽 발행      - ROS2 토픽 발행                  │
└─────────────────────────────────────────────────────────┘
```

### 폴더 구조

```
humiro_fire_suppression/
├── thermal/                    # 열화상 데이터 취득
│   ├── src/
│   │   ├── camera_manager.*    # 카메라 제어
│   │   ├── thermal_processor.* # 열화상 처리
│   │   ├── thermal_data.h      # 데이터 구조
│   │   └── thermal_basic_overlay.*  # 기본 오버레이 (온도, 좌표)
│   └── CMakeLists.txt
│
├── lidar/                      # 라이다 데이터 취득
│   ├── src/
│   │   ├── lidar_interface.*   # 라이다 제어
│   │   ├── lidar_data.h        # 데이터 구조
│   │   └── lidar_point_cloud.* # 포인트 클라우드 처리
│   └── CMakeLists.txt
│
├── targeting/                  # 타겟팅 및 표시 (신규/개선)
│   ├── src/
│   │   ├── targeting_processor.*    # 타겟팅 로직
│   │   ├── distance_overlay.*       # 거리 표시
│   │   ├── aim_indicator.*          # 조준 표시
│   │   ├── hotspot_tracker.*        # Hotspot 추적 표시
│   │   ├── frame_compositor.*       # 타겟팅 정보 합성
│   │   └── targeting_data.h         # 타겟팅 데이터 구조
│   └── CMakeLists.txt
│
└── streaming/                  # 영상 전송 (신규)
    ├── src/
    │   ├── rtsp_server.*       # RTSP 서버
    │   ├── http_server.*       # HTTP 서버
    │   ├── ros2_image_publisher.*  # ROS2 이미지 토픽
    │   ├── frame_buffer.*      # 프레임 버퍼 관리
    │   └── streaming_manager.* # 전송 통합 관리
    └── CMakeLists.txt
```

## 각 계층의 책임

### 1. 데이터 취득 계층 (thermal/, lidar/)

**역할**: 센서에서 데이터를 취득하고 기본 처리만 수행

**thermal/**
- 카메라 프레임 취득
- 열화상 데이터 처리 및 기본 분석
- 기본 오버레이 (온도 값, 좌표 등)
- `ThermalData` 구조체로 데이터 구조화
- ROS2 토픽 발행 (`/thermal/data`, `/thermal/image`)

**lidar/**
- 라이다 데이터 취득
- 포인트 클라우드 생성
- `LidarPointCloud` 구조체로 데이터 구조화
- ROS2 토픽 발행 (`/lidar/points`, `/lidar/front_distance`)

**제거 대상**:
- ❌ 타겟팅 관련 오버레이
- ❌ 거리 표시 (라이다 오버레이)
- ❌ 조준 표시
- ❌ 영상 전송 로직

### 2. 타겟팅/표시 계층 (targeting/)

**역할**: 취득된 데이터를 기반으로 타겟팅 정보를 계산하고 표시

**주요 컴포넌트**:
- `targeting_processor.*`: 타겟팅 로직 (hotspot 선택, 거리 계산 등)
- `distance_overlay.*`: 라이다 기반 거리 표시
- `aim_indicator.*`: 조준 표시 (십자선, 타겟 박스 등)
- `hotspot_tracker.*`: Hotspot 추적 표시
- `frame_compositor.*`: 모든 타겟팅 정보를 프레임에 합성

**입력**:
- `ThermalData` (thermal/에서)
- `LidarPointCloud` (lidar/에서)
- 원본 RGB 프레임

**출력**:
- 타겟팅 정보가 오버레이된 프레임
- `TargetingData` 구조체 (ROS2 토픽으로 발행)

### 3. 영상 전송 계층 (streaming/)

**역할**: 최종 합성된 프레임을 다양한 방식으로 전송

**주요 컴포넌트**:
- `rtsp_server.*`: RTSP 스트리밍
- `http_server.*`: HTTP 스트리밍 (MJPEG)
- `ros2_image_publisher.*`: ROS2 이미지 토픽 발행
- `frame_buffer.*`: 프레임 버퍼 관리 (여러 클라이언트 지원)
- `streaming_manager.*`: 모든 전송 방식 통합 관리

**입력**:
- 타겟팅 정보가 합성된 최종 프레임

**출력**:
- RTSP 스트림
- HTTP 스트림
- ROS2 토픽 (`/camera/image`)

## 데이터 흐름

### 현재 (문제)
```
Camera → ThermalProcessor → FrameCompositor → RTSPServer
                              ↑
                              └─ Lidar (강하게 결합)
                              └─ Targeting (강하게 결합)
```

### 제안 (개선)
```
Camera → ThermalProcessor → ThermalData (ROS2 토픽)
                                        ↓
Lidar → LidarInterface → LidarPointCloud (ROS2 토픽)
                                        ↓
                        TargetingProcessor
                                        ↓
                        FrameCompositor (타겟팅 합성)
                                        ↓
                        StreamingManager
                                        ├─ RTSP Server
                                        ├─ HTTP Server
                                        └─ ROS2 Publisher
```

## 마이그레이션 계획

### Phase 1: 타겟팅 계층 분리 (우선순위 높음)
1. `targeting/` 폴더에 새로운 구조 생성
2. `frame_compositor.cpp`에서 타겟팅 관련 코드 분리
   - `distance_overlay.*` 이동
   - `aim_indicator.*` 생성 (조준 표시)
   - `hotspot_tracker.*` 생성 (hotspot 추적 표시)
3. `thermal/`의 `frame_compositor`를 `thermal_basic_overlay`로 축소

### Phase 2: 스트리밍 계층 분리
1. `streaming/` 폴더 생성
2. `rtsp_server.*`, `http_server.*` 이동
3. `streaming_manager.*` 생성하여 통합 관리

### Phase 3: ROS2 통신 강화
1. 모든 계층 간 통신을 ROS2 토픽으로 전환
2. 동기화 메커니즘 구현 (메시지 타임스탬프 기반)

### Phase 4: 테스트 및 검증
1. 각 계층 단위 테스트 작성
2. 통합 테스트
3. 성능 측정 및 최적화

## 예상 효과

### 장점
1. **관심사 분리**: 각 모듈이 명확한 단일 책임을 가짐
2. **재사용성**: 타겟팅 로직을 다른 출력 방식에서도 사용 가능
3. **테스트 용이성**: 각 계층을 독립적으로 테스트 가능
4. **확장성**: 새로운 전송 방식 추가가 용이
5. **유지보수성**: 변경 영향 범위가 명확함

### 고려사항
1. **성능**: ROS2 토픽을 통한 데이터 전달은 약간의 오버헤드 발생 가능
   - 해결: 공유 메모리 또는 메모리 맵 파일 사용 고려
2. **동기화**: 여러 데이터 소스를 동기화하는 메커니즘 필요
   - 해결: 타임스탬프 기반 동기화
3. **마이그레이션 비용**: 기존 코드를 리팩토링하는 데 시간 소요
   - 해결: 점진적 마이그레이션 (Phase별로 진행)

## 결론

제안된 아키텍처는 **단일 책임 원칙**, **관심사 분리**, **의존성 역전 원칙**을 따르는 깔끔한 계층 구조입니다.

현재 `frame_compositor`가 너무 많은 책임을 가지고 있어 유지보수가 어려운 상황에서, 이를 명확히 분리함으로써:
- 코드 가독성 향상
- 테스트 용이성 향상
- 확장성 향상
- 유지보수성 향상

을 기대할 수 있습니다.

**추천**: Phase 1부터 시작하여 점진적으로 마이그레이션하는 것을 권장합니다.

