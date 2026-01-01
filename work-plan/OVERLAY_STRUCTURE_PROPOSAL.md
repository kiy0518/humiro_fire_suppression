# 오버레이 구조 개선 제안

## 현재 구조

```
thermal/src/
  - thermal_basic_overlay.*        # 열화상 기본 오버레이 (온도, 좌표 등)

targeting/src/
  - distance_overlay.*             # 라이다 거리 오버레이
  - aim_indicator.*                # 조준 표시
  - hotspot_tracker.*              # 핫스팟 추적
  - targeting_frame_compositor.*   # 오버레이들을 조합하는 컴포지터
```

### 문제점

1. **오버레이가 여러 폴더에 분산**: `thermal/`과 `targeting/`에 혼재
2. **새 오버레이 추가 시 위치 모호**: 어디에 추가해야 할지 불명확
3. **오버레이 간 의존성 관리 어려움**: 각 폴더의 빌드 시스템이 분리되어 있음
4. **확장성 부족**: 나중에 추가될 오버레이들의 위치가 불명확

## 제안: overlays/ 폴더 생성

### 옵션 1: 루트에 overlays/ 폴더 생성 (권장)

```
humiro_fire_suppression/
├── overlays/                      # 모든 오버레이 통합 관리
│   ├── src/
│   │   ├── thermal/
│   │   │   └── thermal_overlay.*  # thermal_basic_overlay 이동
│   │   ├── lidar/
│   │   │   └── distance_overlay.* # distance_overlay 이동
│   │   ├── targeting/
│   │   │   ├── aim_indicator.*
│   │   │   └── hotspot_tracker.*
│   │   └── CMakeLists.txt         # overlays_lib 빌드
│   └── README.md
│
├── targeting/
│   └── src/
│       └── targeting_frame_compositor.*  # 오버레이들을 조합
│
├── thermal/
│   └── src/
│       └── (thermal_basic_overlay 제거)
```

**장점:**
- 모든 오버레이가 한 곳에 모여 관리 용이
- 새로운 오버레이 추가 위치가 명확 (overlays/src/ 아래)
- 오버레이 라이브러리로 독립 빌드 가능
- 계층 구조 명확: 데이터 취득 (thermal/lidar) → 오버레이 (overlays) → 합성 (targeting)

**단점:**
- 파일 이동 작업 필요
- CMakeLists.txt 수정 필요

### 옵션 2: targeting/src/overlays/ 서브폴더

```
targeting/src/
├── overlays/
│   ├── thermal_overlay.*          # thermal_basic_overlay 이동
│   ├── distance_overlay.*         # 현재 위치 유지
│   ├── aim_indicator.*            # 현재 위치 유지
│   └── hotspot_tracker.*          # 현재 위치 유지
└── targeting_frame_compositor.*
```

**장점:**
- 파일 이동 작업이 적음 (thermal_basic_overlay만 이동)
- targeting과 오버레이가 같은 빌드 시스템 사용

**단점:**
- 오버레이가 targeting 폴더에 속하게 되어 위치가 다소 모호
- thermal 오버레이가 targeting 폴더에 있는 것이 논리적으로 어색

## 권장 사항: 옵션 1 (overlays/ 폴더)

**이유:**
1. **명확한 책임 분리**: 오버레이는 독립적인 계층
2. **확장성**: 나중에 추가될 오버레이들의 위치가 명확
3. **재사용성**: overlays_lib을 다른 프로젝트에서도 사용 가능
4. **유지보수성**: 모든 오버레이 코드가 한 곳에 있어 관리 용이

## 구현 계획

### Phase 1: overlays/ 폴더 및 기본 구조 생성
1. `overlays/src/` 폴더 생성
2. `overlays/src/thermal/`, `overlays/src/lidar/`, `overlays/src/targeting/` 서브폴더 생성
3. `overlays/src/CMakeLists.txt` 생성 (overlays_lib 빌드)

### Phase 2: 파일 이동
1. `thermal/src/thermal_basic_overlay.*` → `overlays/src/thermal/thermal_overlay.*`
2. `targeting/src/distance_overlay.*` → `overlays/src/lidar/distance_overlay.*`
3. `targeting/src/aim_indicator.*` → `overlays/src/targeting/aim_indicator.*`
4. `targeting/src/hotspot_tracker.*` → `overlays/src/targeting/hotspot_tracker.*`

### Phase 3: 빌드 시스템 업데이트
1. `overlays/src/CMakeLists.txt`: overlays_lib 생성
2. `targeting/src/CMakeLists.txt`: overlays_lib 링크, 기존 오버레이 소스 제거
3. `thermal/src/CMakeLists.txt`: thermal_basic_overlay 소스 제거
4. `application/CMakeLists.txt`: overlays_lib 링크 추가

### Phase 4: 헤더 경로 수정
1. `targeting_frame_compositor.*`: include 경로 수정
2. `application/main.cpp`: include 경로 수정 (필요시)

### Phase 5: 테스트 및 검증
1. 빌드 테스트
2. 실행 테스트
3. 문서 업데이트

## 파일명 변경 고려사항

- `thermal_basic_overlay` → `thermal_overlay`: 더 간결한 이름
- 나머지 파일명은 유지 (distance_overlay, aim_indicator, hotspot_tracker)

## 예상 작업 시간

- Phase 1-2: 파일 구조 생성 및 이동 (30분)
- Phase 3-4: 빌드 시스템 및 경로 수정 (30분)
- Phase 5: 테스트 및 검증 (20분)
- **총 예상 시간: 약 1.5시간**

