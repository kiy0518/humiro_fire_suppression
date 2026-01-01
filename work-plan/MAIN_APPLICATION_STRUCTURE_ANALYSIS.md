# 메인 애플리케이션 구조 분석

## 현재 구조

### 현재 상태
```
thermal/src/main.cpp
  └── thermal_rgb_streaming (실행 파일)
      ├── CameraManager (thermal/)
      ├── ThermalProcessor (thermal/)
      ├── ThermalBasicOverlay (thermal/)
      ├── LidarInterface (lidar/)
      ├── TargetingFrameCompositor (targeting/)
      └── StreamingManager (streaming/)
```

**문제점**:
- 메인 애플리케이션이 `thermal/` 폴더에 있음
- 이름이 `thermal_rgb_streaming`인데, 실제로는 모든 계층을 통합
- 아키텍처상 메인 애플리케이션은 별도 폴더에 있어야 함

## 아키텍처 관점

### 이상적인 구조

```
humiro_fire_suppression/
├── thermal/src/          # 열화상 데이터 취득 계층
│   └── (라이브러리 또는 독립 노드)
├── lidar/src/            # 라이다 데이터 취득 계층
│   └── (라이브러리 또는 독립 노드)
├── targeting/src/        # 타겟팅 계층
│   └── (라이브러리)
├── streaming/src/        # 스트리밍 계층
│   └── (라이브러리)
└── application/          # 통합 애플리케이션 (새로 추가)
    └── main.cpp          # 모든 계층을 통합하는 메인 애플리케이션
```

### 옵션 1: 별도 application/ 폴더 생성 (권장)

**장점**:
- ✅ 아키텍처가 명확함
- ✅ 각 계층이 독립적으로 유지됨
- ✅ 여러 통합 애플리케이션 생성 가능 (예: thermal_only, lidar_only 등)

**구조**:
```
application/
├── CMakeLists.txt
├── main.cpp              # 통합 메인 애플리케이션
└── config.h              # 애플리케이션 설정 (선택적)
```

### 옵션 2: 현재 구조 유지

**장점**:
- ✅ 변경 최소화
- ✅ 실제로는 문제가 없음 (하나의 통합 애플리케이션이므로)

**단점**:
- ⚠️ 아키텍처상 명확하지 않음
- ⚠️ thermal/ 폴더가 데이터 취득 + 통합 애플리케이션 둘 다 포함

## 현재 구조가 작동하는 이유

현재 구조도 실제로는 문제없이 작동합니다:

1. **단일 통합 애플리케이션**: 하나의 실행 파일로 모든 기능 제공
2. **계층 분리**: 각 계층은 라이브러리로 분리되어 있음
3. **main.cpp 역할**: 단순히 각 계층을 초기화하고 연결하는 역할

하지만 **명확성을 위해** 별도 `application/` 폴더를 만드는 것이 더 좋습니다.

## 제안: application/ 폴더 생성 (선택적)

### 장점
1. **명확한 구조**: 통합 애플리케이션이 어디에 있는지 명확
2. **확장성**: 나중에 다른 통합 애플리케이션 추가 용이
3. **책임 분리**: thermal/은 데이터 취득만, application/은 통합만

### 작업 내용
1. `application/` 폴더 생성
2. `thermal/src/main.cpp` → `application/main.cpp` 이동
3. `thermal/src/CMakeLists.txt` → `application/CMakeLists.txt` 생성/이동
4. 빌드 경로 업데이트

### 비용
- 약간의 리팩토링 작업 필요
- 빌드 스크립트 업데이트 필요

## 결론

**현재 구조는 기능적으로는 문제없지만, 아키텍처상 개선 여지가 있습니다.**

### 권장 사항

**옵션 A: 현재 구조 유지** (빠른 해결)
- 실제로 문제 없음
- 추가 작업 없음
- 단지 명확성만 떨어짐

**옵션 B: application/ 폴더 생성** (아키텍처 개선)
- 명확한 구조
- 약간의 리팩토링 필요
- 향후 확장 용이

### 사용자 결정 필요

어떤 방식을 선호하시나요?
1. 현재 구조 유지 (기능상 문제 없음)
2. application/ 폴더로 분리 (아키텍처 개선)

