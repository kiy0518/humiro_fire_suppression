# Application 폴더 분리 완료

## 완료 일자
2025년 1월 1일

## 작업 내용

### 목적
메인 애플리케이션을 `thermal/src/`에서 `application/` 폴더로 분리하여 아키텍처를 명확히 함.

### 변경 사항

#### 1. 폴더 구조 변경

**이전**:
```
thermal/src/
├── main.cpp  ← 모든 계층을 통합하는 메인 애플리케이션
├── camera_manager.*
├── thermal_processor.*
└── ...
```

**현재**:
```
application/
├── main.cpp  ← 통합 애플리케이션
├── config.h
├── CMakeLists.txt
└── build.sh  ← 빌드 스크립트 (새로 추가)

thermal/src/
├── camera_manager.*  ← 데이터 취득 계층만
├── thermal_processor.*
└── CMakeLists.txt  ← thermal_lib 라이브러리 생성
```

#### 2. 파일 이동
- `thermal/src/main.cpp` → `application/main.cpp` (삭제됨)
- `thermal/src/config.h` → `application/config.h`

#### 3. CMakeLists.txt 변경

**thermal/src/CMakeLists.txt**:
- `add_executable(thermal_rgb_streaming ...)` → `add_library(thermal_lib STATIC ...)`
- 메인 애플리케이션 관련 코드 제거
- thermal 라이브러리만 빌드

**application/CMakeLists.txt** (신규):
- `add_executable(humiro_fire_suppression ...)` 생성
- 모든 계층 라이브러리 링크:
  - `thermal_lib`
  - `targeting_lib`
  - `streaming_lib`
- ROS2 옵션 지원

#### 4. include 경로 수정
- `application/main.cpp`의 include 경로를 상대 경로로 수정:
  - `#include "camera_manager.h"` → `#include "../thermal/src/camera_manager.h"`

#### 5. 빌드 스크립트 추가
- `application/build.sh` - 모든 라이브러리와 애플리케이션을 순서대로 빌드

## 최종 구조

```
humiro_fire_suppression/
├── application/          # 통합 애플리케이션 (새로 추가)
│   ├── CMakeLists.txt
│   ├── main.cpp
│   ├── config.h
│   └── build.sh          # 빌드 스크립트
├── thermal/src/          # 열화상 데이터 취득 계층 (라이브러리)
│   ├── CMakeLists.txt   → thermal_lib 생성
│   ├── camera_manager.*
│   ├── thermal_processor.*
│   └── ...
├── lidar/src/            # 라이다 데이터 취득 계층
├── targeting/src/        # 타겟팅 계층 (라이브러리)
│   └── CMakeLists.txt   → targeting_lib 생성
└── streaming/src/        # 스트리밍 계층 (라이브러리)
    └── CMakeLists.txt   → streaming_lib 생성
```

## 빌드 방법

### 방법 1: 빌드 스크립트 사용 (권장)

```bash
cd ~/humiro_fire_suppression
./application/build.sh
```

### 방법 2: 수동 빌드

```bash
# 1. thermal 라이브러리
cd ~/humiro_fire_suppression/thermal/src/build
cmake ..
make -j$(nproc) thermal_lib

# 2. targeting 라이브러리
cd ~/humiro_fire_suppression/targeting/src/build
cmake ..
make -j$(nproc) targeting_lib

# 3. streaming 라이브러리
cd ~/humiro_fire_suppression/streaming/src/build
cmake ..
make -j$(nproc) streaming_lib

# 4. 메인 애플리케이션
cd ~/humiro_fire_suppression/application/build
cmake ..
make -j$(nproc) humiro_fire_suppression
```

## 빌드 결과

### 생성된 파일
- `application/build/humiro_fire_suppression` - 메인 실행 파일
- `thermal/src/build/libthermal_lib.a` - thermal 라이브러리 (489KB)
- `targeting/src/build/libtargeting_lib.a` - targeting 라이브러리 (658KB)
- `streaming/src/build/libstreaming_lib.a` - streaming 라이브러리 (387KB)

### 실행

```bash
cd ~/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

## 아키텍처 개선 효과

### 이전 구조의 문제
- `thermal/src/`에 메인 애플리케이션이 있어 명확하지 않음
- thermal 계층이 데이터 취득 + 통합 애플리케이션 둘 다 포함

### 개선된 구조
- ✅ **명확한 구조**: 통합 애플리케이션이 어디에 있는지 명확
- ✅ **책임 분리**: 
  - `thermal/` - 데이터 취득만 (라이브러리)
  - `application/` - 통합만 (실행 파일)
- ✅ **확장성**: 향후 다른 통합 애플리케이션 추가 용이

## 라이브러리 구조

모든 계층이 정적 라이브러리로 빌드됨:
- `thermal_lib` - 열화상 데이터 취득 (489KB)
- `targeting_lib` - 타겟팅/표시 (658KB)
- `streaming_lib` - 스트리밍 (387KB)

메인 애플리케이션은 이 라이브러리들을 링크하여 사용.

## 실행 파일 이름 변경

- **이전**: `thermal_rgb_streaming`
- **현재**: `humiro_fire_suppression`

더 명확한 이름으로 변경.

## 다음 단계

### 테스트 필요
- [ ] `humiro_fire_suppression` 실행 테스트
- [ ] 모든 기능 정상 동작 확인

### 문서 업데이트
- [ ] README 파일 업데이트
- [ ] 실행 가이드 업데이트

## 결론

메인 애플리케이션을 `application/` 폴더로 분리하여 아키텍처가 더 명확해졌습니다.

각 계층이 독립적인 라이브러리로 빌드되며, 통합 애플리케이션은 이들을 조합하여 사용하는 구조입니다.
