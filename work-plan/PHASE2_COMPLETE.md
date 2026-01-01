# Phase 2 완료 보고서

## 완료 날짜
2024년 (진행 중)

## 완료된 작업

### ✅ 1. streaming/ 폴더 구조 생성
- `streaming/src/` 폴더 생성
- `CMakeLists.txt` 작성 완료
- 정적 라이브러리 `streaming_lib` 빌드 성공

### ✅ 2. 파일 이동 완료

#### rtsp_server.*
- `thermal/src/rtsp_server.h` → `streaming/src/rtsp_server.h`
- `thermal/src/rtsp_server.cpp` → `streaming/src/rtsp_server.cpp`
- 상대 경로 수정 (`thread_safe_queue.h`, `config.h`, `utils.h`)

#### http_server.*
- `thermal/src/http_server.h` → `streaming/src/http_server.h`
- `thermal/src/http_server.cpp` → `streaming/src/http_server.cpp`
- 상대 경로 수정 (`thread_safe_queue.h`, `config.h`, `utils.h`)

### ✅ 3. streaming_manager.* 생성
- `streaming_manager.h` - 스트리밍 통합 관리 클래스 선언
- `streaming_manager.cpp` - RTSP 및 HTTP 서버 통합 관리 구현
- 초기화, 시작, 중지 기능 통합

### ✅ 4. main.cpp 수정
- `RTSPServer`, `HTTPServer` 직접 사용 제거
- `StreamingManager` 사용으로 변경
- 초기화, 시작, 중지 로직 단순화

### ✅ 5. CMakeLists.txt 업데이트

#### streaming/src/CMakeLists.txt
- GStreamer RTSP Server 설정
- libmicrohttpd 설정
- `streaming_lib` 정적 라이브러리 생성

#### thermal/src/CMakeLists.txt
- `rtsp_server.cpp`, `http_server.cpp` 제거
- `streaming_lib` 링크 추가
- `streaming_lib` 빌드 자동화

### ✅ 6. 빌드 테스트
- `streaming_lib` 정적 라이브러리 빌드 성공
- `thermal_rgb_streaming` 실행 파일 빌드 성공

## 파일 구조

### 새로 생성된 파일
```
streaming/src/
├── CMakeLists.txt
├── rtsp_server.h (이동)
├── rtsp_server.cpp (이동)
├── http_server.h (이동)
├── http_server.cpp (이동)
├── streaming_manager.h
└── streaming_manager.cpp
```

### 수정된 파일
```
thermal/src/
├── main.cpp (StreamingManager 사용)
└── CMakeLists.txt (streaming_lib 링크 추가)
```

### 제거된 파일 (thermal/src에서)
```
thermal/src/
├── rtsp_server.h (streaming/src로 이동)
├── rtsp_server.cpp (streaming/src로 이동)
├── http_server.h (streaming/src로 이동)
└── http_server.cpp (streaming/src로 이동)
```

## 아키텍처 개선 효과

### 이전 구조
- `thermal/src/`에 RTSP/HTTP 서버가 포함
- 데이터 취득과 스트리밍이 강하게 결합

### 개선된 구조
- **데이터 취득 계층**: `thermal/`, `lidar/`
- **기본 오버레이**: `ThermalBasicOverlay` (thermal/)
- **타겟팅 계층**: `targeting/` (독립적으로 테스트 가능)
- **스트리밍 계층**: `streaming/` (독립적으로 테스트 가능)

## 데이터 흐름

```
Camera → ThermalProcessor → ThermalData
                                        ↓
Lidar → LidarInterface → LidarPointCloud
                                        ↓
                        ThermalBasicOverlay (기본 오버레이)
                                        ↓
                        TargetingFrameCompositor (타겟팅 합성)
                                        ↓
                        StreamingManager
                                        ├─ RTSP Server
                                        └─ HTTP Server
```

## 다음 단계 (Phase 3)

1. **ROS2 통신 강화**
   - 계층 간 통신을 ROS2 토픽으로 전환
   - 동기화 메커니즘 구현 (타임스탬프 기반)

2. **추가 스트리밍 방식**
   - ROS2 이미지 토픽 발행 추가
   - 프레임 버퍼 관리 개선

3. **테스트 및 검증**
   - 각 계층 단위 테스트 작성
   - 통합 테스트
   - 성능 측정

## 주의사항

- `thermal/src/utils.cpp`의 `check_encoder` 함수가 GStreamer를 사용하므로, `thermal/src/CMakeLists.txt`에 기본 GStreamer 라이브러리 링크가 필요함
- `streaming_lib`는 `thermal/src/CMakeLists.txt`에서 자동으로 빌드되지만, 수동 빌드도 가능
- `rtsp_server.*`와 `http_server.*` 파일들은 `streaming/src/`로 이동했으므로, 참조 시 경로 주의

## 결론

Phase 2가 성공적으로 완료되었습니다. 스트리밍 계층이 명확히 분리되어:
- 코드 모듈화 향상
- 스트리밍 기능의 독립적 관리 가능
- 새로운 스트리밍 방식 추가 용이
- 테스트 및 유지보수 용이

을 달성했습니다.

