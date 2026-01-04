# Phase 1 진행 상황

## 완료된 작업

### ✅ 1. targeting/ 폴더 구조 생성
- `targeting/src/` 폴더 생성
- `CMakeLists.txt` 작성 완료
- 빌드 테스트 성공

### ✅ 2. 컴포넌트 분리 완료
- `distance_overlay.*` - 라이다 거리 오버레이 분리 완료
- `aim_indicator.*` - 조준 표시 분리 완료
- `hotspot_tracker.*` - Hotspot 추적 분리 완료
- `targeting_frame_compositor.*` - 타겟팅 정보 합성 클래스 생성 완료

### ✅ 3. 빌드 테스트
- `targeting_lib` 정적 라이브러리 빌드 성공

## 진행 중인 작업

### 🔄 4. thermal/의 frame_compositor 축소
- `thermal_basic_overlay.*` 생성 필요
- `frame_compositor`에서 타겟팅 관련 코드 제거
- `overlay_thermal`, `overlay_logo`만 유지

## 다음 단계

1. `thermal_basic_overlay.*` 생성
2. `frame_compositor` 간소화
3. `main.cpp`에서 `targeting_frame_compositor` 사용하도록 수정
4. 전체 빌드 테스트
5. 통합 테스트

## 참고사항

- 기존 `frame_compositor`는 호환성을 위해 유지하되, 내부적으로는 `thermal_basic_overlay`와 `targeting_frame_compositor`를 사용하도록 리팩토링할 수 있음
- 또는 완전히 분리하여 `main.cpp`에서 직접 두 클래스를 사용하도록 변경 가능

