# Work Plan - 작업 계획서 v4.0

이 폴더는 Humiro Fire Suppression 프로젝트의 전체 작업 계획을 포함합니다.

**버전**: v4.0 (targeting 재정의)  
**작성일**: 2025-12-31

---

## 📁 폴더 구조

```
work-plan/
├── README.md                      # 이 파일
├── PROJECT_MASTER_PLAN.md         # 프로젝트 마스터 플랜 (v4.0)
├── DEVELOPMENT_STATUS.md          # 개발 현황 상세 보고서 (v5.0) ⭐ NEW
├── DETAILED_DEVELOPMENT_PLAN.md   # 상세 개발 계획서 (v5.0) ⭐ NEW
├── QUICK_START_GUIDE.md           # 빠른 시작 가이드 (v4.0)
├── LIDAR_IMPLEMENTATION_SUMMARY.md # LiDAR 구현 요약
├── UART_CONFIGURATION_UPDATE.md    # UART 설정 업데이트
├── FOLDER_RESTRUCTURE_PLAN.md     # 폴더 재구조화 계획
└── WORK_PLAN_UPDATE_SUMMARY.md     # 작업 계획 업데이트 요약
```

---

## 🎯 v4.0 핵심 변경사항

### targeting 의미 재정의 ⭐

**이전 (v3.0 - 잘못된 이해)**:
- targeting = 거리 측정 + 탄도 계산

**현재 (v4.0 - 올바른 이해)**:
- **targeting** = **핫스팟 트래킹 + 드론 위치 제어** (정조준)
- **lidar** = **거리 측정** (10m 도착 판단)

### 폴더 구조 변경

**Before (v3.0)**:
```
targeting/
└── lidar_integration/  # 잘못된 이름
```

**After (v4.0)**:
```
lidar/                  # 거리 측정 (완료)
targeting/              # 핫스팟 트래킹 + 드론 제어 (새로 설계)
```

---

## 📋 문서 설명

### 1. DEVELOPMENT_STATUS.md (v5.0) ⭐ NEW
**개발 현황 상세 보고서 - 현재 코드 상태 완전 분석**

포함 내용:
- 전체 진행 상황 요약 (40% 완료)
- Phase별 상세 구현 상태
  - ✅ Phase 1: Thermal System (완료, 2,665 LOC)
  - ✅ Phase 2: LiDAR System (완료, 1,188 LOC, HW 테스트 대기)
  - ⏳ Phase 3: Targeting System (미구현, 설계 완료)
  - ⏳ Phase 4: Throwing Mechanism (미구현, 설계 완료)
  - ⏳ Phase 5: Navigation (미구현, 미설계)
- 구현된 파일 목록
- 코드 통계 (3,853 / 8,000 LOC, 48%)
- 기술 스택 현황
- 다음 우선순위 작업

**이 문서를 읽어야 하는 경우**:
- 현재 코드가 어디까지 진행되었는지 정확히 알고 싶을 때 ⭐
- 각 모듈의 구현 상태를 확인하고 싶을 때
- 다음에 무엇을 개발해야 할지 결정할 때
- 전체 프로젝트 진행률을 파악하고 싶을 때

### 2. DETAILED_DEVELOPMENT_PLAN.md (v5.0) ⭐ NEW
**상세 개발 계획서 - 앞으로의 개발 계획**

포함 내용:
- Phase별 상세 개발 계획
  - Phase 3.1: Targeting 기본 구조 (1주)
  - Phase 3.2: 드론 제어 연동 (1주)
  - Phase 3.3: GCS 통합 (1주)
  - Phase 4: Throwing Mechanism (3주)
- 기술적 세부사항 (코드 구조, 클래스 설계)
- 개발 로드맵 (2025년 1월 ~ 4월)
- 통합 계획 (단계별)
- 테스트 계획
- 리스크 관리

**이 문서를 읽어야 하는 경우**:
- 앞으로의 개발 계획을 알고 싶을 때 ⭐
- 각 Phase의 구체적인 작업 항목을 확인하고 싶을 때
- 개발 일정을 계획하고 싶을 때
- 기술적 구현 방법을 알고 싶을 때

### 3. PROJECT_MASTER_PLAN.md (v4.0)
**프로젝트 마스터 플랜 - 전체 프로젝트 개요**

포함 내용:
- **화재 진압 시나리오** 반영
- Phase 1: thermal/src (완료) - 핫스팟 감지
- Phase 2: lidar/ (완료) - 거리 측정
- **Phase 3: targeting/ (다음)** - 핫스팟 트래킹 + 드론 제어 ⭐
- Phase 4: throwing_mechanism/ - 발사
- 영상 오버레이 통합 설명
- GCS 격발 신호 처리

**주요 변경사항** (v3.0 → v4.0):
- ✅ targeting 의미 재정의
- ✅ 화재 진압 시나리오 반영
- ✅ GCS 격발 신호 → targeting 활성화
- ✅ 정조준 (LOCKED) 개념 추가
- ✅ 영상 오버레이 통합 (항상 vs GCS 신호 후)

**이 문서를 읽어야 하는 경우**:
- targeting이 무엇인지 이해하고 싶을 때
- 화재 진압 시나리오를 알고 싶을 때
- 프로젝트 전체 개념을 이해하고 싶을 때

### 4. QUICK_START_GUIDE.md (v4.0)
**빠른 참고 가이드**

포함 내용:
- 프로젝트 현황 (50% 완료)
- targeting vs lidar 구분
- 화재 진압 시나리오 요약
- 빌드 및 실행 명령어
- C++ 코드 예제
- 영상 오버레이 설명

**이 문서를 읽어야 하는 경우**:
- 빠르게 현재 상태를 파악하고 싶을 때
- targeting과 lidar의 차이를 알고 싶을 때
- 빌드 명령어를 찾을 때
- 간단한 C++ 예제가 필요할 때

---

## 🚀 현재 우선순위

### ✅ 완료 (P0)
1. thermal/src (2,665 LOC) - 핫스팟 감지 ✅
2. lidar/ (1,188 LOC) - 거리 측정 ✅

### ⏳ 다음 작업 (P0)
3. **targeting/** ← 현재 작업 ⭐
   - Phase 1: 기본 구조 (1주)
     - hotspot_tracker.cpp
     - targeting_overlay.cpp
     - targeting_manager.cpp
   
   - Phase 2: 드론 제어 (1주)
     - drone_position_controller.cpp
     - PX4 연동
   
   - Phase 3: GCS 통합 (1주)
     - ROS2 토픽 구독
     - 전체 시스템 테스트

### P1 (추후)
4. throwing_mechanism/ - 발사 메커니즘

---

## 📊 개발 진행 상황

```
Phase 1: Thermal System      [██████████] 100%  ✅ 완료
Phase 2: LiDAR System        [██████████] 100%  ✅ 완료 (HW 테스트 대기)
Phase 3: Targeting System    [░░░░░░░░░░]   0%  ⏳ 다음 (핵심)
Phase 4: Throwing Mechanism  [░░░░░░░░░░]   0%  ⏳ 설계 완료
Phase 5: Navigation          [░░░░░░░░░░]   0%  ⏳ 미설계

전체 프로젝트 진행률: 40% (5단계 중 2단계 완료)
코드 작성: 3,853 LOC / 예상 8,000 LOC (48% 완료)
```

**상세 현황**: `DEVELOPMENT_STATUS.md` 참조

---

## 🎯 화재 진압 시나리오

```
Phase 1: 접근
  - 열원까지 10m 지점 자율 비행
  - lidar로 거리 모니터링
  ↓
Phase 2: 대기
  - 10m 도착 (lidar 확인)
  - GCS 격발 신호 대기
  ↓
Phase 3: Targeting 활성화 ⭐
  - GCS 격발 신호 수신
  - targeting/ 모듈 활성화
  - 핫스팟 트래킹 시작
  - 드론 상하좌우 미세 조정
  - 화면 중심에 핫스팟 유지
  - 정조준 완료 (LOCKED)
  ↓
Phase 4: 발사
  - throwing_mechanism/ 발사 실행
```

---

## 🖼️ 영상 오버레이 통합

### 항상 표시
1. **거리 오버레이** (`lidar/`)
   - LINE SHAPE 표시
   - 색상 코딩 (녹색 9-11m, 빨간색 <9m, 파란색 >11m)
   - 거리 텍스트

2. **핫스팟 감지** (`thermal/src/`)
   - 핫스팟 위치 (원 또는 마커)
   - 온도 정보

### GCS 신호 후에만 표시
3. **트래킹 상태** (`targeting/`)
   - "TRACKING ACTIVE" 텍스트
   - 화면 중심 십자선
   - 오차 벡터 (중심 → 핫스팟)
   - "LOCKED" 표시 (정조준 완료 시)

---

## 📝 업데이트 로그

### 2025-01-XX (v5.0) - 현황 기반 상세 계획 작성 ⭐ NEW
- **핵심 추가**: 현재 코드 상태 완전 분석
  - `DEVELOPMENT_STATUS.md`: 구현된 파일 목록, 코드 통계, 기술 스택
  - `DETAILED_DEVELOPMENT_PLAN.md`: Phase별 상세 계획, 개발 로드맵, 리스크 관리
- 실제 코드 파일 확인 및 분석
- Phase 5 (Navigation) 추가
- 전체 예상 코드량 수정 (5,953 → 8,000 LOC)
- 개발 일정 및 로드맵 구체화

### 2025-12-31 (v4.0) - targeting 재정의
- **핵심 변경**: targeting 의미 재정의
  - targeting = 핫스팟 트래킹 + 드론 제어
  - lidar = 거리 측정
- 폴더 구조 변경:
  - targeting/lidar_integration → lidar/
  - targeting/ (새로 정의)
- 화재 진압 시나리오 반영
- GCS 격발 신호 처리 추가
- 영상 오버레이 통합 설계
- PROJECT_MASTER_PLAN.md v4.0
- QUICK_START_GUIDE.md v4.0

### 2025-12-31 (v3.0) - 실제 구현 반영
- 프로젝트 단순화 완료
- thermal/src가 핫스팟 감지 포함 명시
- 불필요한 폴더 제거
- 10m 고정 거리, 고정 각도 설계

### 2025-12-31 (v2.0) - C++ 우선 정책
- C++17 우선 개발 정책 확립
- LiDAR C++ 통합 완료 (1,188 LOC)

### 2025-12-31 (v1.0) - 초기 작성
- 프로젝트 마스터 플랜 작성
- work-plan 폴더 생성

---

## 🔧 개발 가이드라인

### 언어 선택
- **C++17**: 모든 핵심 모듈
  - ✅ thermal/src
  - ✅ lidar/
  - ⏳ targeting/
  - ⏳ throwing_mechanism/

### 빌드 시스템
- **C++**: CMake + Make
- **테스트**: Google Test

### 코드 스타일
- **C++**: Google C++ Style Guide

---

## 🎓 학습 경로

### targeting 이해하기
1. `targeting/README.md` - 핫스팟 트래킹 + 드론 제어 상세 설계
2. `PROJECT_MASTER_PLAN.md` - 화재 진압 시나리오
3. `thermal/src/thermal_processor.cpp` - 핫스팟 감지 구현 (이미 완료)
4. `lidar/src/` - 거리 측정 구현 (이미 완료)

### targeting과 lidar 구분
- **thermal/src**: 핫스팟 **감지**
- **lidar/**: **거리** 측정 (10m 도착 판단)
- **targeting/**: 핫스팟 **트래킹** + 드론 **제어**
- **throwing_mechanism/**: **발사**

---

## 📚 참고 자료

### 프로젝트 문서
- `CLAUDE.md`: C++17 우선 정책, 시스템 아키텍처
- `thermal/src/README.md`: 열화상 시스템
- `lidar/README.md`: LiDAR 거리 측정
- `targeting/README.md`: 핫스팟 트래킹 + 드론 제어
- `throwing_mechanism/README.md`: 발사 메커니즘

### 외부 리소스
- C++ Reference: https://en.cppreference.com/
- CMake: https://cmake.org/
- PX4 Offboard: https://docs.px4.io/main/en/flight_modes/offboard.html
- ROS2 Humble: https://docs.ros.org/en/humble/

---

## 🔑 핵심 이해사항

### targeting ≠ 거리 측정 ⭐

**잘못된 이해 (v3.0)**:
- targeting = lidar 거리 측정

**올바른 이해 (v4.0)**:
- **targeting = 핫스팟 트래킹 + 드론 제어**
- **lidar = 거리 측정**

### 역할 분리
```
thermal/src/     → 핫스팟 감지
lidar/           → 거리 측정
targeting/       → 핫스팟 트래킹 + 드론 제어
throwing_mech/   → 발사
```

### GCS 격발 신호
```
10m 도착 → GCS 신호 대기 → 격발 신호 수신 → targeting 활성화
```

---

**다음 우선순위**: 
1. LD19 LiDAR 하드웨어 테스트
2. targeting/ Phase 3.1 구현 시작

**작성자**: Claude Code Assistant  
**버전**: v5.0 (현황 기반 상세 계획)  
**마지막 업데이트**: 2025-01-XX  
**다음 리뷰 예정**: targeting/ Phase 3.1 완료 시 (약 1주 후)

---

## 🎯 문서 선택 가이드

### 현재 코드 상태를 알고 싶다면
→ **`DEVELOPMENT_STATUS.md`** ⭐

### 앞으로의 개발 계획을 알고 싶다면
→ **`DETAILED_DEVELOPMENT_PLAN.md`** ⭐

### 프로젝트 전체 개념을 이해하고 싶다면
→ **`PROJECT_MASTER_PLAN.md`**

### 빠르게 참고하고 싶다면
→ **`QUICK_START_GUIDE.md`**
