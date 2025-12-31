# 소스 파일 이동 상태

## 이동 완료된 파일

### Thermal (열화상 카메라)
- `thermal/src/`: C++ 소스 코드 (기존 ~/projects/Thermal/src/)
- `thermal/python/`: Python 스크립트 (기존 ~/projects/Thermal/python/)

### Targeting (타겟팅 시스템)
- `targeting/`: 타겟팅 관련 소스 파일들

### Throwing Mechanism (투척 메커니즘)
- `throwing_mechanism/`: 투척 메커니즘 관련 소스 파일들

### Navigation (네비게이션)
- `navigation/`: 네비게이션 관련 소스 파일들

## 확인 필요

다음 위치에서 추가 소스 파일이 있는지 확인하세요:
- `~/projects/Cluster_Drone/` 하위의 Python/C++ 파일들
- 다른 프로젝트 디렉토리의 관련 소스 파일들

## 다음 단계

1. 모든 소스 파일의 하드코딩된 경로 확인 및 수정
2. Python 스크립트의 import 경로 확인
3. C++ 프로젝트의 CMakeLists.txt 경로 확인

## 경로 수정 완료

### Thermal C++ 소스
- `config.h`: LOGO_PATH를 상대 경로로 변경
- `frame_compositor.cpp`: 로고 경로를 실행 파일 기준 상대 경로로 처리하도록 수정

### 다음 단계
1. Python 스크립트 내 하드코딩된 경로 확인 및 수정
2. C++ 빌드 테스트
3. 실행 파일 경로 테스트
