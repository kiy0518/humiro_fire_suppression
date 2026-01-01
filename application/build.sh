#!/bin/bash
# 통합 애플리케이션 빌드 스크립트

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 옵션 확인
CLEAN_ONLY=false
BUILD_ONLY=false
CLEAN_BUILD=false

case "$1" in
    --clean-only|-clean)
        CLEAN_ONLY=true
        ;;
    --build-only|-build)
        BUILD_ONLY=true
        ;;
    --clean|-c)
        CLEAN_BUILD=true
        ;;
    --help|-h)
        echo "사용법: $0 [옵션]"
        echo ""
        echo "옵션:"
        echo "  (없음)          일반 빌드"
        echo "  --clean, -c     클린 후 빌드 (기존 빌드 디렉토리 삭제 후 빌드)"
        echo "  --clean-only    클린만 수행 (빌드하지 않음)"
        echo "  --build-only    빌드만 수행 (클린하지 않음)"
        echo "  --help, -h      도움말 표시"
        exit 0
        ;;
esac

echo "========================================="
echo "  Humiro Fire Suppression 빌드"
if [ "$CLEAN_BUILD" = true ]; then
    echo "  (클린 후 빌드 모드)"
elif [ "$CLEAN_ONLY" = true ]; then
    echo "  (클린만 수행)"
elif [ "$BUILD_ONLY" = true ]; then
    echo "  (빌드만 수행)"
fi
echo "========================================="
echo ""

# 클린 수행
if [ "$CLEAN_BUILD" = true ] || [ "$CLEAN_ONLY" = true ]; then
    echo "[클린] 기존 빌드 제거 중..."
    rm -rf "$PROJECT_ROOT/thermal/src/build"
    rm -rf "$PROJECT_ROOT/osd/src/build"
    rm -rf "$PROJECT_ROOT/targeting/src/build"
    rm -rf "$PROJECT_ROOT/streaming/src/build"
    rm -rf "$PROJECT_ROOT/application/build"
    echo "  ✓ 모든 빌드 디렉토리 삭제 완료"
    echo ""
    
    # 클린만 수행하는 경우 종료
    if [ "$CLEAN_ONLY" = true ]; then
        echo "========================================="
        echo "  클린 완료!"
        echo "========================================="
        exit 0
    fi
fi

# 빌드만 수행하는 경우 스킵
if [ "$BUILD_ONLY" = true ] && [ "$CLEAN_BUILD" != true ] && [ "$CLEAN_ONLY" != true ]; then
    # 빌드만 수행
    :
fi

# 1. thermal 라이브러리 빌드
STEP_NUM=1
TOTAL_STEPS=5
echo "[${STEP_NUM}/${TOTAL_STEPS}] thermal 라이브러리 빌드..."
cd "$PROJECT_ROOT/thermal/src"
mkdir -p build
cd build
cmake .. > /dev/null
make -j$(nproc) thermal_lib
echo "  ✓ thermal_lib 빌드 완료"
echo ""

# 2. OSD 라이브러리 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] OSD 라이브러리 빌드..."
cd "$PROJECT_ROOT/osd/src"
mkdir -p build
cd build
cmake .. > /dev/null
make -j$(nproc) osd_lib
echo "  ✓ osd_lib 빌드 완료"
echo ""

# 3. targeting 라이브러리 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] targeting 라이브러리 빌드..."
cd "$PROJECT_ROOT/targeting/src"
mkdir -p build
cd build
cmake .. > /dev/null
make -j$(nproc) targeting_lib
echo "  ✓ targeting_lib 빌드 완료"
echo ""

# 4. streaming 라이브러리 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] streaming 라이브러리 빌드..."
cd "$PROJECT_ROOT/streaming/src"
mkdir -p build
cd build
cmake .. > /dev/null
make -j$(nproc) streaming_lib
echo "  ✓ streaming_lib 빌드 완료"
echo ""

# 5. 메인 애플리케이션 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] 메인 애플리케이션 빌드..."
cd "$PROJECT_ROOT/application"
mkdir -p build
cd build
cmake .. > /dev/null

# 라이브러리 파일의 타임스탬프를 갱신하여 CMake가 변경을 감지하도록 함
touch "$PROJECT_ROOT/thermal/src/build/libthermal_lib.a" 2>/dev/null || true
touch "$PROJECT_ROOT/osd/src/build/libosd_lib.a" 2>/dev/null || true
touch "$PROJECT_ROOT/targeting/src/build/libtargeting_lib.a" 2>/dev/null || true
touch "$PROJECT_ROOT/streaming/src/build/libstreaming_lib.a" 2>/dev/null || true

# 강제로 재빌드 (라이브러리 변경을 확실히 반영)
make -j$(nproc) humiro_fire_suppression
echo "  ✓ humiro_fire_suppression 빌드 완료"
echo ""

echo "========================================="
echo "  빌드 완료!"
echo "========================================="
echo ""
echo "실행 파일: $PROJECT_ROOT/application/build/humiro_fire_suppression"
echo ""

