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
    echo "[클린] 관련 서비스 중지 중..."
    for SVC in micro-ros-agent mavlink-router; do
        if systemctl is-active --quiet "$SVC" 2>/dev/null; then
            sudo systemctl stop "$SVC"
            echo "  → $SVC 서비스 중지"
        fi
    done

    echo "[클린] 기존 프로세스 종료 중..."
    # 실행 중인 humiro_fire_suppression 프로세스 종료
    EXEC_NAME="humiro_fire_suppression"
    PIDS=$(pgrep -f "$EXEC_NAME" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        echo "  → 실행 중인 프로세스 발견: $PIDS"
        for PID in $PIDS; do
            if [ "$PID" != "$$" ]; then  # 자기 자신 제외
                echo "    → 프로세스 종료: PID $PID"
                kill -TERM "$PID" 2>/dev/null || true
                sleep 0.5
                # 강제 종료 (여전히 실행 중이면)
                if kill -0 "$PID" 2>/dev/null; then
                    echo "    → 강제 종료: PID $PID"
                    kill -KILL "$PID" 2>/dev/null || true
                fi
            fi
        done
        sleep 1
        echo "  ✓ 프로세스 종료 완료"
    else
        echo "  ✓ 실행 중인 프로세스 없음"
    fi
    echo ""
    
    echo "[클린] 기존 빌드 제거 중..."
    rm -rf "$PROJECT_ROOT/thermal/src/build"
    rm -rf "$PROJECT_ROOT/osd/src/build"
    rm -rf "$PROJECT_ROOT/targeting/src/build"
    rm -rf "$PROJECT_ROOT/streaming/src/build"
    rm -rf "$PROJECT_ROOT/navigation/src/offboard/build"
    rm -rf "$PROJECT_ROOT/navigation/build"
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
TOTAL_STEPS=6
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

# ROS2 환경 로드 (fc_bridge + 메인 애플리케이션 공통)
# CMake가 ROS2 패키지를 찾을 수 있도록 환경 변수 설정
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash > /dev/null 2>&1
fi
if [ -f "$PROJECT_ROOT/workspaces/micro_ros_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/workspaces/micro_ros_ws/install/setup.bash" > /dev/null 2>&1
fi
if [ -f "$PROJECT_ROOT/workspaces/px4_ros2_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/workspaces/px4_ros2_ws/install/setup.bash" > /dev/null 2>&1
    export CMAKE_PREFIX_PATH="$PROJECT_ROOT/workspaces/px4_ros2_ws/install:$CMAKE_PREFIX_PATH"
fi
# humiro_msgs (편대 비행 메시지)
if [ -f "$PROJECT_ROOT/install/humiro_msgs/share/humiro_msgs/local_setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash" > /dev/null 2>&1
    export CMAKE_PREFIX_PATH="$PROJECT_ROOT/install:$CMAKE_PREFIX_PATH"
fi

# ROS2 자동 감지 및 활성화
ENABLE_ROS2_FLAG=""
if command -v ros2 > /dev/null 2>&1; then
    ENABLE_ROS2_FLAG="-DENABLE_ROS2=ON"
    echo "[ROS2] 감지됨: 자동 활성화"
else
    ENABLE_ROS2_FLAG="-DENABLE_ROS2=OFF"
    echo "[ROS2] 미설치: 비활성화"
fi
echo ""

# 5. fc_bridge 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] fc_bridge 빌드..."
cd "$PROJECT_ROOT/navigation/src/offboard"
mkdir -p build
cd build
cmake .. > /dev/null
make -j$(nproc) fc_bridge
# wrapper가 참조하는 경로에 복사
mkdir -p "$PROJECT_ROOT/navigation/build/offboard_control"
cp -f fc_bridge "$PROJECT_ROOT/navigation/build/offboard_control/fc_bridge"
echo "  ✓ fc_bridge 빌드 완료"
echo ""

# 6. 메인 애플리케이션 빌드
STEP_NUM=$((STEP_NUM + 1))
echo "[${STEP_NUM}/${TOTAL_STEPS}] 메인 애플리케이션 빌드..."
cd "$PROJECT_ROOT/application"
mkdir -p build
cd build

cmake .. $ENABLE_ROS2_FLAG > /dev/null

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

# =============================================
# 빌드 후 환경 검증 (SITL 연결 문제 방지)
# =============================================
echo "[환경 검증] SITL 연결 필수 설정 확인..."
VERIFY_FAIL=false

# 1) ROS2 활성화 확인
if strings "$PROJECT_ROOT/application/build/humiro_fire_suppression" | grep -q "OffboardManager initialized"; then
    echo "  ✓ ROS2 활성화 확인 (OffboardManager 포함)"
else
    echo "  ✗ ROS2 비활성화! cmake .. -DENABLE_ROS2=ON 필요"
    VERIFY_FAIL=true
fi

# 2) FastDDS 프로파일 확인 (.bashrc)
CORRECT_PROFILE="fastdds_eth0_only.xml"
WRONG_PROFILE="fastrtps_profile.xml"
if [ -f "$HOME/.bashrc" ]; then
    if grep -q "FASTRTPS_DEFAULT_PROFILES_FILE" "$HOME/.bashrc"; then
        if grep "FASTRTPS_DEFAULT_PROFILES_FILE" "$HOME/.bashrc" | grep -q "$WRONG_PROFILE"; then
            echo "  ✗ .bashrc에 잘못된 FastDDS 프로파일: $WRONG_PROFILE"
            echo "    → 자동 수정: $CORRECT_PROFILE"
            sed -i "s|$WRONG_PROFILE|$CORRECT_PROFILE|g" "$HOME/.bashrc"
            echo "  ✓ .bashrc 수정 완료"
        else
            echo "  ✓ .bashrc FastDDS 프로파일 정상"
        fi
    fi
fi

# 3) wrapper 스크립트 FastDDS 프로파일 확인
WRAPPER="$PROJECT_ROOT/scripts/runtime/humiro_fire_suppression_wrapper.sh"
if [ -f "$WRAPPER" ]; then
    if grep -q "$WRONG_PROFILE" "$WRAPPER"; then
        echo "  ✗ wrapper.sh에 잘못된 FastDDS 프로파일: $WRONG_PROFILE"
        echo "    → 자동 수정: $CORRECT_PROFILE"
        sed -i "s|$WRONG_PROFILE|$CORRECT_PROFILE|g" "$WRAPPER"
        echo "  ✓ wrapper.sh 수정 완료"
    else
        echo "  ✓ wrapper.sh FastDDS 프로파일 정상"
    fi
fi

# 4) stale colcon install 디렉토리 정리
if [ -d "$PROJECT_ROOT/install/offboard_control" ]; then
    echo "  ✗ stale 디렉토리 발견: install/offboard_control"
    echo "    → 자동 삭제"
    rm -rf "$PROJECT_ROOT/install/offboard_control"
    echo "  ✓ 삭제 완료"
else
    echo "  ✓ stale colcon 디렉토리 없음"
fi

# 5) systemd 서비스 FastDDS 프로파일 확인
SERVICE_FILE="/etc/systemd/system/humiro-fire-suppression.service"
if [ -f "$SERVICE_FILE" ]; then
    if grep -q "$WRONG_PROFILE" "$SERVICE_FILE"; then
        echo "  ✗ systemd 서비스에 잘못된 FastDDS 프로파일!"
        echo "    → 수동 수정 필요: sudo vi $SERVICE_FILE"
        VERIFY_FAIL=true
    else
        echo "  ✓ systemd 서비스 FastDDS 프로파일 정상"
    fi
fi

if [ "$VERIFY_FAIL" = true ]; then
    echo ""
    echo "  ⚠ 일부 검증 실패 - 위 항목을 확인하세요"
else
    echo ""
    echo "  ✓ 모든 환경 검증 통과 - SITL 연결 준비 완료"
fi
echo ""

