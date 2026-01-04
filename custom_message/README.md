# 커스텀 MAVLink 메시지 송수신 라이브러리

**작성일**: 2026-01-03  
**목적**: QGC와 VIM4 간 화재 진압 미션 전용 커스텀 MAVLink 메시지를 송수신하는 C++ 라이브러리

> **📖 상세 문서**: [커스텀 MAVLink 메시지 설계 및 사용 가이드](../work-plan/019_MAVLINK_CUSTOM_MESSAGE.md)  
> 이 문서는 라이브러리 빠른 시작 가이드입니다. 전체 설계, 워크플로우, 상세 사용법은 위 링크를 참고하세요.

---

## 빠른 시작

### 빌드

```bash
cd custom_message
mkdir build && cd build
cmake ..
make
```

### 기본 사용 예제

```cpp
#include "custom_message/custom_message.h"

using namespace custom_message;

int main() {
    // 메시지 송수신기 생성
    CustomMessage msg_handler(14550, 14550, "0.0.0.0", "127.0.0.1");
    
    // 콜백 등록
    msg_handler.setFireMissionStatusCallback([](const FireMissionStatus& status) {
        std::cout << "미션 상태: " << status.status_text << std::endl;
    });
    
    // 시작
    msg_handler.start();
    
    // 메인 루프
    while (msg_handler.isRunning()) {
        // 작업 수행
    }
    
    msg_handler.stop();
    return 0;
}
```

---

## 지원 메시지

| 메시지 ID | 메시지 이름 | 방향 | 설명 |
|-----------|------------|------|------|
| 12900 | FIRE_MISSION_COMMAND | QGC → VIM4 | 화재 진압 미션 명령 |
| 12901 | FIRE_MISSION_STATUS | VIM4 → QGC | 미션 진행 상태 |
| 12902 | FIRE_THERMAL_DATA | QGC ↔ VIM4 | 열화상 데이터 |
| 12903 | FIRE_SUPPRESSION_RESULT | VIM4 → QGC | 발사 결과 |

---

## 주요 API

### 생성자

```cpp
CustomMessage(
    uint16_t receive_port = 14550,
    uint16_t send_port = 14550,
    const std::string& bind_address = "0.0.0.0",
    const std::string& target_address = "127.0.0.1",
    uint8_t system_id = 1,
    uint8_t component_id = 1
)
```

### 주요 메서드

**수신 콜백 등록**:
- `setFireMissionCommandCallback()` - 미션 명령
- `setFireMissionStatusCallback()` - 미션 상태
- `setFireThermalDataCallback()` - 열화상 데이터
- `setFireSuppressionResultCallback()` - 진압 결과

**송신**:
- `sendFireMissionCommand()` - 미션 명령 전송
- `sendFireMissionStatus()` - 미션 상태 전송
- `sendFireThermalData()` - 열화상 데이터 전송
- `sendFireSuppressionResult()` - 진압 결과 전송

**제어**:
- `start()` - 송수신 시작
- `stop()` - 송수신 중지
- `isRunning()` - 실행 상태 확인
- `getStatistics()` - 통계 정보

---

## CMake 통합

```cmake
add_subdirectory(custom_message)
target_link_libraries(your_target PRIVATE custom_message)
```

---

## 예제 프로그램

```bash
cd examples
g++ -std=c++17 -I../include example_usage.cpp -L../build -lcustom_message -pthread -o example_usage
./example_usage
```

---

## 상세 문서

전체 설계, 워크플로우, 상세 사용법은 다음 문서를 참고하세요:

- **[커스텀 MAVLink 메시지 설계 및 사용 가이드](../work-plan/019_MAVLINK_CUSTOM_MESSAGE.md)** ⭐
  - 전체 설계 및 워크플로우
  - 메시지 정의 및 사용 시나리오
  - 구현 참고사항
  - 라이브러리 상세 사용법
  - 문제 해결

---

## 참고 자료

- [MAVLink 프로토콜 문서](https://mavlink.io/)
- [RTK GPS 좌표 형식 가이드](../work-plan/020_RTK_GPS_COORDINATE_FORMAT.md)
- [목적지 도착 알림 가이드](../work-plan/021_DESTINATION_ARRIVAL_NOTIFICATION.md)
- [QGC 개발 가이드](../work-plan/016_QGC_DEVELOPMENT_GUIDE.md)

---

**작성자**: Humiro Fire Suppression Team  
**버전**: v2.0  
**작성일**: 2026-01-03
