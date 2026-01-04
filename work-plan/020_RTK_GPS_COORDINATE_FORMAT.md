# RTK GPS 좌표 형식 가이드

**작성일**: 2026-01-03  
**목적**: RTK GPS 좌표를 MAVLink 커스텀 메시지에서 사용하는 방법

---

## 개요

RTK GPS는 센티미터 단위의 높은 정밀도를 제공하지만, MAVLink 프로토콜 표준에 따라 `int32_t` 형식(degrees * 1e7)으로 표현하는 것이 적절합니다.

---

## 좌표 표현 방식

### MAVLink 표준 형식

MAVLink 프로토콜에서는 GPS 좌표를 다음과 같이 표현합니다:

```cpp
int32_t latitude;   // degrees * 1e7
int32_t longitude;  // degrees * 1e7
```

### 정밀도 분석

**int32_t (degrees * 1e7) 형식의 정밀도**:
- 최소 단위: 0.0000001도
- 적도 기준 거리: 약 1.1cm
- 위도 37도 기준 거리: 약 0.9cm

**RTK GPS 정밀도**:
- 일반 RTK: 1-2cm
- 고정밀 RTK: 0.5-1cm

**결론**: int32_t 형식으로 RTK GPS의 정밀도를 충분히 표현 가능

---

## 좌표 변환 예제

### 부동소수점 → int32_t 변환

```cpp
// RTK GPS에서 획득한 좌표 (부동소수점)
double rtk_lat = 37.5665000;  // 위도
double rtk_lon = 126.9780000; // 경도

// MAVLink 형식으로 변환
int32_t target_lat = static_cast<int32_t>(rtk_lat * 1e7);
int32_t target_lon = static_cast<int32_t>(rtk_lon * 1e7);

// 결과: target_lat = 375665000, target_lon = 1269780000
```

### int32_t → 부동소수점 변환

```cpp
// MAVLink 메시지에서 수신한 좌표
int32_t target_lat = 375665000;
int32_t target_lon = 1269780000;

// 부동소수점으로 변환
double lat = target_lat / 1e7;
double lon = target_lon / 1e7;

// 결과: lat = 37.5665000, lon = 126.9780000
```

---

## RTK GPS 사용 예제

### QGC에서 RTK GPS 좌표 전송

```cpp
// RTK GPS로 획득한 정밀 좌표
double fire_lat = 37.5665123;  // RTK GPS 위도 (센티미터 정밀도)
double fire_lon = 126.9780456; // RTK GPS 경도 (센티미터 정밀도)
double fire_alt = 50.5;        // 고도 (m)

// MAVLink 메시지 형식으로 변환
FireMissionStart mission_start;
mission_start.target_lat = static_cast<int32_t>(fire_lat * 1e7);
mission_start.target_lon = static_cast<int32_t>(fire_lon * 1e7);
mission_start.target_alt = fire_alt;

// 메시지 전송
msg_handler.sendFireMissionStart(mission_start);
```

### VIM4에서 RTK GPS 좌표 수신 및 사용

```cpp
// 메시지 수신
msg_handler.setFireMissionStartCallback([](const FireMissionStart& start) {
    // int32_t → double 변환
    double target_lat = start.target_lat / 1e7;
    double target_lon = start.target_lon / 1e7;
    double target_alt = start.target_alt;
    
    // RTK GPS 정밀도 유지
    // 예: target_lat = 37.5665123 (센티미터 정밀도)
    
    // 드론 이동 명령
    drone.action.goto_location(target_lat, target_lon, target_alt, 0);
});
```

---

## 정밀도 비교

| 좌표 형식 | 정밀도 | RTK GPS 지원 |
|----------|--------|-------------|
| `float` (degrees) | 약 7자리 | 가능하나 부정확 |
| `double` (degrees) | 약 15자리 | 가능 |
| `int32_t` (degrees * 1e7) | 0.0000001도 ≈ 1.1cm | **권장** |

**int32_t 형식의 장점**:
- MAVLink 표준과 호환
- 메모리 효율적 (4바이트)
- 정밀도 충분 (RTK GPS 지원)
- 정수 연산으로 빠름

---

## 주의사항

### 1. 오버플로우 방지

```cpp
// 잘못된 예: 오버플로우 가능
int32_t lat = static_cast<int32_t>(lat_degrees * 1e7);

// 올바른 예: 범위 확인
if (lat_degrees >= -90.0 && lat_degrees <= 90.0) {
    int32_t lat = static_cast<int32_t>(lat_degrees * 1e7);
} else {
    // 오류 처리
}
```

### 2. 반올림 처리

```cpp
// RTK GPS 좌표 변환 시 반올림
double rtk_lat = 37.56651234;
int32_t target_lat = static_cast<int32_t>(std::round(rtk_lat * 1e7));
```

### 3. 정밀도 손실 방지

```cpp
// 부동소수점 연산 후 변환 (정밀도 유지)
double precise_lat = rtk_gps.getLatitude();  // RTK GPS에서 직접 획득
int32_t mavlink_lat = static_cast<int32_t>(precise_lat * 1e7);
```

---

## MAVLink 표준 참고

MAVLink 프로토콜에서 사용하는 좌표 형식:

| 메시지 | 필드 | 타입 | 설명 |
|--------|------|------|------|
| GLOBAL_POSITION_INT | lat, lon | int32_t | degrees * 1e7 |
| MISSION_ITEM_INT | x, y | int32_t | degrees * 1e7 |
| SET_POSITION_TARGET_GLOBAL_INT | lat_int, lon_int | int32_t | degrees * 1e7 |

**결론**: 커스텀 메시지도 동일한 형식을 사용하여 MAVLink 표준과 호환성을 유지합니다.

---

## 실전 예제

### RTK GPS 좌표를 사용한 미션 시작

```cpp
#include "custom_message/custom_message.h"
#include <cmath>

// RTK GPS로 화재 위치 획득
class RTKFireLocation {
public:
    double latitude;   // RTK GPS 위도
    double longitude;  // RTK GPS 경도
    double altitude;   // 고도 (m)
    
    // MAVLink 메시지로 변환
    FireMissionStart toMissionStart(uint8_t system_id, uint8_t component_id,
                                   bool auto_fire, uint8_t max_projectiles) {
        FireMissionStart msg;
        msg.target_system = system_id;
        msg.target_component = component_id;
        msg.target_lat = static_cast<int32_t>(std::round(latitude * 1e7));
        msg.target_lon = static_cast<int32_t>(std::round(longitude * 1e7));
        msg.target_alt = altitude;
        msg.auto_fire = auto_fire ? 1 : 0;
        msg.max_projectiles = max_projectiles;
        return msg;
    }
};

// 사용 예제
RTKFireLocation fire_location;
fire_location.latitude = 37.5665123;   // RTK GPS 정밀 좌표
fire_location.longitude = 126.9780456;
fire_location.altitude = 50.5;

FireMissionStart mission = fire_location.toMissionStart(1, 1, true, 3);
msg_handler.sendFireMissionStart(mission);
```

---

## 참고 자료

- [MAVLink 공식 문서 - GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)
- [MAVLink 공식 문서 - MISSION_ITEM_INT](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT)
- [커스텀 메시지 설계](./019_MAVLINK_CUSTOM_MESSAGE.md)

---

**작성자**: Humiro Fire Suppression Team  
**버전**: v1.0  
**작성일**: 2026-01-03

