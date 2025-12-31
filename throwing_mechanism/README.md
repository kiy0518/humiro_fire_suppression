# Throwing Mechanism - 발사 메커니즘

소화탄 투척 장치 GPIO 제어 모듈

## 개요

**목적**: 10m 고정 거리에서 고정 각도로 소화탄 발사

**현재 상태**: 미구현

## 시스템 요구사항

### 하드웨어
- **서보 모터**: 발사 각도 조정 (테스트를 통해 최적 각도 결정)
- **발사 메커니즘**: GPIO 트리거
- **전원**: 5V/12V (서보 사양에 따름)

### 제어 방식
- **거리**: 10m 고정
- **각도**: 테스트를 통해 결정된 고정 각도
- **탄도 계산**: 불필요 (고정 각도 사용)

## 구현 계획

### Phase 1: 서보 제어 (C++)
**파일**: `servo_controller.h/cpp`

```cpp
class ServoController {
public:
    ServoController(int gpio_pin, int pwm_channel);
    
    // 서보 각도 설정 (테스트로 결정된 고정값)
    bool setAngle(float angle_deg);
    
    // 발사 준비 위치로 이동
    bool moveToFirePosition();
    
    // 초기 위치로 복귀
    bool reset();
    
private:
    int gpio_pin_;
    int pwm_channel_;
    float fire_angle_;  // 테스트로 결정된 최적 각도
};
```

### Phase 2: GPIO 트리거 (C++)
**파일**: `fire_trigger.h/cpp`

```cpp
class FireTrigger {
public:
    FireTrigger(int trigger_gpio);
    
    // 발사 실행
    bool fire();
    
    // 안전 체크
    bool isSafe();
    
private:
    int trigger_gpio_;
    bool is_armed_;
};
```

### Phase 3: 통합 제어 (C++)
**파일**: `throwing_controller.h/cpp`

```cpp
class ThrowingController {
public:
    ThrowingController();
    
    // 초기화
    bool initialize();
    
    // 발사 준비
    bool prepare();
    
    // 발사 실행 (10m 고정 거리)
    bool execute();
    
    // 상태 확인
    bool isReady();
    
private:
    ServoController servo_;
    FireTrigger trigger_;
    
    // 테스트로 결정된 파라미터
    const float FIRE_ANGLE = 45.0f;  // 예: 45도 (테스트로 조정)
    const float TARGET_DISTANCE = 10.0f;  // 10m 고정
};
```

## 데이터 흐름

```
[thermal/src/] → 화재 감지 (핫스팟)
        ↓
[targeting/lidar_integration/] → 거리 확인 (~10m)
        ↓
[throwing_mechanism/] → 발사 실행
    ├─ ServoController: 고정 각도로 이동
    └─ FireTrigger: 발사
```

## 개발 순서

### 1주차: 하드웨어 준비 및 테스트
1. 서보 모터 선정
2. GPIO 핀 매핑
3. 수동 테스트로 최적 각도 찾기

### 2주차: 소프트웨어 구현
1. `servo_controller.cpp` 구현
   - PWM 제어
   - 각도 설정
2. `fire_trigger.cpp` 구현
   - GPIO 제어
   - 안전 체크

### 3주차: 통합 및 테스트
1. `throwing_controller.cpp` 구현
2. thermal + LiDAR + throwing 통합
3. 실제 발사 테스트

## VIM4 GPIO 사용

### GPIO 핀맵
```
서보 PWM: GPIO3_D3 (Pin 7)
트리거: GPIO3_D4 (Pin 11)
전원: 5V (Pin 2)
GND: GND (Pin 6)
```

### 예제 코드
```cpp
// 서보 초기화
ServoController servo(GPIO_PIN_7, PWM_CHANNEL_0);
servo.setAngle(45.0f);  // 테스트로 결정된 각도

// 발사 트리거 초기화
FireTrigger trigger(GPIO_PIN_11);

// 발사 실행
if (servo.moveToFirePosition() && trigger.fire()) {
    std::cout << "발사 성공!" << std::endl;
}
```

## 안전 고려사항

1. **최소 거리**: 5m 이상 (안전 거리)
2. **최대 거리**: 15m 이하 (유효 사거리)
3. **발사 전 확인**:
   - LiDAR 거리 확인 (~10m)
   - 사람 감지 없음
   - 배터리 충분
4. **비상 정지**: GPIO 인터럽트

## 테스트 계획

### 1. 정적 테스트
- 서보 각도별 비행 거리 측정
- 최적 각도 결정 (예: 30°, 45°, 60° 테스트)

### 2. 동적 테스트
- 10m 거리에서 정확도 테스트
- 연속 발사 테스트 (장전 시간)

### 3. 통합 테스트
- 열화상 감지 → LiDAR 거리 → 발사 전체 흐름

## 다음 단계

1. ⏳ 서보 모터 선정 및 주문
2. ⏳ GPIO 핀 매핑 확정
3. ⏳ 수동 각도 테스트 (물리적)
4. ⏳ `servo_controller.cpp` 구현
5. ⏳ `fire_trigger.cpp` 구현
6. ⏳ 통합 테스트

---

**작성일**: 2025-12-31  
**상태**: 설계 단계  
**우선순위**: Phase 1 (LiDAR 다음)
