# Front Distance (정면 거리) 설명

## Front Distance란?

**Front Distance**는 드론의 **정면(0도)** 방향으로 측정된 거리입니다.

## Front Distance는 별도 데이터가 아닙니다

LD19 LiDAR는 **360° 스캔**을 수행하며, 각 포인트는 다음과 같은 정보를 포함합니다:
- **각도 (angle)**: 0° ~ 360° (0° = 정면)
- **거리 (distance)**: 0.05m ~ 12m
- **품질 (quality)**: 신호 품질

**Front Distance는 별도 데이터가 아니라, 0°에 해당하는 포인트의 거리입니다.**

## 현재 구현 방식

### 방법 1: getFrontDistance() 함수 사용 (권장) ⭐

```cpp
// 0도에 가장 가까운 포인트 찾기
float front_distance = lidar.getFrontDistance(1.0f);  // ±1도 범위
```

**동작 원리:**
1. 모든 스캔 포인트 중에서 각도가 **0°에 가장 가까운** 포인트를 찾습니다
2. 허용 범위(기본 ±1도) 내에서 가장 가까운 포인트의 거리를 반환합니다
3. 데이터가 없으면 -1 반환

**장점:**
- 정확한 정면 거리 측정
- 각도 오차 최소화
- 가장 신뢰할 수 있는 방법

### 방법 2: getRangeData() 사용 (이전 방식)

```cpp
// 정면 60도 범위 데이터 가져오기
std::vector<LidarPoint> lidar_data = lidar.getRangeData(330.0f, 30.0f);

// 첫 번째 요소 사용 (부정확할 수 있음)
float front_distance = lidar_data[0].distance;
```

**문제점:**
- `lidar_data[0]`은 배열의 첫 번째 요소일 뿐
- 0°에 해당하는 데이터가 아닐 수 있음
- 정확도 낮음

## LD19 각도 시스템

### 각도 정의
- **0°**: 정면 (드론 전방)
- **90°**: 우측
- **180°**: 후방
- **270°**: 좌측
- **360°**: 정면 (0°와 동일)

### 스캔 데이터 구조
```
포인트 1: angle=0.0°,   distance=10.5m
포인트 2: angle=1.0°,   distance=10.3m
포인트 3: angle=2.0°,   distance=10.1m
...
포인트 360: angle=359.0°, distance=9.8m
```

**Front Distance = angle=0.0°인 포인트의 distance**

## 코드 예제

### 정면 거리 가져오기

```cpp
#include "lidar_interface.h"

LidarInterface lidar(config);

if (lidar.isConnected()) {
    // 방법 1: getFrontDistance() 사용 (권장)
    float front_dist = lidar.getFrontDistance(1.0f);  // ±1도 범위
    if (front_dist > 0) {
        std::cout << "정면 거리: " << front_dist << "m" << std::endl;
    }
    
    // 방법 2: 0도 근처 범위의 평균
    std::vector<LidarPoint> front_data = lidar.getRangeData(359.0f, 1.0f);
    if (!front_data.empty()) {
        float sum = 0.0f;
        for (const auto& p : front_data) {
            sum += p.distance;
        }
        float avg_front = sum / front_data.size();
        std::cout << "정면 평균 거리: " << avg_front << "m" << std::endl;
    }
}
```

## getFrontDistance() 함수 상세

### 함수 시그니처
```cpp
float getFrontDistance(float tolerance = 1.0f);
```

### 파라미터
- **tolerance**: 허용 각도 범위 (도, 기본: 1.0도)
  - 예: `1.0f` → 0° ± 1° 범위 내의 포인트 검색
  - 예: `0.5f` → 0° ± 0.5° 범위 (더 정확하지만 데이터가 없을 수 있음)

### 반환값
- **거리 (미터)**: 0도에 가장 가까운 포인트의 거리
- **-1.0f**: 데이터 없음 (범위 내 포인트 없음)

### 동작 알고리즘
```cpp
1. 모든 스캔 포인트 순회
2. 각 포인트의 각도와 0도 사이의 차이 계산
3. 차이가 tolerance 이내이고 가장 작은 포인트 선택
4. 해당 포인트의 거리 반환
```

## 사용 예제

### 예제 1: 기본 사용
```cpp
float front = lidar.getFrontDistance();  // 기본 ±1도
```

### 예제 2: 더 정확한 측정
```cpp
float front = lidar.getFrontDistance(0.5f);  // ±0.5도 (더 정확)
```

### 예제 3: 넓은 범위
```cpp
float front = lidar.getFrontDistance(3.0f);  // ±3도 (더 많은 데이터)
```

## 주의사항

1. **각도 해상도**: LD19는 1° 해상도이므로, 정확히 0°인 포인트가 없을 수 있습니다
2. **Tolerance 조정**: 너무 작으면 데이터가 없을 수 있고, 너무 크면 부정확할 수 있습니다
3. **데이터 유효성**: 반환값이 -1이면 데이터가 없는 것이므로 체크해야 합니다

## 관련 코드 위치

- 함수 정의: `lidar_interface.h` (선언)
- 함수 구현: `lidar_interface.cpp` (구현)
- 사용 예제: `main_test.cpp` (headless 모드)

