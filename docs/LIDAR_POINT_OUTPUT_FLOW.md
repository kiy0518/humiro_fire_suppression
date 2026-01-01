# 라이다 포인트 출력 흐름 설명

## 전체 개요

라이다 포인트는 다음과 같은 단계를 거쳐 화면에 출력됩니다:

```
시리얼 통신 → 패킷 파싱 → 포인트 추출 → 스캔 누적 → 데이터 전달 → 화면 표시
```

---

## 1단계: 시리얼 통신으로 데이터 수신

**위치**: `lidar/src/lidar_interface.cpp` - `receiveThread()`

```cpp
void LidarInterface::receiveThread() {
    uint8_t buffer[1024];
    
    while (is_running_.load()) {
        // 시리얼 포트에서 데이터 읽기
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer));
        
        if (bytes_read > 0) {
            // 패킷 버퍼에 추가
            packet_buffer_.insert(packet_buffer_.end(), buffer, buffer + bytes_read);
            
            // 패킷 파싱
            while (packet_buffer_.size() >= LD19_PACKET_SIZE) {
                // 헤더(0x54) 찾기
                auto it = std::find(packet_buffer_.begin(), packet_buffer_.end(), 
                                    static_cast<uint8_t>(LD19_HEADER));
                
                if (it != packet_buffer_.end()) {
                    // 패킷 파싱 시도
                    if (parsePacket(packet_buffer_.data(), LD19_PACKET_SIZE)) {
                        // 파싱 성공, 패킷 제거
                        packet_buffer_.erase(packet_buffer_.begin(), 
                                            packet_buffer_.begin() + LD19_PACKET_SIZE);
                    }
                }
            }
        }
        usleep(1000);  // 1ms 대기
    }
}
```

**설명**:
- 별도 스레드에서 시리얼 포트(`/dev/ttyS4`)로부터 데이터를 지속적으로 읽습니다
- 읽은 데이터를 `packet_buffer_`에 누적합니다
- 버퍼에 완전한 패킷(47바이트)이 있으면 파싱을 시도합니다

---

## 2단계: 패킷 파싱 및 포인트 추출

**위치**: `lidar/src/lidar_interface.cpp` - `parsePacket()`

### 2-1. 패킷 구조 확인

LD19 프로토콜 패킷 구조 (47바이트):
```
[0]     Header (0x54)
[1]     VerLen (0x2C) - 패킷 타입(1) + 포인트 수(12)
[2-3]   Speed (LSB, MSB) - degrees per second
[4-5]   Start angle (LSB, MSB) - 0.01도 단위
[6-41]  Data (12 points × 3 bytes = 36 bytes)
        - 각 포인트: distance(2바이트, mm) + intensity(1바이트)
[42-43] End angle (LSB, MSB) - 0.01도 단위
[44-45] Timestamp (LSB, MSB) - milliseconds
[46]    CRC check
```

### 2-2. 각도 계산

```cpp
// 시작 각도 및 종료 각도 읽기 (little-endian, 0.01도 단위)
uint16_t start_angle_raw = data[4] | (data[5] << 8);
uint16_t end_angle_raw = data[42] | (data[43] << 8);

// 각도를 도 단위로 변환
float start_angle = start_angle_raw / 100.0f;  // 예: 4396 → 43.96도
float end_angle = end_angle_raw / 100.0f;      // 예: 8238 → 82.38도

// 각도 간격 계산 (LD19 V2.5 Manual 공식)
// step = (end_angle - start_angle) / (len - 1)
float angle_diff = end_angle - start_angle;
if (angle_diff > 180.0f) angle_diff -= 360.0f;  // 360도 경계 처리
else if (angle_diff < -180.0f) angle_diff += 360.0f;

float angle_step = angle_diff / (LD19_POINTS_PER_PACK - 1);  // 12개 포인트
```

### 2-3. 각 포인트 추출

```cpp
for (int i = 0; i < LD19_POINTS_PER_PACK; i++) {  // 12개 포인트
    // Data 섹션 시작 위치: 6번째 바이트부터
    int offset = 6 + i * 3;  // 각 포인트는 3바이트
    
    LidarPoint point;
    
    // 1. 거리 읽기 (little-endian, mm 단위)
    uint16_t distance_mm = data[offset] | (data[offset + 1] << 8);
    point.distance = distance_mm / 1000.0f;  // mm → m 변환
    
    // 2. 각도 계산 (선형 보간)
    point.angle = start_angle + angle_step * i;
    // 예: start=43.96°, step=3.2° → i=0: 43.96°, i=1: 47.16°, ...
    
    // 3. 각도 정규화 (0~360도 범위)
    while (point.angle >= 360.0f) point.angle -= 360.0f;
    while (point.angle < 0.0f) point.angle += 360.0f;
    
    // 4. Intensity (신호 강도)
    point.quality = data[offset + 2];
    
    // 5. 유효 범위 체크 (0.05m ~ 12m)
    if (point.distance >= 0.05f && point.distance <= 12.0f) {
        new_scan.points.push_back(point);
    }
}
```

**예시**:
- 패킷 1: start_angle=43.96°, end_angle=82.38° → 12개 포인트 (각도: 43.96°, 47.16°, ..., 82.38°)
- 패킷 2: start_angle=82.38°, end_angle=120.80° → 12개 포인트 (각도: 82.38°, 85.58°, ..., 120.80°)
- 패킷 3: start_angle=120.80°, end_angle=159.22° → 12개 포인트
- ... (계속 누적)

---

## 3단계: 스캔 데이터 누적

**위치**: `lidar/src/lidar_interface.cpp` - `parsePacket()` (하단부)

### 3-1. 새 스캔 감지

```cpp
// 새 스캔 감지 조건:
// 1. 각도가 0도 근처로 돌아옴 (정상 회전 완료)
bool angle_wrapped = (start_angle < 30.0f && max_existing_angle > 330.0f);

// 2. 타임스탬프가 100ms 이상 증가 (새 스캔 시작)
bool timestamp_jump = (timestamp_diff > 100);

// 3. 각도 범위가 크게 달라짐 (60도 이상 차이)
bool angle_range_changed = (std::abs(angle_diff) > 60.0f);

if (angle_wrapped || timestamp_jump || angle_range_changed) {
    is_new_scan = true;
    current_scan_.points.clear();  // 이전 데이터 초기화
}
```

### 3-2. 포인트 누적 및 중복 제거

```cpp
// 새 포인트 추가 (중복 제거: 같은 각도(±0.5도)의 포인트가 있으면 업데이트)
for (const auto& new_point : new_scan.points) {
    bool found = false;
    for (auto& existing_point : current_scan_.points) {
        float angle_diff = std::abs(existing_point.angle - new_point.angle);
        if (angle_diff > 180.0f) angle_diff = 360.0f - angle_diff;
        
        if (angle_diff < 0.5f) {
            // 같은 각도 범위의 포인트 업데이트
            existing_point = new_point;
            found = true;
            break;
        }
    }
    if (!found) {
        current_scan_.points.push_back(new_point);  // 새 포인트 추가
    }
}

// 각도 순으로 정렬
std::sort(current_scan_.points.begin(), current_scan_.points.end(),
          [](const LidarPoint& a, const LidarPoint& b) {
              return a.angle < b.angle;
          });

// 최대 360개 포인트 유지
if (current_scan_.points.size() > 360) {
    current_scan_.points.erase(current_scan_.points.begin() + 360,
                              current_scan_.points.end());
}
```

**결과**: `current_scan_.points`에 0°~360° 범위의 모든 포인트가 누적됩니다.

---

## 4단계: 메인 스레드에서 데이터 가져오기

**위치**: `application/main.cpp` - `lidar_thread()`

```cpp
void lidar_thread() {
    while (is_running) {
        if (lidar_interface->isConnected()) {
            // 최신 스캔 데이터 가져오기 (mutex로 보호)
            LidarScan scan;
            if (lidar_interface->getLatestScan(scan)) {
                // TargetingFrameCompositor에 라이다 데이터 전달
                targeting_compositor->setLidarData(scan.points);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz
    }
}
```

**설명**:
- 별도 스레드에서 100ms마다 최신 스캔 데이터를 가져옵니다
- `getLatestScan()`은 mutex로 보호되어 스레드 안전합니다
- 가져온 데이터를 `TargetingFrameCompositor`에 전달합니다

---

## 5단계: 오버레이에 데이터 전달

**위치**: `osd/src/lidar/distance_overlay.cpp` - `setLidarData()`

```cpp
void DistanceOverlay::setLidarData(const std::vector<LidarPoint>& lidar_points) {
    // 라이다 포인트 저장 (mutex로 보호)
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        lidar_points_ = lidar_points;
    }
    
    // 전방 거리 계산 및 캐시 업데이트
    if (!lidar_points.empty()) {
        // -2~2도 범위의 포인트들을 평균내어 전방 거리 계산
        std::vector<float> center_distances;
        for (const auto& point : lidar_points) {
            float adjusted_angle = point.angle + angle_offset;
            // ... 각도 범위 체크 ...
            if (angle_diff <= 2.0f) {
                center_distances.push_back(point.distance);
            }
        }
        
        // 평균 거리 계산 및 캐시 업데이트
        if (!center_distances.empty()) {
            float avg_distance = /* 평균 계산 */;
            cached_center_distance_ = avg_distance;
        }
    }
}
```

**설명**:
- 라이다 포인트를 내부 변수에 저장합니다 (mutex로 보호)
- 전방 거리(-2~2도 범위)를 계산하여 캐시에 저장합니다

---

## 6단계: 화면에 표시

**위치**: `osd/src/lidar/distance_overlay.cpp` - `drawOverlay()` → `drawMinimap()`

### 6-1. 오버레이 그리기 시작

```cpp
void DistanceOverlay::drawOverlay(cv::Mat& frame) {
    // 라이다 데이터 가져오기 (mutex로 보호)
    std::vector<LidarPoint> points;
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        points = lidar_points_;
    }
    
    // 전방 거리 텍스트 표시
    drawCenterDistanceText(frame, center_x);
    
    // 미니맵 표시 (오른쪽 상단)
    if (!points.empty()) {
        drawMinimap(frame, points, angle_offset);
    }
}
```

### 6-2. 미니맵에 포인트 그리기

```cpp
void DistanceOverlay::drawMinimap(cv::Mat& frame, const std::vector<LidarPoint>& points, float angle_offset) {
    const int MINIMAP_SIZE = 100;  // 100x100 픽셀
    const int MINIMAP_RADIUS = 50;  // 반경 50픽셀
    const float MAX_RANGE = 12.0f;  // 12m
    
    // 미니맵 중심 위치 (오른쪽 상단)
    int minimap_center_x = frame.cols - 5 - 50;
    int minimap_center_y = 5 + 50;
    
    // 배경 그리기 (원형, 검정색, 투명도 0.8)
    // ... 원형 영역 내부를 검정색으로 채움 ...
    
    // 거리 링 그리기 (12m 원)
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), 
               static_cast<int>((12.0f / MAX_RANGE) * MINIMAP_RADIUS),
               cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
    
    // 센터에 점 그리기 (흰색)
    cv::circle(frame, cv::Point(minimap_center_x, minimap_center_y), 2,
               cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    
    // 라이다 포인트 그리기
    for (const auto& point : points) {
        // 1. 오프셋 적용하여 실제 각도 계산
        float adjusted_angle = point.angle + angle_offset;
        while (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;
        while (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
        
        // 2. 유효 범위 체크
        if (point.distance < 0.05f || point.distance > 12.0f) {
            continue;
        }
        
        // 3. 거리를 픽셀 반경으로 변환 (12m = 50픽셀)
        float normalized_dist = std::min(point.distance / MAX_RANGE, 1.0f);
        float pixel_radius = normalized_dist * MINIMAP_RADIUS;
        
        // 4. 각도를 라디안으로 변환 (0° = 위쪽)
        float angle_rad = (adjusted_angle - 90.0f) * PI / 180.0f;
        
        // 5. 미니맵 좌표 계산
        int x = minimap_center_x + static_cast<int>(std::round(pixel_radius * std::cos(angle_rad)));
        int y = minimap_center_y + static_cast<int>(std::round(pixel_radius * std::sin(angle_rad)));
        
        // 6. 미니맵 영역 내에 있는지 체크
        int dx = x - minimap_center_x;
        int dy = y - minimap_center_y;
        if (dx * dx + dy * dy <= MINIMAP_RADIUS * MINIMAP_RADIUS) {
            // 7. 거리에 따른 색상 (그라데이션)
            cv::Scalar color = getLidarColor(point.distance);
            
            // 8. 포인트 그리기 (1픽셀)
            frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<uchar>(color[0]),
                static_cast<uchar>(color[1]),
                static_cast<uchar>(color[2])
            );
        }
    }
}
```

**좌표 변환 예시**:
- 각도 0° (전방) → 화면 위쪽
- 각도 90° (오른쪽) → 화면 오른쪽
- 각도 180° (후방) → 화면 아래쪽
- 각도 270° (왼쪽) → 화면 왼쪽

**거리 변환 예시**:
- 거리 0m → 중심 (0픽셀)
- 거리 6m → 반경의 50% (25픽셀)
- 거리 12m → 반경의 100% (50픽셀)

---

## 전체 데이터 흐름 요약

```
┌─────────────────────────────────────────────────────────────┐
│ 1. 시리얼 통신 (receiveThread)                             │
│    /dev/ttyS4 → 패킷 버퍼 (47바이트 패킷)                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. 패킷 파싱 (parsePacket)                                   │
│    - Header 검증 (0x54)                                      │
│    - Start/End angle 읽기                                    │
│    - 12개 포인트 추출 (distance + angle 계산)                │
│    - 각도: start_angle + step * i                           │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. 스캔 누적 (current_scan_)                                 │
│    - 새 스캔 감지 (각도 wrap, timestamp jump 등)             │
│    - 포인트 누적 및 중복 제거 (±0.5도)                       │
│    - 각도 순 정렬                                            │
│    - 최대 360개 포인트 유지                                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. 데이터 전달 (lidar_thread)                                │
│    - getLatestScan() → scan.points                          │
│    - setLidarData(scan.points)                               │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. 오버레이 저장 (DistanceOverlay::setLidarData)             │
│    - lidar_points_에 저장 (mutex 보호)                       │
│    - 전방 거리 계산 및 캐시                                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. 화면 표시 (composite_thread → drawOverlay)                │
│    - drawCenterDistanceText() - 전방 거리 텍스트            │
│    - drawMinimap() - 미니맵 (100x100, 오른쪽 상단)          │
│      * 각 포인트를 극좌표 → 직교좌표 변환                    │
│      * 거리에 따른 색상 그라데이션                          │
│      * 1픽셀 점으로 표시                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 주요 특징

1. **비동기 처리**: 시리얼 수신, 데이터 처리, 화면 표시가 각각 별도 스레드에서 실행됩니다
2. **스레드 안전성**: 모든 공유 데이터 접근은 mutex로 보호됩니다
3. **포인트 누적**: 여러 패킷의 포인트를 누적하여 360도 전체 범위를 표시합니다
4. **중복 제거**: 같은 각도(±0.5도)의 포인트는 최신 값으로 업데이트합니다
5. **실시간 업데이트**: 10Hz로 최신 스캔 데이터를 가져와 화면에 표시합니다

