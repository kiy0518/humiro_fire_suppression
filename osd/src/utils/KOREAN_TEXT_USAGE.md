# 한글 텍스트 사용 가이드

## 개요

OpenCV의 `cv::putText`는 한글을 지원하지 않으므로, `KoreanTextRenderer` 유틸리티를 사용하여 한글 텍스트를 렌더링합니다.

## 사용 방법

### 1. 헤더 파일 포함

```cpp
#include "utils/korean_text_renderer.h"
```

### 2. 기본 사용법

```cpp
// 프레임에 한글 텍스트 그리기
KoreanTextRenderer::putText(
    frame,
    "대기중",                    // 한글 텍스트
    cv::Point(100, 100),         // 위치
    1.0,                         // 폰트 크기
    cv::Scalar(255, 255, 255),  // 색상 (BGR)
    1                            // 두께
);
```

### 3. 텍스트 크기 계산

```cpp
cv::Size text_size = KoreanTextRenderer::getTextSize("대기중", 1.0, 1);
int width = text_size.width;
int height = text_size.height;
```

### 4. 텍스트 이미지로 렌더링

```cpp
cv::Mat text_img = KoreanTextRenderer::renderText(
    "격발 대기",
    1.0,
    cv::Scalar(0, 255, 0),  // 녹색
    1
);

// 프레임에 합성
text_img.copyTo(frame(cv::Rect(x, y, text_img.cols, text_img.rows)));
```

## status_overlay.cpp에서 사용 예제

### 기존 코드 (영어)
```cpp
cv::putText(frame, status_text,
            cv::Point(status_bg_x + status_padding, status_bg_y + status_size.height + status_padding),
            FONT_FACE, FONT_SCALE, 
            status_color, FONT_THICKNESS, cv::LINE_AA);
```

### 한글 사용 코드
```cpp
#include "utils/korean_text_renderer.h"

// ...

// 한글 텍스트로 변경
std::string korean_status_text = getStatusTextKorean(current_status_);  // "대기중", "격발대기" 등

// 텍스트 크기 계산 (한글용)
cv::Size status_size = KoreanTextRenderer::getTextSize(korean_status_text, FONT_SCALE, FONT_THICKNESS);

// 한글 텍스트 그리기
KoreanTextRenderer::putText(
    frame,
    korean_status_text,
    cv::Point(status_bg_x + status_padding, status_bg_y + status_size.height + status_padding),
    FONT_SCALE,
    status_color,
    FONT_THICKNESS
);
```

## 폰트 설정

### 시스템 기본 폰트 사용 (권장)
- 자동으로 시스템의 한글 폰트를 찾습니다
- Ubuntu/Debian: NanumGothic, NotoSansCJK 등

### 커스텀 폰트 사용
```cpp
KoreanTextRenderer::putText(
    frame,
    "대기중",
    cv::Point(100, 100),
    1.0,
    cv::Scalar(255, 255, 255),
    1,
    "/path/to/custom/font.ttf"  // 커스텀 폰트 경로
);
```

## 요구사항

### 필수
- Python3
- PIL/Pillow (`pip3 install Pillow`)
- OpenCV

### 선택사항 (성능 향상)
- FreeType 라이브러리 (`sudo apt-get install libfreetype6-dev`)
  - FreeType이 있으면 더 빠르고 효율적으로 렌더링됩니다
  - 없어도 PIL을 사용하여 동작합니다

## 설치

```bash
# PIL 설치
pip3 install Pillow

# FreeType 설치 (선택사항)
sudo apt-get install libfreetype6-dev

# CMake 재빌드
cd /home/khadas/humiro_fire_suppression/osd/src
mkdir -p build && cd build
cmake ..
make
```

## 주의사항

1. **성능**: PIL을 사용하는 경우 약간의 오버헤드가 있습니다. FreeType을 사용하면 더 빠릅니다.
2. **폰트 경로**: 시스템에 한글 폰트가 설치되어 있어야 합니다.
3. **UTF-8 인코딩**: 텍스트는 UTF-8 인코딩이어야 합니다.

## 예제: getStatusText 함수를 한글로 변경

```cpp
std::string StatusOverlay::getStatusText(DroneStatus status) {
    switch (status) {
        case DroneStatus::IDLE: return "대기중";
        case DroneStatus::ARMING: return "시동중";
        case DroneStatus::TAKEOFF: return "이륙중";
        case DroneStatus::NAVIGATING: return "이동중";
        case DroneStatus::DESTINATION_REACHED: return "목적지 도착";
        case DroneStatus::FIRE_READY: return "격발 대기";
        case DroneStatus::FIRING_AUTO_TARGETING: return "자동 조준 중";
        case DroneStatus::AUTO_FIRING: return "자동 격발";
        case DroneStatus::MISSION_COMPLETE: return "임무 완료";
        case DroneStatus::RETURNING: return "복귀중";
        case DroneStatus::LANDING: return "착륙중";
        case DroneStatus::DISARMED: return "시동 꺼짐";
        default: return "알 수 없음";
    }
}
```

