# Lepton 3.5 Gain Mode 정보

## PureThermal 기본 설정

PureThermal 보드는 **기본적으로 High Gain Mode**로 설정되어 있습니다.

### Gain Mode별 온도 범위

| Gain Mode | 온도 범위 | 정밀도 | 용도 |
|-----------|----------|--------|------|
| **High Gain** | -10°C ~ 140°C | 높음 | 일반적인 열화상 측정 |
| **Low Gain** | -10°C ~ 400°C (실온 기준)<br>-10°C ~ 450°C (일반적) | 낮음 | 고온 측정 (화재 등) |

## 현재 설정 확인 방법

### 1. 실제 온도 측정값으로 확인
- 알려진 온도 물체를 측정하여 온도 범위로 판단
- 140°C 이상의 온도가 측정되면 Low Gain Mode
- 140°C 이하에서 정밀하게 측정되면 High Gain Mode

### 2. PureThermal 펌웨어 확인
- 현재 펌웨어: v1.3.0
- 기본 설정: High Gain Mode

### 3. 코드 설정
`config.h`에서 `LEPTON_GAIN_MODE` 설정:
```cpp
#define LEPTON_GAIN_MODE 0  // 0 = HIGH_GAIN (기본값)
#define LEPTON_GAIN_MODE 1  // 1 = LOW_GAIN (400°C까지 측정)
```

## Gain Mode 변경 방법

### PureThermal 펌웨어 수정
Low Gain Mode로 변경하려면:
1. PureThermal 펌웨어 소스 코드 확인
2. Lepton 3.5 Gain Mode 설정 부분 수정
3. 수정된 펌웨어를 보드에 업로드

**주의**: 펌웨어 수정은 보드의 공식 문서나 개발자 가이드를 참고해야 합니다.

## 권장 사항

### 화재 진압 드론의 경우
- **Low Gain Mode 권장** (400°C까지 측정 가능)
- 화재 현장의 고온 측정에 적합
- 정밀도는 상대적으로 낮지만, 고온 측정이 우선

### 일반적인 열화상 측정
- **High Gain Mode 권장** (140°C까지)
- 높은 정밀도로 정확한 온도 측정
- 대부분의 일반적인 용도에 적합

## 현재 코드 동작

코드는 `LEPTON_GAIN_MODE` 설정에 따라 온도 변환 범위를 자동으로 조정합니다:

- `LEPTON_GAIN_MODE = 0` (High Gain): -10°C ~ 140°C
- `LEPTON_GAIN_MODE = 1` (Low Gain): -10°C ~ 400°C

**중요**: 코드 설정과 실제 센서 Gain Mode가 일치해야 정확한 온도 측정이 가능합니다.

