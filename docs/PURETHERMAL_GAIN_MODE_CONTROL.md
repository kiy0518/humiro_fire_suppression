# PureThermal Gain Mode 제어 방법

## 현재 상황

### 확인된 정보
- **디바이스**: PureThermal (fw:v1.3.0)
- **USB ID**: 1e4e:0104
- **인터페이스**: UVC (USB Video Class)
- **V4L2 속성**: Brightness, Contrast만 확인됨

### Gain Mode 제어 가능 여부
**현재 Linux 환경에서는 USB를 통한 Gain Mode 직접 제어가 어려울 수 있습니다.**

## 가능한 방법

### 1. UVC Extension Unit 사용 (고급)
PureThermal이 UVC Extension Unit을 지원하는 경우, 직접 USB 통신을 통해 Gain Mode를 제어할 수 있습니다.

**필요한 작업:**
- UVC Extension Unit ID 확인
- USB 제어 요청 (Control Request) 구현
- Lepton SDK의 Gain Mode 제어 명령 사용

**구현 복잡도**: 높음 (USB 통신 프로토콜 이해 필요)

### 2. 펌웨어 수정
PureThermal 보드의 펌웨어를 수정하여 기본 Gain Mode를 변경할 수 있습니다.

**필요한 작업:**
- PureThermal 펌웨어 소스 코드 확인
- Lepton Gain Mode 설정 부분 수정
- 수정된 펌웨어 업로드

**구현 복잡도**: 중간 (펌웨어 개발 경험 필요)

### 3. Windows 소프트웨어 사용
FLIR에서 제공하는 Windows용 소프트웨어를 사용하여 Gain Mode를 설정한 후, Linux에서 사용할 수 있습니다.

**제한사항:**
- Windows PC 필요
- 설정 후 재부팅 시 유지 여부 확인 필요

### 4. 현재 코드에서 온도 범위 조정 (권장)
Gain Mode를 변경할 수 없더라도, 코드에서 온도 변환 범위를 조정하여 사용할 수 있습니다.

**현재 구현:**
- `LEPTON_GAIN_MODE` 설정으로 온도 변환 범위 조정
- High Gain: -10°C ~ 140°C
- Low Gain: -10°C ~ 400°C

**주의사항:**
- 센서의 실제 Gain Mode와 코드 설정이 일치해야 정확한 온도 측정 가능
- 센서가 High Gain Mode인데 코드를 Low Gain으로 설정하면 온도 값이 부정확함

## 권장 사항

### 화재 진압 드론의 경우
1. **펌웨어 수정** (장기적 해결책)
   - PureThermal 펌웨어를 Low Gain Mode로 수정
   - 400°C까지 측정 가능하도록 설정

2. **코드 설정 조정** (임시 해결책)
   - 현재 센서가 High Gain Mode라면, 코드도 High Gain으로 설정
   - 센서를 Low Gain으로 변경한 후, 코드를 Low Gain으로 설정

### 확인 방법
1. **실제 온도 측정으로 확인**
   - 알려진 온도 물체를 측정
   - 140°C 이상이 측정되면 Low Gain Mode
   - 140°C 이하에서 정밀하게 측정되면 High Gain Mode

2. **PureThermal 문서 확인**
   - 펌웨어 v1.3.0의 기본 Gain Mode 확인
   - UVC Extension Unit 지원 여부 확인

## 구현 예시 (UVC Extension Unit 사용 시)

만약 UVC Extension Unit을 통해 제어 가능하다면:

```cpp
// USB 제어 요청을 통한 Gain Mode 설정
// (실제 구현은 PureThermal의 UVC Extension Unit 사양 필요)
int setLeptonGainMode(int camera_fd, bool low_gain) {
    // UVC Extension Unit 제어 요청
    // 실제 구현은 PureThermal 문서 참조 필요
    return 0;
}
```

## 결론

**현재 상태:**
- USB를 통한 Gain Mode 직접 제어는 어려울 수 있음
- 펌웨어 수정 또는 Windows 소프트웨어 사용 권장
- 코드에서 온도 변환 범위는 이미 구현됨

**다음 단계:**
1. 실제 센서의 Gain Mode 확인 (온도 측정으로)
2. 필요시 펌웨어 수정 또는 Windows 소프트웨어 사용
3. 코드의 `LEPTON_GAIN_MODE` 설정을 실제 센서와 일치시키기

