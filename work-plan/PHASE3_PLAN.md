# Phase 3 계획: ROS2 통신 강화

## 목표
모든 계층 간 통신을 ROS2 토픽으로 전환하여 모듈화와 확장성을 향상시킵니다.

## 현재 상태
- Phase 1 완료: 타겟팅 계층 분리
- Phase 2 완료: 스트리밍 계층 분리
- 현재: C++ 클래스 간 직접 호출 및 큐 사용

## 계획된 구조

### ROS2 토픽 구조

```
thermal/
  └── thermal_node (ROS2 노드)
      ├── 발행: /thermal/data (ThermalData 메시지)
      └── 발행: /thermal/image (Image 메시지)

lidar/
  └── lidar_node (ROS2 노드)
      ├── 발행: /lidar/points (PointCloud2 또는 커스텀 메시지)
      └── 발행: /lidar/front_distance (Float32)

targeting/
  └── targeting_node (ROS2 노드)
      ├── 구독: /thermal/data
      ├── 구독: /lidar/points
      ├── 구독: /thermal/image (원본 프레임)
      └── 발행: /targeting/image (합성된 프레임)
      └── 발행: /targeting/data (TargetingData 메시지)

streaming/
  └── streaming_node (ROS2 노드)
      ├── 구독: /targeting/image (또는 /thermal/image)
      └── RTSP/HTTP 스트리밍

또는

main (통합 노드)
  ├── thermal_node (내부)
  ├── lidar_node (내부)
  ├── targeting_node (내부)
  └── streaming_node (내부)
```

## 구현 옵션

### 옵션 1: 완전 분리 (각 계층을 독립 ROS2 노드로)
- 장점: 완전한 모듈화, 독립 실행 가능
- 단점: 오버헤드 증가, 복잡도 증가

### 옵션 2: 하이브리드 (현재 구조 유지 + ROS2 토픽 추가)
- 장점: 점진적 전환, 성능 유지
- 단점: 일부 결합 유지

### 옵션 3: 단일 노드 내부 통신 (현재 유지) + ROS2 토픽 발행 (외부 접근용)
- 장점: 성능 최적, 구현 간단
- 단점: 내부 결합 유지

## 추천 접근법

**옵션 3 (점진적 접근)**을 권장합니다:
1. 현재 구조 유지 (성능 보장)
2. ROS2 토픽 발행 추가 (외부 모니터링/디버깅용)
3. 향후 필요시 점진적으로 옵션 2 또는 1로 전환

## 구현 단계

### Step 1: ROS2 메시지 타입 정의
- `ThermalData.msg` - 열화상 데이터
- `LidarPoints.msg` - 라이다 포인트 클라우드
- `TargetingData.msg` - 타겟팅 정보

### Step 2: 각 계층에 ROS2 래퍼 노드 추가 (선택사항)
- C++ 클래스는 그대로 유지
- ROS2 노드는 C++ 클래스의 래퍼 역할

### Step 3: ROS2 토픽 발행 추가
- thermal/에서 `/thermal/data`, `/thermal/image` 발행
- lidar/에서 `/lidar/points`, `/lidar/front_distance` 발행
- targeting/에서 `/targeting/image` 발행
- streaming/에서 ROS2 이미지 토픽 구독 (옵션)

### Step 4: 동기화 (필요시)
- 타임스탬프 기반 동기화
- message_filters 사용

## 고려사항

1. **성능**: ROS2 토픽은 약간의 오버헤드가 있음
   - 해결: 내부 통신은 큐 유지, 외부 접근용으로만 ROS2 사용

2. **실시간성**: 현재 30 FPS 스트리밍
   - 해결: ROS2는 모니터링용으로만 사용, 실제 스트리밍은 기존 방식 유지

3. **점진적 전환**: 기존 코드와 호환성 유지
   - 해결: ROS2 기능은 선택적으로 활성화

## 다음 단계

사용자와 상의하여 구현 옵션 결정 필요:
1. 완전 분리 (옵션 1)
2. 하이브리드 (옵션 2)  
3. 점진적 접근 (옵션 3) ← 추천

