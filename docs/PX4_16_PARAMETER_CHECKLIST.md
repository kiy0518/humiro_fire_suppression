# PX4 16.0.0 uXRCE-DDS 파라미터 체크리스트

## 필수 파라미터 확인

QGroundControl에서 다음 파라미터들을 확인하세요:

### 1. DDS 도메인 ID
- **파라미터**: `uXRCE-DDS_DOM_ID`
- **값**: `0`
- **설명**: VIM4의 `ROS_DOMAIN_ID=0`과 일치해야 함

### 2. Agent IP 주소
- **파라미터**: `uxrce_dds_ag_ip`
- **값**: `167772171` (정수 형식)
- **IP 주소**: `10.0.0.11` (VIM4 eth0 IP)
- **변환**: `167772171` = `10.0.0.11`
- **상태**: ✅ 확인됨

### 3. Agent 포트
- **파라미터**: `UXRCE_DDS_PRT`
- **값**: `8888`
- **설명**: micro-ROS Agent UDP 포트
- **상태**: ✅ 확인됨

## 파라미터 확인 방법

### QGroundControl에서 확인

1. **Parameters** 메뉴 열기
2. 검색창에 다음 중 하나 입력:
   - `uxrce` 또는 `uXRCE`
   - `dds`
3. 다음 파라미터 확인:
   - `uXRCE-DDS_DOM_ID` = `0`
   - `uxrce_dds_ag_ip` = `167772171` ✅ (10.0.0.11)
   - `UXRCE_DDS_PRT` = `8888` ✅

## IP 주소 변환

### 정수 → IP 주소
```bash
python3 -c "n=167772171; print(f'{n>>24&0xFF}.{n>>16&0xFF}.{n>>8&0xFF}.{n&0xFF}')"
# 출력: 10.0.0.11
```

### IP 주소 → 정수
```bash
python3 -c "ip='10.0.0.11'; parts=ip.split('.'); print(int(parts[0])*256**3 + int(parts[1])*256**2 + int(parts[2])*256 + int(parts[3]))"
# 출력: 167772171
```

## 연결 확인 스크립트

```bash
# 파라미터 확인
./scripts/debug/check_px4_uxrce_params.sh

# 전체 연결 상태 확인
./scripts/debug/check_px4_connection.sh
```

## 문제 해결

### 메시지가 수신되지 않는 경우

1. **PX4 시동 확인**
   - QGC에서 PX4가 실제로 시동이 걸려 있는지 확인
   - 상태는 보이지만 시동이 안 걸렸을 수 있음

2. **파라미터 확인**
   - 위의 3개 파라미터 모두 확인
   - 특히 `uXRCE-DDS_DOM_ID = 0` 확인

3. **QoS 설정 확인**
   - 코드에서 Durability를 `TransientLocal`로 수정 완료
   - 재빌드 필요:
     ```bash
     cd /home/khadas/humiro_fire_suppression/application/build
     cmake ..
     make -j4
     ./scripts/runtime/service-control.sh restart
     ```

4. **micro-ROS Agent 재시작**
   ```bash
   pkill -f micro_ros_agent
   ./scripts/runtime/start_micro_ros_agent_wrapper.sh &
   ```

## 참고

- PX4 16.0.0 이상에서는 `uxrce_dds_ag_ip` 파라미터를 사용
- 이전 버전의 `uXRCE-DDS_CLIENT_IP` 파라미터는 더 이상 사용되지 않음
- IP 주소는 정수 형식으로 저장됨 (네트워크 바이트 순서)

