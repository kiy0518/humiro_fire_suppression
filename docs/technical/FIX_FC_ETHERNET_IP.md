# FC(PX4) 이더넷 IP 설정 수정 방법

## 문제 상황
FC의 eth0가 `192.168.0.3`으로 설정되어 있어, VIM4의 DHCP 서버가 할당하는 `10.0.0.22`를 받지 못하고 있습니다.

## 해결 방법

### 방법 1: DHCP 사용 (권장)
VIM4의 dnsmasq가 자동으로 `10.0.0.22`를 할당하도록 DHCP를 사용합니다.

**QGroundControl에서:**
1. QGroundControl 연결
2. **Analyze Tools** > **MAVLink Console** 열기
3. 다음 명령어 실행:

```bash
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=dhcp >> /fs/microsd/net.cfg
```

4. FC 재부팅

### 방법 2: 고정 IP 설정 (10.0.0.22)
DHCP가 작동하지 않는 경우 고정 IP를 설정합니다.

**QGroundControl에서:**
1. QGroundControl 연결
2. **Analyze Tools** > **MAVLink Console** 열기
3. 다음 명령어 실행:

```bash
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.0.0.22 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo ROUTER=10.0.0.21 >> /fs/microsd/net.cfg
echo DNS=8.8.8.8 >> /fs/microsd/net.cfg
```

4. FC 재부팅

### 방법 3: Fallback 모드 (권장)
DHCP를 먼저 시도하고, 실패 시 고정 IP를 사용합니다.

```bash
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=fallback >> /fs/microsd/net.cfg
echo IPADDR=10.0.0.22 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo ROUTER=10.0.0.21 >> /fs/microsd/net.cfg
echo DNS=8.8.8.8 >> /fs/microsd/net.cfg
```

## 현재 설정 확인

QGroundControl MAVLink Console에서:
```bash
cat /fs/microsd/net.cfg
```

## 설정 적용 후 확인

VIM4에서:
```bash
ping -c 3 10.0.0.22
arp -n | grep 10.0.0.22
```

## 참고
- `BOOTPROTO=dhcp`: DHCP만 사용 (VIM4 dnsmasq가 자동 할당)
- `BOOTPROTO=static`: 고정 IP만 사용
- `BOOTPROTO=fallback`: DHCP 시도 후 실패 시 고정 IP 사용 (가장 안전)
