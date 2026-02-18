#!/usr/bin/env python3
"""
PureThermal Lepton 3.5 — Radiometry + Low Gain 설정 스크립트

GroupGets libuvc (https://github.com/groupgets/libuvc) 필요.
GetThermal (https://github.com/groupgets/GetThermal) 참조.

사용법:
  sudo python3 scripts/setup_lepton.py          # 기본: TLinear ON + Low Gain
  sudo python3 scripts/setup_lepton.py --status  # 현재 설정 확인만
  sudo python3 scripts/setup_lepton.py --high    # High Gain 모드로 전환

UVC Extension Unit 매핑 (GetThermal leptonvariation.cpp):
  AGC=3, OEM=4, RAD=5, SYS=6, VID=7
  Control ID = ((CCI_reg & 0x00FF) >> 2) + 1
"""

import sys
import os
import time
from ctypes import (
    cdll, POINTER, Structure, c_void_p, c_uint8, c_uint16, c_uint32,
    c_int, c_ulong, c_ubyte, byref, create_string_buffer
)

# ── libuvc 로드 ──────────────────────────────────────────────
try:
    libuvc = cdll.LoadLibrary("libuvc.so")
except OSError:
    print("[ERROR] libuvc.so를 찾을 수 없습니다.")
    print("  GroupGets libuvc를 설치하세요:")
    print("  git clone https://github.com/groupgets/libuvc.git")
    print("  cd libuvc && mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig")
    sys.exit(1)

# ── PureThermal USB ID ───────────────────────────────────────
PT_USB_VID = 0x1e4e
PT_USB_PID = 0x0100

# ── UVC Extension Unit IDs (GetThermal 기준) ─────────────────
RAD_UNIT_ID = 5   # Radiometry 모듈
SYS_UNIT_ID = 6   # System 모듈

# ── Control IDs: ((CCI_reg & 0xFF) >> 2) + 1 ─────────────────
# RAD module (base 0x4E00)
RAD_ENABLE_CTRL            = 5    # 0x4E10 → (0x10>>2)+1 = 5
RAD_TLINEAR_ENABLE_CTRL    = 49   # 0x4EC0 → (0xC0>>2)+1 = 49
RAD_TLINEAR_RESOLUTION_CTRL = 50  # 0x4EC4 → (0xC4>>2)+1 = 50
# SYS module (base 0x0200)
SYS_GAIN_MODE_CTRL         = 19   # 0x0248 → (0x48>>2)+1 = 19

# ── Enum 값 ──────────────────────────────────────────────────
GAIN_HIGH = 0   # -10°C ~ 140°C (정밀)
GAIN_LOW  = 1   # -10°C ~ 400°C (넓은 범위)
GAIN_AUTO = 2

RAD_DISABLE = 0
RAD_ENABLE  = 1

TLINEAR_RES_0_1K  = 0   # 0.1K per count
TLINEAR_RES_0_01K = 1   # 0.01K per count (센티켈빈)

GAIN_NAMES = {0: "HIGH (-10~140°C)", 1: "LOW (-10~400°C)", 2: "AUTO"}
RES_NAMES  = {0: "0.1K", 1: "0.01K"}


# ── ctypes 구조체 (libuvc) ────────────────────────────────────
class uvc_context(Structure):
    _fields_ = [("usb_ctx", c_void_p),
                ("own_usb_ctx", c_uint8),
                ("open_devices", c_void_p),
                ("handler_thread", c_ulong),
                ("kill_handler_thread", c_int)]

class uvc_device(Structure):
    _fields_ = [("ctx", POINTER(uvc_context)),
                ("ref", c_int),
                ("usb_dev", c_void_p)]

class uvc_device_handle(Structure):
    _fields_ = [("dev", POINTER(uvc_device)),
                ("prev", c_void_p),
                ("next", c_void_p),
                ("usb_devh", c_void_p),
                ("info", c_void_p),
                ("status_xfer", c_void_p),
                ("status_buf", c_ubyte * 32),
                ("status_cb", c_void_p),
                ("status_user_ptr", c_void_p),
                ("button_cb", c_void_p),
                ("button_user_ptr", c_void_p),
                ("streams", c_void_p),
                ("is_isight", c_ubyte)]


# ── UVC 제어 함수 ────────────────────────────────────────────
# HumiroThermal fw v1.3.1: RAD 모듈은 4바이트, SYS는 2/4바이트 모두 OK
RAD_DATA_SIZE = 4   # RAD 제어 데이터 크기 (2바이트 → PIPE error, 4바이트 OK)
SYS_DATA_SIZE = 4   # SYS 제어 데이터 크기


def uvc_get(devh, unit_id, ctrl_id, size=None):
    """UVC extension unit에서 값 읽기."""
    if size is None:
        size = RAD_DATA_SIZE if unit_id == RAD_UNIT_ID else SYS_DATA_SIZE
    data = create_string_buffer(size)
    ret = libuvc.uvc_get_ctrl(devh, unit_id, ctrl_id, data, size, 0x81)  # GET_CUR
    if ret < 0:
        raise RuntimeError(f"uvc_get_ctrl 실패: unit={unit_id} ctrl={ctrl_id} ret={ret}")
    if size <= 2:
        return c_uint16.from_buffer_copy(data.raw[:2] + b'\x00' * max(0, 2 - size)).value
    elif size == 4:
        return c_uint32.from_buffer_copy(data).value
    return int.from_bytes(data.raw, 'little')


def uvc_set(devh, unit_id, ctrl_id, value, size=None):
    """UVC extension unit에 값 쓰기."""
    if size is None:
        size = RAD_DATA_SIZE if unit_id == RAD_UNIT_ID else SYS_DATA_SIZE
    if size == 4:
        data = c_uint32(value)
    elif size == 2:
        data = c_uint16(value)
    elif size == 1:
        data = c_uint8(value)
    else:
        raise ValueError(f"지원하지 않는 크기: {size}")
    ret = libuvc.uvc_set_ctrl(devh, unit_id, ctrl_id, byref(data), size)
    if ret < 0:
        raise RuntimeError(f"uvc_set_ctrl 실패: unit={unit_id} ctrl={ctrl_id} ret={ret}")
    return ret


# ── 상태 조회 ─────────────────────────────────────────────────
def print_status(devh):
    """현재 Lepton 설정 출력."""
    print("\n=== Lepton 3.5 현재 설정 ===")
    try:
        gain = uvc_get(devh, SYS_UNIT_ID, SYS_GAIN_MODE_CTRL)
        print(f"  Gain Mode    : {GAIN_NAMES.get(gain, f'UNKNOWN({gain})')}")
    except RuntimeError as e:
        print(f"  Gain Mode    : 읽기 실패 ({e})")

    try:
        rad = uvc_get(devh, RAD_UNIT_ID, RAD_ENABLE_CTRL)
        print(f"  RAD Enable   : {'ON' if rad else 'OFF'}")
    except RuntimeError as e:
        print(f"  RAD Enable   : 읽기 실패 ({e})")

    try:
        tlin = uvc_get(devh, RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL)
        print(f"  TLinear      : {'ON' if tlin else 'OFF'}")
    except RuntimeError as e:
        print(f"  TLinear      : 읽기 실패 ({e})")

    try:
        res = uvc_get(devh, RAD_UNIT_ID, RAD_TLINEAR_RESOLUTION_CTRL)
        print(f"  TLinear Res  : {RES_NAMES.get(res, f'UNKNOWN({res})')}")
    except RuntimeError as e:
        print(f"  TLinear Res  : 읽기 실패 ({e})")
    print()


# ── 메인 ──────────────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description="PureThermal Lepton 3.5 설정")
    parser.add_argument("--status", action="store_true", help="현재 설정 확인만")
    parser.add_argument("--high", action="store_true", help="High Gain 모드 (-10~140°C)")
    parser.add_argument("--auto-gain", action="store_true", help="Auto Gain 모드")
    parser.add_argument("--no-tlinear", action="store_true", help="TLinear 비활성화")
    args = parser.parse_args()

    # root 권한 확인
    if os.geteuid() != 0:
        print("[WARNING] root 권한 없이 실행 중. USB 접근에 실패할 수 있습니다.")
        print("  sudo python3 scripts/setup_lepton.py")

    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()

    res = libuvc.uvc_init(byref(ctx), 0)
    if res < 0:
        print(f"[ERROR] uvc_init 실패: {res}")
        sys.exit(1)

    try:
        res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            print(f"[ERROR] PureThermal 장치를 찾을 수 없음 (VID={PT_USB_VID:#06x} PID={PT_USB_PID:#06x})")
            print("  USB 연결 확인: lsusb | grep 1e4e")
            sys.exit(1)

        try:
            res = libuvc.uvc_open(dev, byref(devh))
            if res < 0:
                print(f"[ERROR] uvc_open 실패: {res}")
                print("  다른 프로세스가 카메라를 사용 중이면, 먼저 종료하세요.")
                print("  권한 문제라면 sudo로 실행하세요.")
                sys.exit(1)

            print("[OK] PureThermal 장치 열림")

            # 현재 상태 표시
            print_status(devh)

            if args.status:
                return

            # ── 설정 적용 ──
            if not args.no_tlinear:
                # 1. RAD Enable
                print("[SET] Radiometry 활성화...")
                uvc_set(devh, RAD_UNIT_ID, RAD_ENABLE_CTRL, RAD_ENABLE)
                time.sleep(0.1)

                # 2. TLinear Enable
                print("[SET] TLinear 활성화...")
                uvc_set(devh, RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL, RAD_ENABLE)
                time.sleep(0.1)

                # 3. TLinear Resolution → 0.01K (센티켈빈)
                print("[SET] TLinear 해상도 → 0.01K (센티켈빈)...")
                uvc_set(devh, RAD_UNIT_ID, RAD_TLINEAR_RESOLUTION_CTRL, TLINEAR_RES_0_01K)
                time.sleep(0.1)
            else:
                print("[SET] TLinear 비활성화...")
                uvc_set(devh, RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL, RAD_DISABLE)
                time.sleep(0.1)

            # 4. Gain Mode
            if args.high:
                gain = GAIN_HIGH
                print("[SET] Gain Mode → HIGH (-10~140°C)...")
            elif args.auto_gain:
                gain = GAIN_AUTO
                print("[SET] Gain Mode → AUTO...")
            else:
                gain = GAIN_LOW
                print("[SET] Gain Mode → LOW (-10~400°C)...")
            uvc_set(devh, SYS_UNIT_ID, SYS_GAIN_MODE_CTRL, gain)
            time.sleep(0.5)  # Gain 전환 시 FFC (셔터) 동작 대기

            # 검증
            print_status(devh)
            print("[DONE] Lepton 설정 완료.")
            print("  Y16 데이터: 각 픽셀 = 센티켈빈 (0.01K)")
            print("  온도 변환: T(°C) = pixel * 0.01 - 273.15")

        finally:
            libuvc.uvc_unref_device(dev)
    finally:
        libuvc.uvc_exit(ctx)


if __name__ == '__main__':
    main()
