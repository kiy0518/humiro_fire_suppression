#!/usr/bin/env python3
"""
==========================================================================
RGB + 열화상 데이터 RTSP 스트리밍 서버 (저지연 최적화)
==========================================================================
- RGB 영상만 전송 (열화상 영상 오버레이 없음)
- 열화상 데이터(온도값, 좌표 등)를 텍스트로 표시
- 타겟팅 마커만 표시
- 지연 최소화 및 처리 속도 향상

Lepton 3.5 열화상 카메라 (8~9 FPS)에 맞춰 최적화

QGC 설정:
  - Video Source: "RTSP 비디오 스트림"
  - RTSP URL: rtsp://<VIM4_IP>:8554/stream

필요 패키지:
  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
                   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                   gstreamer1.0-plugins-ugly libgstrtspserver-1.0-0 \
                   gir1.2-gst-rtsp-server-1.0 python3-opencv
==========================================================================
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstRtspServer, GLib, GstApp

import cv2
import numpy as np
import sys
import signal
import socket
import subprocess
import time
import os
import threading
from queue import Queue

# GStreamer 초기화
Gst.init(None)

# ============================================================
# 설정
# ============================================================
RTSP_PORT = "8554"
RTSP_MOUNT_POINT = "/stream"

# 카메라 설정 (이름 기반 자동 감지)
# 실제 카메라 이름: PureThermal (열화상), USB Camera (RGB)
THERMAL_CAMERA_KEYWORDS = ['purethermal', 'thermal', 'lepton', 'flir']  # 열화상 카메라 키워드
THERMAL_CAMERA_ID = None  # 동적으로 설정됨

# 출력 해상도 - RGB 우선 (30 FPS)
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
OUTPUT_FPS = 30  # RGB 카메라 FPS (열화상은 데이터만 사용)
RGB_TARGET_FPS = 30  # RGB 목표 FPS

# 열화상 데이터 처리 설정 (오버레이 안 함, 타겟팅만)
SCALE = 0.75
THERMAL_WIDTH = int(640 * SCALE)  # 480
THERMAL_HEIGHT = int(480 * SCALE)  # 360
THERMAL_DX = 0
THERMAL_DY = -10
CUT_PIXELS = 30
# ALPHA_THERMAL_LAYER 제거 (오버레이 안 함)

# 마커 설정 (미리 계산)
THERMAL_CROPPED_HEIGHT = THERMAL_HEIGHT - CUT_PIXELS  # 330
CENTER = (THERMAL_WIDTH // 2, THERMAL_CROPPED_HEIGHT // 2)  # (240, 165)
RADIUS_OUTER = 100
RADIUS_INNER = 50

# 인코더 설정
BITRATE_KBPS = 1500  # 낮은 FPS이므로 비트레이트 감소
USE_HARDWARE_ENCODER = False

# 전역 변수
server = None
loop = None
is_running = True
cap_thermal = None
cap_rgb = None
frame_queue = Queue(maxsize=2)
rgb_frame_queue = Queue(maxsize=2)  # RGB 프레임 큐

# 열화상 데이터 저장 (이전 값 유지용)
thermal_data = {
    'hotspot_x': None,  # RGB 좌표계에서의 화점 X
    'hotspot_y': None,  # RGB 좌표계에서의 화점 Y
    'center_x': None,   # RGB 좌표계에서의 중심 X
    'center_y': None,   # RGB 좌표계에서의 중심 Y
    'max_val': None,    # 최대값
    'min_val': None,    # 최소값
    'distance': None,   # 거리
    'color': (255, 255, 255),  # 마커 색상
    'max_color': (0, 0, 255),   # 화점 마커 색상
    'rel_x': 0,         # 상대 좌표 X
    'rel_y': 0,         # 상대 좌표 Y
    'timestamp': 0,     # 마지막 업데이트 시간
    'valid': False      # 유효한 데이터인지
}
thermal_data_lock = threading.Lock()  # 열화상 데이터 동기화용

# 미리 계산된 값들 (최적화)
RGB_CROP_X = int(((640 - THERMAL_WIDTH) / 2) + THERMAL_DX)  # 80
RGB_CROP_Y = int(min((480 - THERMAL_HEIGHT) / 2 + THERMAL_DY, 
                     480 - THERMAL_HEIGHT + CUT_PIXELS))  # 약 60

# ============================================================
# 유틸리티
# ============================================================
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

def check_encoder(encoder_name):
    factory = Gst.ElementFactory.find(encoder_name)
    return factory is not None

def get_camera_info(device_path):
    """카메라 정보 가져오기 (이름, VID/PID)"""
    info = {
        'name': None,
        'vid': None,
        'pid': None,
        'model': None,
    }
    
    # /sys/class/video4linux에서 이름 가져오기
    dev_name = os.path.basename(device_path)
    sys_path = f"/sys/class/video4linux/{dev_name}/name"
    if os.path.exists(sys_path):
        try:
            with open(sys_path, 'r') as f:
                info['name'] = f.read().strip()
        except:
            pass
    
    # udevadm으로 VID/PID 가져오기
    try:
        result = subprocess.run(['udevadm', 'info', '--name', device_path],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'ID_VENDOR_ID=' in line:
                    info['vid'] = line.split('=')[1].strip()
                elif 'ID_MODEL_ID=' in line:
                    info['pid'] = line.split('=')[1].strip()
                elif 'ID_MODEL=' in line:
                    info['model'] = line.split('=')[1].strip()
    except:
        pass
    
    return info

def find_camera_by_name(keywords, exclude_keywords=None):
    """키워드로 카메라 찾기 (사용 안 함 - init_cameras에서 직접 처리)"""
    # 이 함수는 더 이상 사용하지 않지만 호환성을 위해 유지
    if exclude_keywords is None:
        exclude_keywords = []
    
    available_devices = []
    for dev_num in range(10):  # video0~video9까지 확인
        device = f"/dev/video{dev_num}"
        if not os.path.exists(device):
            continue
        
        info = get_camera_info(device)
        camera_name = (info['name'] or '').lower()
        
        # 제외 키워드 확인
        is_excluded = any(kw.lower() in camera_name for kw in exclude_keywords)
        if is_excluded:
            continue
        
        # 키워드 매칭 확인
        is_match = any(kw.lower() in camera_name for kw in keywords)
        
        if is_match or (not keywords and camera_name):  # keywords가 비어있으면 모든 카메라
            available_devices.append({
                'id': dev_num,
                'device': device,
                'name': info['name'],
                'info': info
            })
    
    return available_devices

def kill_existing_processes():
    """기존 프로세스 및 카메라 사용 프로세스 종료"""
    my_pid = os.getpid()
    print(f"  → 기존 프로세스 정리... (현재 PID: {my_pid})")
    
    # 카메라를 사용 중인 프로세스 확인 및 종료 (먼저 처리)
    print("  → 카메라 사용 프로세스 확인...")
    for dev_num in range(5):
        device = f"/dev/video{dev_num}"
        if os.path.exists(device):
            try:
                # lsof로 카메라 사용 프로세스 확인
                result = subprocess.run(['lsof', device], 
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0 and result.stdout:
                    lines = result.stdout.strip().split('\n')
                    if len(lines) > 1:  # 헤더 제외
                        for line in lines[1:]:
                            parts = line.split()
                            if len(parts) >= 2:
                                pid = parts[1]
                                try:
                                    pid_int = int(pid)
                                    # 자기 자신이 아니면 종료
                                    if pid_int != my_pid:
                                        print(f"    → /dev/video{dev_num} 사용 중인 프로세스 종료: PID {pid}")
                                        subprocess.run(['kill', '-9', pid], 
                                                     capture_output=True, timeout=1)
                                except:
                                    pass
            except:
                pass
    
    # 일반 프로세스 종료 (더 구체적인 패턴 사용)
    processes_to_kill = [
        'gst-launch-1.0',
        'gst-launch',
    ]
    
    for proc in processes_to_kill:
        try:
            # ps로 프로세스 찾기 (자기 자신 제외)
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if proc in line and str(my_pid) not in line:
                        parts = line.split()
                        if len(parts) >= 2:
                            pid = parts[1]
                            try:
                                pid_int = int(pid)
                                if pid_int != my_pid:
                                    print(f"    → 프로세스 종료: {proc} (PID {pid})")
                                    subprocess.run(['kill', '-9', pid], 
                                                 capture_output=True, timeout=1)
                            except:
                                pass
        except:
            pass
    
    time.sleep(1.0)  # 프로세스 종료 대기

def init_cameras():
    """카메라 초기화 (이름 기반 자동 감지)"""
    global cap_thermal, cap_rgb, RGB_CAMERA_ID
    
    print("  → 카메라 정보 수집...")
    
    # 모든 카메라 정보 수집
    all_cameras = []
    for dev_num in range(10):
        device = f"/dev/video{dev_num}"
        if os.path.exists(device):
            info = get_camera_info(device)
            camera_name = info['name'] or 'Unknown'
            all_cameras.append({
                'id': dev_num,
                'device': device,
                'name': camera_name,
                'info': info
            })
            print(f"    → {device}: {camera_name}")
    
    # 열화상 카메라 찾기 (이름 기반: PureThermal)
    print("\n  → 열화상 카메라 찾기 (PureThermal)...")
    thermal_cameras = []
    for cam in all_cameras:
        camera_name_lower = cam['name'].lower()
        # PureThermal 또는 열화상 키워드 확인
        if any(kw in camera_name_lower for kw in THERMAL_CAMERA_KEYWORDS):
            thermal_cameras.append(cam)
            print(f"    → {cam['device']}: {cam['name']} (열화상으로 인식)")
    
    if not thermal_cameras:
        print("    ⚠ 이름으로 열화상 카메라를 찾을 수 없습니다")
        print("    → 해상도 기반으로 시도...")
        # 이름으로 찾지 못하면 해상도로 찾기 (작은 해상도 = 열화상)
        for cam in all_cameras:
            try:
                test_cap = cv2.VideoCapture(cam['id'], cv2.CAP_V4L2)
                if test_cap.isOpened():
                    ret, frame = test_cap.read()
                    if ret and frame is not None:
                        h, w = frame.shape[:2]
                        if h <= 200 and w <= 200:  # 작은 해상도 = 열화상 가능성
                            thermal_cameras.append(cam)
                            print(f"    → {cam['device']} ({cam['name']}, 해상도: {w}x{h}) - 열화상 가능")
                    test_cap.release()
            except:
                pass
    
    if not thermal_cameras:
        print("    ✗ 열화상 카메라를 찾을 수 없습니다")
        print("    → 예상: /dev/video0 또는 /dev/video1에 PureThermal이 있어야 합니다")
        return False
    
    thermal_cam = thermal_cameras[0]  # 첫 번째 열화상 카메라 사용
    THERMAL_CAMERA_ID = thermal_cam['id']  # 전역 변수에 저장
    print(f"    ✓ 열화상 카메라: {thermal_cam['device']} ({thermal_cam['name']})")
    
    # 열화상 카메라 초기화
    cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
    if not cap_thermal.isOpened():
        print(f"    ✗ 열화상 카메라 열기 실패")
        return False
    
    # 열화상 카메라 타임아웃 설정 (느린 응답 대비)
    try:
        # V4L2 타임아웃 설정 (밀리초 단위, 2초)
        cap_thermal.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # 버퍼 크기 최소화
        cap_thermal.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except:
        pass
    
    # RGB 카메라 찾기 (열화상이 아닌 카메라: USB Camera)
    print("\n  → RGB 카메라 찾기 (USB Camera)...")
    rgb_camera_found = False
    rgb_camera_id = None
    
    # 열화상 카메라 ID 목록
    thermal_ids = [cam['id'] for cam in thermal_cameras]
    
    # RGB 카메라 찾기 (열화상이 아닌 것)
    for cam in all_cameras:
        if cam['id'] in thermal_ids:  # 열화상 카메라는 제외
            continue
        
        # USB Camera 또는 열화상 키워드가 없는 카메라
        camera_name_lower = cam['name'].lower()
        is_thermal = any(kw in camera_name_lower for kw in THERMAL_CAMERA_KEYWORDS)
        if is_thermal:
            continue
        
        device = cam['device']
        camera_name = cam['name']
        rgb_id = cam['id']
        
        print(f"    → {device} 시도 중... ({camera_name} - RGB 후보)")
        
        # 카메라가 사용 중인지 확인
        try:
            result = subprocess.run(['lsof', device], 
                                  capture_output=True, text=True, timeout=1)
            if result.returncode == 0 and result.stdout:
                lines = result.stdout.strip().split('\n')
                if len(lines) > 1:  # 다른 프로세스가 사용 중
                    print(f"      ⚠ {device} 사용 중 - 프로세스 종료 시도...")
                    # 사용 중인 프로세스 종료
                    for line in lines[1:]:
                        parts = line.split()
                        if len(parts) >= 2:
                            pid = parts[1]
                            try:
                                pid_int = int(pid)
                                if pid_int != os.getpid():
                                    subprocess.run(['kill', '-9', pid], 
                                                 capture_output=True, timeout=1)
                                    print(f"        → PID {pid} 종료")
                            except:
                                pass
                    time.sleep(0.5)  # 프로세스 종료 대기
        except:
            pass
        
        try:
            cap_rgb = cv2.VideoCapture(rgb_id, cv2.CAP_V4L2)
            if not cap_rgb.isOpened():
                print(f"      ⚠ {device} 열기 실패")
                continue
            
            cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 테스트 프레임 읽기 (여러 번 시도)
            test_frame = None
            for attempt in range(3):
                ret, test_frame = cap_rgb.read()
                if ret and test_frame is not None and test_frame.size > 0:
                    break
                time.sleep(0.1)
            
            if test_frame is not None and test_frame.size > 0:
                # RGB 카메라인지 확인 (해상도가 충분히 큰지)
                h, w = test_frame.shape[:2]
                print(f"      → 해상도: {w}x{h}")
                
                if h >= 200 and w >= 200:  # RGB 카메라는 보통 큰 해상도
                    print(f"    ✓ RGB 카메라 발견: {device} ({camera_name}, 해상도: {w}x{h})")
                    rgb_camera_found = True
                    rgb_camera_id = rgb_id
                    break
                else:
                    print(f"      ⚠ 해상도가 작음 (열화상일 수 있음) - 스킵")
            else:
                print(f"      ⚠ 프레임 읽기 실패")
            
            if cap_rgb.isOpened():
                cap_rgb.release()
        except Exception as e:
            print(f"      ⚠ 오류: {e}")
            if 'cap_rgb' in locals() and cap_rgb.isOpened():
                cap_rgb.release()
    
    if not rgb_camera_found:
        print(f"    ✗ RGB 카메라를 찾을 수 없습니다")
        print(f"\n    디버깅 정보:")
        thermal_list = [f"{c['device']} ({c['name']})" for c in thermal_cameras]
        rgb_candidates = [f"{c['device']} ({c['name']})" for c in all_cameras if c['id'] not in thermal_ids]
        print(f"    → 발견된 열화상 카메라: {thermal_list}")
        print(f"    → 시도한 RGB 후보: {rgb_candidates}")
        print(f"    → 예상: /dev/video2 또는 /dev/video3에 USB Camera가 있어야 합니다")
        print(f"\n    수동 확인:")
        print(f"    → ls -l /dev/video*")
        print(f"    → for dev in /sys/class/video4linux/video*; do echo \"$(basename $dev): $(cat $dev/name)\"; done")
        print(f"    → lsof /dev/video*")
        cap_thermal.release()
        return False
    
    # RGB 카메라 ID 저장
    RGB_CAMERA_ID = rgb_camera_id
    
    # RGB 카메라 재초기화 (확실하게)
    cap_rgb.release()
    time.sleep(0.5)
    cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
    cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # RGB 카메라 버퍼 최소화 (지연 감소)
    try:
        cap_rgb.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except:
        pass
    
    # 카메라 워밍업 (버퍼 비우기)
    print("  → 카메라 워밍업...")
    print("    → RGB 카메라 워밍업...")
    for _ in range(5):
        cap_rgb.read()
        time.sleep(0.05)
    
    print("    → 열화상 카메라 워밍업 (느린 응답 대비)...")
    # 열화상은 느리므로 더 긴 대기 시간
    for i in range(5):
        ret, _ = cap_thermal.read()
        if ret:
            print(f"      → 프레임 {i+1}/5 읽기 성공")
        else:
            print(f"      → 프레임 {i+1}/5 읽기 실패 (재시도 중...)")
        time.sleep(0.2)  # 열화상은 느리므로 더 긴 대기
    
    # 테스트 (타임아웃 대비)
    print("    → 최종 테스트...")
    ret2, f2 = cap_rgb.read()
    if not ret2 or f2 is None:
        print("    ✗ RGB 카메라 테스트 실패")
        return False
    
    # 열화상은 여러 번 시도 (타임아웃 대비)
    ret1, f1 = None, None
    for attempt in range(3):
        ret1, f1 = cap_thermal.read()
        if ret1 and f1 is not None:
            break
        print(f"      → 열화상 읽기 시도 {attempt+1}/3...")
        time.sleep(0.3)
    
    if not ret2 or f2 is None:
        print("    ✗ RGB 프레임 읽기 실패")
        return False
    
    print(f"    ✓ RGB: {f2.shape}")
    
    if not ret1 or f1 is None:
        print("    ⚠ 열화상 프레임 읽기 실패 (나중에 재시도됨)")
        print("    → RGB만 먼저 시작, 열화상은 백그라운드에서 재연결 시도")
    else:
        print(f"    ✓ 열화상: {f1.shape}")
    
    # RGB 카메라가 제대로 읽히는지 확인
    if f2.shape[0] < 100 or f2.shape[1] < 100:
        print(f"    ⚠ RGB 카메라 해상도가 작습니다: {f2.shape}")
        print(f"    → RGB 카메라 설정 확인 필요")
    
    return True

# ============================================================
# 영상 처리 (최적화)
# ============================================================
# 마커 템플릿 미리 생성
MARKER_TEMPLATE = None

def create_marker_template():
    """마커 템플릿 미리 생성"""
    global MARKER_TEMPLATE
    template = np.zeros((THERMAL_CROPPED_HEIGHT, THERMAL_WIDTH, 3), dtype=np.uint8)
    
    # 기본 마커 (흰색으로 생성, 나중에 색상 변경)
    cv2.circle(template, CENTER, 10, (255, 255, 255), 1)
    cv2.circle(template, CENTER, RADIUS_OUTER, (255, 255, 255), 1)
    cv2.circle(template, CENTER, RADIUS_INNER, (255, 255, 255), 1)
    
    extend_radius = RADIUS_OUTER + 20
    cv2.line(template, (CENTER[0]+10, CENTER[1]), (CENTER[0]+extend_radius, CENTER[1]), (255, 255, 255), 2)
    cv2.line(template, (CENTER[0]-extend_radius, CENTER[1]), (CENTER[0]-10, CENTER[1]), (255, 255, 255), 2)
    cv2.line(template, (CENTER[0], CENTER[1]+10), (CENTER[0], CENTER[1]+extend_radius), (255, 255, 255), 2)
    cv2.line(template, (CENTER[0], CENTER[1]-extend_radius), (CENTER[0], CENTER[1]-10), (255, 255, 255), 2)
    
    MARKER_TEMPLATE = template

def extract_thermal_data(frame_thermal):
    """열화상 프레임에서 데이터 추출 (화점, 온도 등)"""
    global thermal_data, thermal_data_lock
    
    if frame_thermal is None or len(frame_thermal.shape) < 2 or frame_thermal.size == 0:
        return False
    
    try:
        # 열화상 데이터 추출 (화점 검출용)
        frame_thermal_small = cv2.resize(frame_thermal, (THERMAL_WIDTH, THERMAL_HEIGHT), 
                                          interpolation=cv2.INTER_NEAREST)
        frame_thermal_cropped = frame_thermal_small[:THERMAL_CROPPED_HEIGHT, :]
        
        # 화점 검출 (녹색 채널)
        if len(frame_thermal_cropped.shape) < 3 or frame_thermal_cropped.shape[2] < 2:
            return False
        
        green = frame_thermal_cropped[:, :, 1]
        min_val, max_val, min_loc, max_loc_thermal = cv2.minMaxLoc(green)
        
        # RGB 영상에 타겟팅 마커 위치 계산
        rgb_center_x = RGB_CROP_X + CENTER[0]
        rgb_center_y = RGB_CROP_Y + CENTER[1]
        rgb_max_x = RGB_CROP_X + max_loc_thermal[0]
        rgb_max_y = RGB_CROP_Y + max_loc_thermal[1]
        
        # 경계 체크
        rgb_center_x = max(0, min(639, rgb_center_x))
        rgb_center_y = max(0, min(479, rgb_center_y))
        rgb_max_x = max(0, min(639, rgb_max_x))
        rgb_max_y = max(0, min(479, rgb_max_y))
        
        # 거리 계산
        dx = rgb_center_x - rgb_max_x
        dy = rgb_center_y - rgb_max_y
        distance = np.sqrt(dx*dx + dy*dy)
        
        # 색상 결정
        if distance < RADIUS_INNER:
            color = (0, 255, 0)  # 녹색
            max_color = (0, 255, 0)
        elif distance < RADIUS_OUTER:
            color = (0, 255, 255)  # 노란색
            max_color = (0, 255, 255)
        else:
            color = (255, 255, 255)  # 흰색
            max_color = (0, 0, 255)  # 빨간색
        
        rel_x, rel_y = max_loc_thermal[0] - CENTER[0], max_loc_thermal[1] - CENTER[1]
        
        # 열화상 데이터 업데이트 (thread-safe)
        with thermal_data_lock:
            thermal_data['hotspot_x'] = rgb_max_x
            thermal_data['hotspot_y'] = rgb_max_y
            thermal_data['center_x'] = rgb_center_x
            thermal_data['center_y'] = rgb_center_y
            thermal_data['max_val'] = max_val
            thermal_data['min_val'] = min_val
            thermal_data['distance'] = distance
            thermal_data['color'] = color
            thermal_data['max_color'] = max_color
            thermal_data['rel_x'] = rel_x
            thermal_data['rel_y'] = rel_y
            thermal_data['timestamp'] = time.time()
            thermal_data['valid'] = True
        
        return True
    except Exception as e:
        print(f"  ⚠ 열화상 데이터 추출 오류: {e}")
        return False

def composite_frames_optimized(frame_rgb):
    """RGB 영상 + 저장된 열화상 데이터 표시 (이전 값 유지)"""
    global thermal_data, thermal_data_lock
    
    # 프레임 검증
    if frame_rgb is None or len(frame_rgb.shape) < 2 or frame_rgb.size == 0:
        return None
    
    # RGB 영상이 메인 - 항상 RGB 영상부터 복사
    rgb_h, rgb_w = frame_rgb.shape[:2]
    if rgb_h != 480 or rgb_w != 640:
        frame_rgb = cv2.resize(frame_rgb, (640, 480), interpolation=cv2.INTER_LINEAR)
    
    # RGB 영상 복사 (메인)
    output = frame_rgb.copy()
    
    # 저장된 열화상 데이터 가져오기 (thread-safe, 이전 값 유지)
    with thermal_data_lock:
        data = thermal_data.copy()
    
    # 열화상 데이터가 유효하면 마커 그리기
    if data['valid'] and data['center_x'] is not None:
        rgb_center_x = int(data['center_x'])
        rgb_center_y = int(data['center_y'])
        rgb_max_x = int(data['hotspot_x'])
        rgb_max_y = int(data['hotspot_y'])
        color = data['color']
        max_color = data['max_color']
        
        # 타겟팅 마커 그리기
        cv2.circle(output, (rgb_center_x, rgb_center_y), 10, color, 1)
        cv2.circle(output, (rgb_center_x, rgb_center_y), RADIUS_OUTER, color, 1)
        cv2.circle(output, (rgb_center_x, rgb_center_y), RADIUS_INNER, color, 1)
        
        extend_radius = RADIUS_OUTER + 20
        cv2.line(output, (rgb_center_x+10, rgb_center_y), 
                 (rgb_center_x+extend_radius, rgb_center_y), color, 2)
        cv2.line(output, (rgb_center_x-extend_radius, rgb_center_y), 
                 (rgb_center_x-10, rgb_center_y), color, 2)
        cv2.line(output, (rgb_center_x, rgb_center_y+10), 
                 (rgb_center_x, rgb_center_y+extend_radius), color, 2)
        cv2.line(output, (rgb_center_x, rgb_center_y-extend_radius), 
                 (rgb_center_x, rgb_center_y-10), color, 2)
        
        # 화점 마커
        cv2.circle(output, (rgb_max_x, rgb_max_y), 20, max_color, 2)
        
        # 열화상 데이터 텍스트 표시 (상세 정보)
        text_y = 20
        text_spacing = 25
        
        # 좌표 정보
        cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
        cv2.putText(output, f"Offset: ({data['rel_x']:3d}, {data['rel_y']:3d})", (10, text_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 온도값 정보
        text_y += text_spacing
        cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
        if data['max_val'] is not None:
            cv2.putText(output, f"Max: {data['max_val']:.1f}", (10, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 최소값 정보
        text_y += text_spacing
        cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
        if data['min_val'] is not None:
            cv2.putText(output, f"Min: {data['min_val']:.1f}", (10, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 거리 정보
        text_y += text_spacing
        cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
        if data['distance'] is not None:
            cv2.putText(output, f"Dist: {data['distance']:.1f}px", (10, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 상태 정보
        text_y += text_spacing
        if data['distance'] is not None:
            status = "INNER" if data['distance'] < RADIUS_INNER else ("OUTER" if data['distance'] < RADIUS_OUTER else "OUT")
            status_color = (0, 255, 0) if data['distance'] < RADIUS_INNER else ((0, 255, 255) if data['distance'] < RADIUS_OUTER else (0, 0, 255))
            cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
            cv2.putText(output, f"Status: {status}", (10, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
    
    # 출력 크기로 리사이즈 (필요시에만)
    if output.shape[:2] != (OUTPUT_HEIGHT, OUTPUT_WIDTH):
        output = cv2.resize(output, (OUTPUT_WIDTH, OUTPUT_HEIGHT), interpolation=cv2.INTER_LINEAR)
    
    return output
    
    # 거리 정보
    text_y += text_spacing
    cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
    cv2.putText(output, f"Dist: {distance:.1f}px", (10, text_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 상태 정보
    text_y += text_spacing
    status = "INNER" if distance < RADIUS_INNER else ("OUTER" if distance < RADIUS_OUTER else "OUT")
    status_color = (0, 255, 0) if distance < RADIUS_INNER else ((0, 255, 255) if distance < RADIUS_OUTER else (0, 0, 255))
    cv2.rectangle(output, (5, text_y-15), (200, text_y+10), (0, 0, 0), -1)
    cv2.putText(output, f"Status: {status}", (10, text_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
    
    # 10. 출력 크기로 리사이즈 (필요시에만)
    if output.shape[:2] != (OUTPUT_HEIGHT, OUTPUT_WIDTH):
        output = cv2.resize(output, (OUTPUT_WIDTH, OUTPUT_HEIGHT), interpolation=cv2.INTER_LINEAR)
    
    return output

def rgb_capture_thread():
    """RGB 카메라 캡처 스레드 (30 FPS 목표)"""
    global cap_rgb, is_running
    
    frame_interval = 1.0 / RGB_TARGET_FPS  # 30 FPS = 0.033초 간격
    last_frame_time = time.time()
    
    while is_running:
        try:
            current_time = time.time()
            elapsed = current_time - last_frame_time
            
            # 30 FPS 유지 (너무 빠르면 대기)
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
                continue
            
            last_frame_time = time.time()
            
            if cap_rgb is None or not cap_rgb.isOpened():
                time.sleep(0.1)
                continue
            
            ret, frame_rgb = cap_rgb.read()
            
            if not ret or frame_rgb is None or frame_rgb.size == 0:
                print("  ⚠ RGB 프레임 읽기 실패, 재연결...")
                # 카메라가 사용 중인지 확인
                if RGB_CAMERA_ID is not None:
                    device = f"/dev/video{RGB_CAMERA_ID}"
                    try:
                        result = subprocess.run(['lsof', device], 
                                              capture_output=True, text=True, timeout=1)
                        if result.returncode == 0 and result.stdout:
                            print(f"    ⚠ {device} 사용 중인 프로세스 확인 중...")
                            # 사용 중인 프로세스 종료
                            lines = result.stdout.strip().split('\n')
                            for line in lines[1:]:
                                parts = line.split()
                                if len(parts) >= 2:
                                    pid = parts[1]
                                    try:
                                        if int(pid) != os.getpid():
                                            subprocess.run(['kill', '-9', pid], 
                                                         capture_output=True, timeout=1)
                                    except:
                                        pass
                    except:
                        pass
                
                cap_rgb.release()
                time.sleep(0.5)
                if RGB_CAMERA_ID is not None:
                    cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
                    cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                time.sleep(0.1)
                continue
            
            # RGB 프레임 크기 확인 및 리사이즈
            rgb_h, rgb_w = frame_rgb.shape[:2]
            
            # RGB 프레임이 너무 작으면 (열화상일 가능성) 스킵
            if rgb_h < 200 or rgb_w < 200:
                continue
            
            if rgb_h != 480 or rgb_w != 640:
                frame_rgb = cv2.resize(frame_rgb, (640, 480), interpolation=cv2.INTER_LINEAR)
            
            # 큐에 넣기 (가득 차면 오래된 것 버림)
            if rgb_frame_queue.full():
                try:
                    rgb_frame_queue.get_nowait()
                except:
                    pass
            rgb_frame_queue.put(frame_rgb)
            
        except Exception as e:
            print(f"  ⚠ RGB 캡처 오류: {e}")
            time.sleep(0.1)
    
    if cap_rgb:
        cap_rgb.release()

def thermal_capture_thread():
    """열화상 카메라 캡처 스레드 (비동기)"""
    global cap_thermal, is_running
    
    while is_running:
        try:
            if cap_thermal is None or not cap_thermal.isOpened():
                time.sleep(0.1)
                continue
            
            ret, frame_thermal = cap_thermal.read()
            
            if not ret or frame_thermal is None or frame_thermal.size == 0:
                # 재연결
                cap_thermal.release()
                cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
                time.sleep(0.1)
                continue
            
            # 열화상 데이터 추출 및 저장 (이전 값 유지)
            extract_thermal_data(frame_thermal)
            
            # Lepton 3.5는 8-9 FPS이므로 적절한 대기
            time.sleep(0.1)  # 약 10 FPS로 처리
            
        except Exception as e:
            print(f"  ⚠ 열화상 캡처 오류: {e}")
            time.sleep(0.1)
    
def composite_thread():
    """프레임 합성 스레드 (RGB 우선, 저장된 열화상 데이터 사용)"""
    global is_running
    
    frame_count = 0
    last_time = time.time()
    frame_interval = 1.0 / RGB_TARGET_FPS  # 30 FPS 유지
    last_frame_time = time.time()
    
    while is_running:
        try:
            current_time = time.time()
            elapsed = current_time - last_frame_time
            
            # 30 FPS 유지 (너무 빠르면 대기)
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
                continue
            
            last_frame_time = time.time()
            
            # RGB 프레임 가져오기 (최신 것만)
            frame_rgb = None
            if not rgb_frame_queue.empty():
                try:
                    frame_rgb = rgb_frame_queue.get_nowait()
                    # 큐에 더 있으면 모두 버리고 최신 것만 사용
                    while not rgb_frame_queue.empty():
                        try:
                            rgb_frame_queue.get_nowait()
                        except:
                            break
                except:
                    pass
            
            # RGB 프레임이 없으면 스킵
            if frame_rgb is None or frame_rgb.size == 0:
                time.sleep(0.01)
                continue
            
            # 프레임 합성 (저장된 열화상 데이터 사용, 이전 값 유지)
            composite = composite_frames_optimized(frame_rgb)
            
            if composite is None or composite.size == 0:
                time.sleep(0.01)
                continue
            
            # BGR → RGB
            composite = cv2.cvtColor(composite, cv2.COLOR_BGR2RGB)
            
            # 큐에 넣기 (가득 차면 오래된 것 버림)
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except:
                    pass
            frame_queue.put(composite)
            
            # FPS 계산
            frame_count += 1
            now = time.time()
            if now - last_time >= 5.0:
                fps = frame_count / (now - last_time)
                print(f"  → 합성 FPS: {fps:.1f} (목표: {RGB_TARGET_FPS} FPS)")
                if frame_rgb is not None:
                    print(f"     RGB: {frame_rgb.shape}, Output: {composite.shape}")
                frame_count = 0
                last_time = now
                
        except Exception as e:
            print(f"  ⚠ 합성 오류: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)

# ============================================================
# 파이프라인
# ============================================================
def get_launch_string():
    """인코더에 맞는 파이프라인 생성"""
    base = (
        f"appsrc name=src is-live=true format=time do-timestamp=true "
        f"caps=video/x-raw,format=RGB,width={OUTPUT_WIDTH},height={OUTPUT_HEIGHT},framerate={OUTPUT_FPS}/1 ! "
        f"videoconvert ! "
    )
    
    if USE_HARDWARE_ENCODER and check_encoder("amlvenc"):
        enc = f"video/x-raw,format=NV12 ! amlvenc bitrate={BITRATE_KBPS} gop={OUTPUT_FPS} ! "
    elif check_encoder("openh264enc"):
        enc = f"video/x-raw,format=I420 ! openh264enc bitrate={BITRATE_KBPS * 1000} complexity=0 ! "
    elif check_encoder("x264enc"):
        enc = f"video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate={BITRATE_KBPS} speed-preset=ultrafast ! "
    else:
        enc = f"video/x-raw,format=I420 ! openh264enc bitrate={BITRATE_KBPS * 1000} ! "
    
    tail = "video/x-h264,profile=baseline ! h264parse config-interval=1 ! rtph264pay name=pay0 pt=96"
    
    return base + enc + tail

# ============================================================
# RTSP 팩토리
# ============================================================
class CompositeRTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, launch_string):
        super().__init__()
        self.launch_string = launch_string
        self.set_shared(True)
        self.appsrc = None
        self.push_thread = None
        
    def do_create_element(self, url):
        pipeline = Gst.parse_launch(self.launch_string)
        self.appsrc = pipeline.get_by_name("src")
        
        if self.appsrc:
            self.appsrc.set_property("format", Gst.Format.TIME)
            self.appsrc.set_property("is-live", True)
            self.appsrc.set_property("do-timestamp", True)
            
            if self.push_thread is None or not self.push_thread.is_alive():
                self.push_thread = threading.Thread(target=self.push_frames, daemon=True)
                self.push_thread.start()
        
        return pipeline
    
    def push_frames(self):
        global frame_queue, is_running
        
        if self.appsrc is None:
            return
        
        frame_duration = Gst.SECOND // OUTPUT_FPS
        timestamp = 0
        
        while is_running:
            try:
                frame = frame_queue.get(timeout=0.2)
                
                if frame is None or frame.size == 0:
                    continue
                
                data = frame.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.pts = timestamp
                buf.dts = timestamp
                buf.duration = frame_duration
                
                ret = self.appsrc.emit("push-buffer", buf)
                if ret != Gst.FlowReturn.OK:
                    if ret == Gst.FlowReturn.FLUSHING:
                        break
                
                timestamp += frame_duration
                
            except Exception as e:
                if "timeout" not in str(e).lower():
                    print(f"  ⚠ 푸시 오류: {e}")

# ============================================================
# 시그널 핸들러
# ============================================================
def signal_handler(sig, frame):
    global loop, is_running
    print("\n\n[종료 요청]")
    is_running = False
    
    if loop and loop.is_running():
        loop.quit()
    
    def force_exit():
        time.sleep(2)
        os._exit(0)
    threading.Thread(target=force_exit, daemon=True).start()

# ============================================================
# 메인
# ============================================================
def main():
    global server, loop, is_running
    
    print("=" * 60)
    print("  RGB + 열화상 데이터 RTSP (저지연 최적화)")
    print("=" * 60)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("\n[초기화]")
    kill_existing_processes()
    
    if not init_cameras():
        sys.exit(1)
    print(f"  ✓ 출력: {OUTPUT_WIDTH}x{OUTPUT_HEIGHT} @ {OUTPUT_FPS}fps")
    
    # 카메라 스레드 시작
    # 비동기 카메라 캡처 스레드 시작
    rgb_thread = threading.Thread(target=rgb_capture_thread, daemon=True)
    thermal_thread = threading.Thread(target=thermal_capture_thread, daemon=True)
    composite_thread_obj = threading.Thread(target=composite_thread, daemon=True)
    
    rgb_thread.start()
    thermal_thread.start()
    composite_thread_obj.start()
    
    print("  → 비동기 카메라 캡처 시작 (RGB, 열화상 독립 스레드)")
    time.sleep(1)
    
    # 인코더 확인
    print("\n[인코더]")
    if check_encoder("openh264enc"):
        print("  ✓ openh264enc")
    elif check_encoder("x264enc"):
        print("  ✓ x264enc")
    
    launch_string = get_launch_string()
    print(f"\n[파이프라인]\n  {launch_string[:70]}...")
    
    # 서버 시작
    print("\n[서버 시작]")
    try:
        server = GstRtspServer.RTSPServer()
        server.set_service(RTSP_PORT)
        
        factory = CompositeRTSPMediaFactory(launch_string)
        mount_points = server.get_mount_points()
        mount_points.add_factory(RTSP_MOUNT_POINT, factory)
        server.attach(None)
        
    except Exception as e:
        print(f"  ✗ 실패: {e}")
        sys.exit(1)
    
    local_ip = get_local_ip()
    rtsp_url = f"rtsp://{local_ip}:{RTSP_PORT}{RTSP_MOUNT_POINT}"
    
    print("\n" + "=" * 60)
    print("  ✓ RTSP 서버 시작됨")
    print("=" * 60)
    print(f"\n  ★ URL: {rtsp_url}")
    print(f"  ★ FPS: {OUTPUT_FPS} (Lepton 3.5 기준)")
    print("\n" + "-" * 60)
    print(f"  QGC: RTSP URL → {rtsp_url}")
    print(f"  VLC: vlc {rtsp_url}")
    print("=" * 60)
    print("\n대기 중... (Ctrl+C: 종료)\n")
    
    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass
    
    print("\n[완료]")

if __name__ == "__main__":
    main()