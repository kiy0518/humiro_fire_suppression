#!/usr/bin/env python3
"""
==========================================================================
RGB + 열화상 카메라 합성 RTSP 스트리밍 서버 (최적화 버전)
==========================================================================
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

# 카메라 설정
THERMAL_CAMERA_ID = 2  # 열화상 카메라 (Lepton 3.5)
RGB_CAMERA_ID = 0       # RGB 카메라

# 출력 해상도 - Lepton 3.5 FPS에 맞춤
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
OUTPUT_FPS = 9  # ★ Lepton 3.5: 8~9 FPS

# 열화상 오버레이 설정
SCALE = 0.75
THERMAL_WIDTH = int(640 * SCALE)  # 480
THERMAL_HEIGHT = int(480 * SCALE)  # 360
THERMAL_DX = 0
THERMAL_DY = -10
CUT_PIXELS = 30
ALPHA_THERMAL_LAYER = 0.6

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

def kill_existing_processes():
    print("  → 기존 프로세스 정리...")
    for proc in ['gst-launch', 'rtsp_server', 'thermal_rtsp']:
        try:
            subprocess.run(['pkill', '-9', '-f', proc], capture_output=True, timeout=2)
        except:
            pass
    time.sleep(0.5)

def init_cameras():
    """카메라 초기화"""
    global cap_thermal, cap_rgb
    
    print("  → 열화상 카메라 초기화...")
    cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
    if not cap_thermal.isOpened():
        print(f"    ✗ 열화상 카메라 ({THERMAL_CAMERA_ID}) 실패")
        return False
    
    print("  → RGB 카메라 초기화...")
    cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
    if not cap_rgb.isOpened():
        print(f"    ✗ RGB 카메라 ({RGB_CAMERA_ID}) 실패")
        cap_thermal.release()
        return False
    
    cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 카메라 워밍업 (버퍼 비우기)
    print("  → 카메라 워밍업...")
    for _ in range(5):
        cap_thermal.read()
        cap_rgb.read()
    
    # 테스트
    ret1, f1 = cap_thermal.read()
    ret2, f2 = cap_rgb.read()
    
    if not ret1 or not ret2:
        print("    ✗ 프레임 읽기 실패")
        return False
    
    print(f"    ✓ 열화상: {f1.shape}")
    print(f"    ✓ RGB: {f2.shape}")
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

def composite_frames_optimized(frame_thermal, frame_rgb):
    """최적화된 프레임 합성"""
    
    # 1. RGB 리사이즈 (필요시에만)
    if frame_rgb.shape[:2] != (480, 640):
        frame_rgb = cv2.resize(frame_rgb, (640, 480), interpolation=cv2.INTER_NEAREST)
    
    # 2. 열화상 리사이즈 및 크롭 (한번에)
    frame_thermal_resized = cv2.resize(frame_thermal, (THERMAL_WIDTH, THERMAL_HEIGHT), 
                                        interpolation=cv2.INTER_NEAREST)
    frame_thermal_cropped = frame_thermal_resized[:THERMAL_CROPPED_HEIGHT, :]
    
    # 3. 화점 검출 (녹색 채널만)
    green = frame_thermal_cropped[:, :, 1]
    _, max_val, _, max_loc = cv2.minMaxLoc(green)
    
    # 4. 거리 계산
    dx = CENTER[0] - max_loc[0]
    dy = CENTER[1] - max_loc[1]
    distance = np.sqrt(dx*dx + dy*dy)
    
    # 5. 색상 결정
    if distance < RADIUS_INNER:
        color = (0, 255, 0)  # 녹색
    elif distance < RADIUS_OUTER:
        color = (0, 255, 255)  # 노란색
    else:
        color = (255, 255, 255)  # 흰색
    
    # 6. RGB 크롭
    y_end = RGB_CROP_Y + THERMAL_CROPPED_HEIGHT
    x_end = RGB_CROP_X + THERMAL_WIDTH
    
    # 경계 체크
    y_start = max(0, RGB_CROP_Y)
    x_start = max(0, RGB_CROP_X)
    y_end = min(480, y_end)
    x_end = min(640, x_end)
    
    frame_rgb_cropped = frame_rgb[y_start:y_end, x_start:x_end]
    
    # 크기 맞추기
    if frame_rgb_cropped.shape[:2] != (THERMAL_CROPPED_HEIGHT, THERMAL_WIDTH):
        frame_rgb_cropped = cv2.resize(frame_rgb_cropped, (THERMAL_WIDTH, THERMAL_CROPPED_HEIGHT),
                                        interpolation=cv2.INTER_NEAREST)
    
    # 7. 합성 (addWeighted)
    overlay = cv2.addWeighted(frame_rgb_cropped, ALPHA_THERMAL_LAYER, 
                              frame_thermal_cropped, 1 - ALPHA_THERMAL_LAYER, 0)
    
    # 8. 마커 그리기 (직접 그리기 - 템플릿 대신)
    cv2.circle(overlay, CENTER, 10, color, 1)
    cv2.circle(overlay, CENTER, RADIUS_OUTER, color, 1)
    cv2.circle(overlay, CENTER, RADIUS_INNER, color, 1)
    
    extend_radius = RADIUS_OUTER + 20
    cv2.line(overlay, (CENTER[0]+10, CENTER[1]), (CENTER[0]+extend_radius, CENTER[1]), color, 2)
    cv2.line(overlay, (CENTER[0]-extend_radius, CENTER[1]), (CENTER[0]-10, CENTER[1]), color, 2)
    cv2.line(overlay, (CENTER[0], CENTER[1]+10), (CENTER[0], CENTER[1]+extend_radius), color, 2)
    cv2.line(overlay, (CENTER[0], CENTER[1]-extend_radius), (CENTER[0], CENTER[1]-10), color, 2)
    
    # 화점 마커
    max_color = (0, 255, 0) if distance < RADIUS_INNER else ((0, 255, 255) if distance < RADIUS_OUTER else (0, 0, 255))
    cv2.circle(overlay, max_loc, 20, max_color, 2)
    
    # 9. 텍스트 (간소화)
    rel_x, rel_y = max_loc[0] - CENTER[0], max_loc[1] - CENTER[1]
    cv2.putText(overlay, f"({rel_x},{rel_y})", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(overlay, f"Max:{max_val:.0f}", (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 10. 출력 크기로 리사이즈
    if overlay.shape[:2] != (OUTPUT_HEIGHT, OUTPUT_WIDTH):
        overlay = cv2.resize(overlay, (OUTPUT_WIDTH, OUTPUT_HEIGHT), interpolation=cv2.INTER_LINEAR)
    
    return overlay

def camera_thread():
    """카메라 스레드 (열화상 FPS에 맞춤)"""
    global cap_thermal, cap_rgb, is_running
    
    frame_count = 0
    last_time = time.time()
    
    while is_running:
        # 열화상 프레임 읽기 (blocking - 8~9 FPS가 속도 결정)
        ret1, frame_thermal = cap_thermal.read()
        
        if not ret1:
            # 재연결
            cap_thermal.release()
            cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
            time.sleep(0.1)
            continue
        
        # RGB 프레임 읽기 (최신 프레임)
        ret2, frame_rgb = cap_rgb.read()
        
        if not ret2:
            cap_rgb.release()
            cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
            cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            time.sleep(0.1)
            continue
        
        try:
            # 프레임 합성
            composite = composite_frames_optimized(frame_thermal, frame_rgb)
            
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
                print(f"  → 캡처 FPS: {fps:.1f}")
                frame_count = 0
                last_time = now
                
        except Exception as e:
            print(f"  ⚠ 합성 오류: {e}")
    
    if cap_thermal:
        cap_thermal.release()
    if cap_rgb:
        cap_rgb.release()

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
    print("  RGB + 열화상 합성 RTSP (Lepton 3.5 최적화)")
    print("=" * 60)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("\n[초기화]")
    kill_existing_processes()
    
    if not init_cameras():
        sys.exit(1)
    print(f"  ✓ 출력: {OUTPUT_WIDTH}x{OUTPUT_HEIGHT} @ {OUTPUT_FPS}fps")
    
    # 카메라 스레드 시작
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()
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