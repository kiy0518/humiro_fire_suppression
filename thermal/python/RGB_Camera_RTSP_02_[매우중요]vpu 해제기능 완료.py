#!/usr/bin/env python3
"""
==========================================================================
RGB 카메라 RTSP 스트리밍 서버
==========================================================================
QGC 설정:
  - Video Source: "RTSP 비디오 스트림"
  - RTSP URL: rtsp://<VIM4_IP>:8554/stream

필요 패키지:
  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
                   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                   gstreamer1.0-plugins-ugly libgstrtspserver-1.0-0 \
                   gir1.2-gst-rtsp-server-1.0
==========================================================================
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

import sys
import signal
import socket
import subprocess
import time
import os
import threading

# GStreamer 초기화
Gst.init(None)

# ============================================================
# 설정
# ============================================================
RTSP_PORT = "8554"
RTSP_MOUNT_POINT = "/stream"

# 카메라 설정
CAMERA_DEVICE = "/dev/video0"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# 인코더 설정
BITRATE_KBPS = 2000  # kbps

# ★★★ 하드웨어 인코더 사용 여부 ★★★
# VPU 문제 발생 시 False로 설정
USE_HARDWARE_ENCODER = False

# 전역 변수
server = None
loop = None
is_running = True

# ============================================================
# 유틸리티
# ============================================================
def get_local_ip():
    """로컬 IP 주소 가져오기"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

def check_encoder(encoder_name):
    """인코더 사용 가능 여부 확인"""
    factory = Gst.ElementFactory.find(encoder_name)
    return factory is not None

def kill_existing_processes():
    """기존 스트리밍 프로세스 종료"""
    print("  → 기존 프로세스 정리...")
    
    processes = ['gst-launch', 'rtsp_server', 'udp_h264_server']
    for proc in processes:
        try:
            subprocess.run(['pkill', '-9', '-f', proc], 
                          capture_output=True, timeout=2)
        except:
            pass
    
    time.sleep(0.5)

def check_camera():
    """카메라 확인"""
    global CAMERA_DEVICE
    
    if os.path.exists(CAMERA_DEVICE):
        return True
    
    # 다른 비디오 디바이스 검색
    for i in range(5):
        dev = f"/dev/video{i}"
        if os.path.exists(dev):
            CAMERA_DEVICE = dev
            return True
    
    return False

# ============================================================
# 파이프라인 생성
# ============================================================
def get_launch_string_amlvenc():
    """VIM4 하드웨어 인코더"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=NV12 ! "
        f"amlvenc bitrate={BITRATE_KBPS} gop=30 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

def get_launch_string_openh264():
    """OpenH264 소프트웨어 인코더"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"openh264enc bitrate={BITRATE_KBPS * 1000} complexity=0 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

def get_launch_string_x264():
    """x264 소프트웨어 인코더"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"x264enc tune=zerolatency bitrate={BITRATE_KBPS} speed-preset=ultrafast ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

def select_encoder():
    """인코더 선택"""
    print("\n[인코더 선택]")
    
    # 하드웨어 인코더
    if USE_HARDWARE_ENCODER:
        if check_encoder("amlvenc"):
            print("  ✓ amlvenc (하드웨어) 사용")
            return get_launch_string_amlvenc(), "amlvenc"
    
    # 소프트웨어 인코더
    if check_encoder("openh264enc"):
        print("  ✓ openh264enc (소프트웨어) 사용")
        return get_launch_string_openh264(), "openh264enc"
    
    if check_encoder("x264enc"):
        print("  ✓ x264enc (소프트웨어) 사용")
        return get_launch_string_x264(), "x264enc"
    
    # 마지막 시도: 하드웨어
    if check_encoder("amlvenc"):
        print("  ✓ amlvenc (하드웨어) 사용 - 폴백")
        return get_launch_string_amlvenc(), "amlvenc"
    
    return None, None

# ============================================================
# 시그널 핸들러
# ============================================================
def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    global loop, is_running
    
    print("\n\n[종료 요청]")
    is_running = False
    
    if loop and loop.is_running():
        loop.quit()
    
    # 강제 종료 타이머
    def force_exit():
        time.sleep(2)
        if is_running == False:
            print("  → 강제 종료...")
            os._exit(0)
    
    threading.Thread(target=force_exit, daemon=True).start()

# ============================================================
# 메인
# ============================================================
def main():
    global server, loop, is_running
    
    print("=" * 60)
    print("  RGB 카메라 RTSP 스트리밍 서버")
    print("=" * 60)
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 초기화
    print("\n[초기화]")
    kill_existing_processes()
    
    # 카메라 확인
    if not check_camera():
        print(f"  ✗ 카메라를 찾을 수 없습니다")
        sys.exit(1)
    print(f"  ✓ 카메라: {CAMERA_DEVICE}")
    print(f"    해상도: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
    
    # 인코더 선택
    launch_string, encoder_name = select_encoder()
    if launch_string is None:
        print("  ✗ 사용 가능한 인코더가 없습니다")
        sys.exit(1)
    
    print(f"\n[파이프라인]")
    print(f"  {launch_string[:60]}...")
    
    # RTSP 서버 생성
    print("\n[서버 시작]")
    try:
        server = GstRtspServer.RTSPServer()
        server.set_service(RTSP_PORT)
        
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(launch_string)
        factory.set_shared(True)
        
        mount_points = server.get_mount_points()
        mount_points.add_factory(RTSP_MOUNT_POINT, factory)
        
        server.attach(None)
        
    except Exception as e:
        print(f"  ✗ 서버 생성 실패: {e}")
        sys.exit(1)
    
    local_ip = get_local_ip()
    rtsp_url = f"rtsp://{local_ip}:{RTSP_PORT}{RTSP_MOUNT_POINT}"
    
    print("\n" + "=" * 60)
    print("  ✓ RTSP 서버 시작됨")
    print("=" * 60)
    print(f"\n  ★ RTSP URL: {rtsp_url}")
    print(f"  ★ 인코더: {encoder_name}")
    print("\n" + "-" * 60)
    print("  QGC: Video Source → RTSP 비디오 스트림")
    print(f"       RTSP URL → {rtsp_url}")
    print("-" * 60)
    print(f"  VLC: vlc {rtsp_url}")
    print("=" * 60)
    print("\n클라이언트 접속 대기 중... (Ctrl+C: 종료)\n")
    
    # 메인 루프
    loop = GLib.MainLoop()
    
    try:
        loop.run()
    except:
        pass
    
    print("\n[정리 완료]")

if __name__ == "__main__":
    main()