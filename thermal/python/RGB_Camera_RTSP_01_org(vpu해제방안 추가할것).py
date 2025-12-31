#!/usr/bin/env python3
"""
==========================================================================
RGB 카메라 RTSP 스트리밍 서버
==========================================================================
QGC 설정:
  - Video Source: "RTSP 비디오 스트림"
  - RTSP URL: rtsp://<VIM4_IP>:8554/stream

VLC 테스트:
  vlc rtsp://<VIM4_IP>:8554/stream

필요 패키지:
  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
                   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                   gstreamer1.0-plugins-ugly libgstrtspserver-1.0-0 \
                   gir1.2-gst-rtsp-server-1.0 v4l-utils
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

# 전역 변수
server = None
loop = None

# ============================================================
# 카메라 초기화/정리
# ============================================================
def reset_vpu():
    """VPU (하드웨어 인코더) 리셋"""
    print("  → VPU 인코더 리셋...")
    
    # 방법 1: encode_status 리셋
    vpu_status_path = "/sys/class/amvenc/amvenc_multi/encode_status"
    if os.path.exists(vpu_status_path):
        try:
            subprocess.run(['sudo', 'tee', vpu_status_path], 
                          input=b'1', capture_output=True, timeout=3)
            print("    ✓ VPU encode_status 리셋")
        except:
            pass
    
    # 방법 2: amvenc 모듈 언로드/로드
    try:
        subprocess.run(['sudo', 'rmmod', 'amvenc_multi'], 
                      capture_output=True, timeout=5)
        time.sleep(0.3)
        subprocess.run(['sudo', 'modprobe', 'amvenc_multi'], 
                      capture_output=True, timeout=5)
        print("    ✓ amvenc_multi 모듈 재로드")
    except:
        pass
    
    time.sleep(0.5)

def reset_usb_camera():
    """USB 카메라 초기화 (이전 연결 정리)"""
    global CAMERA_DEVICE
    
    print("\n[카메라 초기화]")
    
    # 1. 기존 스트리밍 프로세스 종료
    print("  → 기존 스트리밍 프로세스 정리...")
    try:
        subprocess.run(['pkill', '-9', '-f', 'gst-launch'], 
                      capture_output=True, timeout=3)
    except:
        pass
    
    try:
        subprocess.run(['pkill', '-9', '-f', 'udp_h264_server'], 
                      capture_output=True, timeout=3)
    except:
        pass
    
    try:
        subprocess.run(['pkill', '-9', '-f', 'rtsp_server'], 
                      capture_output=True, timeout=3)
    except:
        pass
    
    time.sleep(0.3)
    
    # 2. VPU 리셋 (하드웨어 인코더)
    reset_vpu()
    
    # 3. 카메라 존재 확인
    if not os.path.exists(CAMERA_DEVICE):
        print(f"  ✗ {CAMERA_DEVICE} 를 찾을 수 없습니다")
        
        # USB 디바이스 재검색
        print("  → USB 디바이스 재검색...")
        for i in range(5):
            dev = f"/dev/video{i}"
            if os.path.exists(dev):
                print(f"  → {dev} 발견")
                CAMERA_DEVICE = dev
                break
        else:
            return False
    
    # 4. v4l2 디바이스 상태 확인
    print(f"  → {CAMERA_DEVICE} 상태 확인...")
    try:
        result = subprocess.run(
            ['v4l2-ctl', '-d', CAMERA_DEVICE, '--info'], 
            capture_output=True, 
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'Driver name' in line or 'Card type' in line:
                    print(f"    {line.strip()}")
    except Exception as e:
        print(f"  ⚠ v4l2-ctl 실행 실패: {e}")
    
    # 5. 잠시 대기
    time.sleep(0.5)
    
    # 6. 카메라 접근 테스트
    print("  → 카메라 접근 테스트...")
    test_pipeline_str = f"v4l2src device={CAMERA_DEVICE} num-buffers=5 ! fakesink"
    try:
        test_pipeline = Gst.parse_launch(test_pipeline_str)
        ret = test_pipeline.set_state(Gst.State.PLAYING)
        
        if ret == Gst.StateChangeReturn.FAILURE:
            print("  ✗ 카메라 접근 실패")
            test_pipeline.set_state(Gst.State.NULL)
            return False
        
        # 상태 전환 대기
        time.sleep(1.0)
        
        # 정리
        test_pipeline.set_state(Gst.State.NULL)
        test_pipeline.get_state(Gst.CLOCK_TIME_NONE)
        del test_pipeline
        
        # GC 대기
        time.sleep(0.5)
        
        print("  ✓ 카메라 초기화 완료")
        return True
        
    except Exception as e:
        print(f"  ✗ 카메라 테스트 실패: {e}")
        return False

def cleanup():
    """서버 및 리소스 정리"""
    global server
    
    print("\n[정리 중]")
    
    if server:
        print("  → 서버 정리...")
        del server
        server = None
    
    # GC 대기
    time.sleep(0.5)
    
    # GStreamer 디초기화
    Gst.deinit()
    
    print("  ✓ 정리 완료")

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

# ============================================================
# 파이프라인 생성
# ============================================================
def get_launch_string_amlvenc():
    """VIM4 하드웨어 인코더 파이프라인"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! "
        f"video/x-raw,format=NV12 ! "
        f"amlvenc bitrate={BITRATE_KBPS} gop=30 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

def get_launch_string_openh264():
    """OpenH264 소프트웨어 인코더 파이프라인"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! "
        f"video/x-raw,format=I420 ! "
        f"openh264enc bitrate={BITRATE_KBPS * 1000} complexity=0 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

def get_launch_string_x264():
    """x264 소프트웨어 인코더 파이프라인"""
    return (
        f"( "
        f"v4l2src device={CAMERA_DEVICE} ! "
        f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1 ! "
        f"videoconvert ! "
        f"video/x-raw,format=I420 ! "
        f"x264enc tune=zerolatency bitrate={BITRATE_KBPS} speed-preset=ultrafast ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
        f")"
    )

# ============================================================
# 시그널 핸들러
# ============================================================
def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    global loop
    print("\n\n중지 요청...")
    if loop:
        loop.quit()

# ============================================================
# 메인
# ============================================================
def main():
    global server, loop
    
    print("=" * 60)
    print("  RGB 카메라 RTSP 스트리밍 서버")
    print("=" * 60)
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 카메라 초기화
    if not reset_usb_camera():
        print("\n✗ 카메라 초기화 실패")
        sys.exit(1)
    
    print(f"\n[카메라] {CAMERA_DEVICE}")
    print(f"  해상도: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
    
    # 인코더 선택
    print("\n[인코더 확인]")
    launch_string = None
    
    if check_encoder("amlvenc"):
        print("  ✓ amlvenc (VIM4 하드웨어 인코더) 사용")
        launch_string = get_launch_string_amlvenc()
    elif check_encoder("openh264enc"):
        print("  ✓ openh264enc (소프트웨어 인코더) 사용")
        launch_string = get_launch_string_openh264()
    elif check_encoder("x264enc"):
        print("  ✓ x264enc (소프트웨어 인코더) 사용")
        launch_string = get_launch_string_x264()
    else:
        print("  ✗ 사용 가능한 H.264 인코더가 없습니다")
        sys.exit(1)
    
    print(f"\n[파이프라인]")
    print(f"  {launch_string[:70]}...")
    
    # RTSP 서버 생성
    print("\n[RTSP 서버 시작 중...]")
    try:
        server = GstRtspServer.RTSPServer()
        server.set_service(RTSP_PORT)
        
        # 미디어 팩토리 생성
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(launch_string)
        factory.set_shared(True)  # 여러 클라이언트 공유
        
        # 마운트 포인트 설정
        mount_points = server.get_mount_points()
        mount_points.add_factory(RTSP_MOUNT_POINT, factory)
        
        # 서버 시작
        server.attach(None)
        
    except Exception as e:
        print(f"\n✗ RTSP 서버 생성 실패: {e}")
        sys.exit(1)
    
    local_ip = get_local_ip()
    rtsp_url = f"rtsp://{local_ip}:{RTSP_PORT}{RTSP_MOUNT_POINT}"
    
    print("\n" + "=" * 60)
    print("  ✓ RTSP 서버 시작됨")
    print("=" * 60)
    print(f"\n  ★ RTSP URL: {rtsp_url}")
    print("\n" + "-" * 60)
    print("  QGC 설정:")
    print("    1. Settings > General > Video")
    print("    2. Video Source: 'RTSP 비디오 스트림'")
    print(f"    3. RTSP URL: {rtsp_url}")
    print("-" * 60)
    print(f"  VLC 테스트: vlc {rtsp_url}")
    print("-" * 60)
    print("  GStreamer 테스트:")
    print(f"    gst-launch-1.0 rtspsrc location={rtsp_url} latency=0 ! \\")
    print(f"      decodebin ! autovideosink")
    print("=" * 60)
    print("\n서버 실행 중... (Ctrl+C로 종료)")
    print("클라이언트 접속 대기 중...\n")
    
    # 메인 루프 실행
    loop = GLib.MainLoop()
    
    try:
        loop.run()
    except:
        pass
    
    # 정리
    cleanup()
    print("\n✓ 서버 종료 완료")

if __name__ == "__main__":
    main()