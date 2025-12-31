#!/usr/bin/env python3
"""
==========================================================================
RGB + 열화상 카메라 합성 RTSP 스트리밍 서버
==========================================================================
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
THERMAL_CAMERA_ID = 2  # 열화상 카메라
RGB_CAMERA_ID = 0       # RGB 카메라

# 출력 해상도
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
OUTPUT_FPS = 30

# 열화상 오버레이 설정 (참고 파일 기준)
SCALE = 0.75
THERMAL_WIDTH = int(640 * SCALE)
THERMAL_HEIGHT = int(480 * SCALE)
THERMAL_DX = 0
THERMAL_DY = -10
CUT_PIXELS = 30
ALPHA_THERMAL_LAYER = 0.6
ALPHA_MARKER_LAYER = 0.0

# 마커 설정
CENTER = (THERMAL_WIDTH // 2, THERMAL_HEIGHT // 2)
RADIUS_OUTER = 100
RADIUS_INNER = 50

# 인코더 설정
BITRATE_KBPS = 2000  # kbps

# ★★★ 하드웨어 인코더 사용 여부 ★★★
# VPU 문제 발생 시 False로 설정
USE_HARDWARE_ENCODER = False

# 전역 변수
server = None
loop = None
is_running = True
cap_thermal = None
cap_rgb = None
frame_queue = Queue(maxsize=2)
appsrc = None

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

def check_camera_connection(cap):
    """카메라 연결 상태 확인"""
    if cap is None or not cap.isOpened():
        return False
    ret, frame = cap.read()
    return ret and frame is not None

def init_cameras():
    """열화상 및 RGB 카메라 초기화"""
    global cap_thermal, cap_rgb
    
    print("  → 열화상 카메라 초기화 중...")
    cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
    if not cap_thermal.isOpened():
        print(f"    ✗ 열화상 카메라 ({THERMAL_CAMERA_ID}) 열기 실패")
        return False
    
    print("  → RGB 카메라 초기화 중...")
    cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
    if not cap_rgb.isOpened():
        print(f"    ✗ RGB 카메라 ({RGB_CAMERA_ID}) 열기 실패")
        cap_thermal.release()
        return False
    
    # RGB 카메라 해상도 설정
    cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 실제 설정된 해상도 확인
    actual_width = int(cap_rgb.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap_rgb.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 테스트 프레임 읽기
    ret1, frame1 = cap_thermal.read()
    ret2, frame2 = cap_rgb.read()
    
    if not ret1 or not ret2:
        print("    ✗ 카메라 프레임 읽기 실패")
        cap_thermal.release()
        cap_rgb.release()
        return False
    
    print(f"    ✓ 열화상 카메라: {frame1.shape}")
    print(f"    ✓ RGB 카메라: {frame2.shape} (설정: {actual_width}x{actual_height})")
    
    # RGB 카메라가 작은 해상도로 읽힌 경우 리사이즈 필요
    if frame2.shape[0] < 400 or frame2.shape[1] < 600:
        print(f"    ⚠ RGB 카메라 해상도가 작습니다. 리사이즈가 필요합니다.")
    
    return True

def draw_marker(frame, center, radius_outer, radius_inner, color):
    """마커 그리기"""
    frame = cv2.circle(frame, center, 10, color, 1)
    frame = cv2.circle(frame, center, radius_outer, color, 1)
    frame = cv2.circle(frame, center, radius_inner, color, 1)
    
    extend_radius = radius_outer + 20
    frame = cv2.line(frame, (center[0]+10, center[1]),
                     (center[0] + extend_radius, center[1]), color, 2)
    frame = cv2.line(frame, (center[0] - extend_radius, center[1]),
                     (center[0]-10, center[1]), color, 2)
    frame = cv2.line(frame, (center[0], center[1] + 10),
                     (center[0], center[1] + extend_radius), color, 2)
    frame = cv2.line(frame, (center[0], center[1]-10),
                     (center[0], center[1] - extend_radius), color, 2)
    return frame

def composite_frames(frame_thermal, frame_rgb):
    """열화상과 RGB 프레임 합성"""
    # RGB 프레임을 항상 640x480으로 리사이즈 (카메라 해상도가 작을 수 있음)
    rgb_height, rgb_width = frame_rgb.shape[:2]
    if rgb_height != 480 or rgb_width != 640:
        # RGB 프레임을 640x480으로 리사이즈
        frame_rgb = cv2.resize(frame_rgb, (640, 480), interpolation=cv2.INTER_LINEAR)
        rgb_height, rgb_width = 480, 640
    
    # 열화상 리사이즈
    frame_thermal_resized = cv2.resize(frame_thermal, (THERMAL_WIDTH, THERMAL_HEIGHT))
    frame_thermal_cropped = frame_thermal_resized[:-CUT_PIXELS, :]
    
    thermal_cropped_height, thermal_cropped_width = frame_thermal_cropped.shape[:2]
    
    # 그린 채널 추출 (열화상)
    green = frame_thermal_cropped[:, :, 1]
    
    # 최대값 위치 찾기
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(green)
    
    # 마커 색상 결정 (GPIO 없이 기본값 사용)
    distance_to_max = np.sqrt(
        (CENTER[0] - max_loc[0])**2 + (CENTER[1] - max_loc[1])**2)
    
    if distance_to_max < RADIUS_INNER:
        color = (0, 255, 0)
        max_loc_color = (0, 255, 0)
    elif distance_to_max < RADIUS_OUTER:
        color = (0, 255, 255)
        max_loc_color = (0, 255, 255)
    else:
        color = (255, 255, 255)
        max_loc_color = (0, 0, 255)
    
    # 마커 그리기
    overlay_marker = draw_marker(
        np.zeros_like(frame_thermal_cropped), CENTER, RADIUS_OUTER, RADIUS_INNER, color)
    overlay_marker = cv2.circle(overlay_marker, max_loc, 20, max_loc_color, 2)
    
    # RGB 이미지 크롭 위치 계산
    x_pos = int(((rgb_width - THERMAL_WIDTH) / 2) + THERMAL_DX)
    y_pos = int(min((rgb_height - THERMAL_HEIGHT) / 2 + THERMAL_DY, 
                    rgb_height - THERMAL_HEIGHT + CUT_PIXELS))
    
    # 크롭 범위 검증
    if y_pos < 0:
        y_pos = 0
    if x_pos < 0:
        x_pos = 0
    if y_pos + thermal_cropped_height > rgb_height:
        y_pos = rgb_height - thermal_cropped_height
    if x_pos + thermal_cropped_width > rgb_width:
        x_pos = rgb_width - thermal_cropped_width
    
    # RGB 이미지 크롭
    frame_rgb_cropped = frame_rgb[y_pos:y_pos+thermal_cropped_height, 
                                   x_pos:x_pos+thermal_cropped_width]
    
    # 크기 검증
    if frame_rgb_cropped.shape[:2] != frame_thermal_cropped.shape[:2]:
        # 크기가 다르면 리사이즈
        frame_rgb_cropped = cv2.resize(
            frame_rgb_cropped, 
            (thermal_cropped_width, thermal_cropped_height),
            interpolation=cv2.INTER_LINEAR
        )
    
    # 오버레이 합성
    overlay_area = cv2.addWeighted(
        frame_rgb_cropped, ALPHA_THERMAL_LAYER, 
        frame_thermal_cropped, 1 - ALPHA_THERMAL_LAYER, 0)
    
    # 마커 오버레이
    mask = overlay_marker.any(axis=2)
    overlay_area = np.where(
        mask[:, :, np.newaxis],
        np.float32(overlay_area) * ALPHA_MARKER_LAYER +
        np.float32(overlay_marker) * (1 - ALPHA_MARKER_LAYER),
        overlay_area).astype(np.uint8)
    
    # 텍스트 추가
    relative_loc = (max_loc[0] - CENTER[0], max_loc[1] - CENTER[1])
    relative_loc_str = "(dx, dy): ({}, {})".format(relative_loc[0], relative_loc[1])
    cv2.putText(overlay_area, relative_loc_str, (11, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(overlay_area, relative_loc_str, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    max_val_str = "Max raw value: {:.1f}".format(max_val)
    cv2.putText(overlay_area, max_val_str, (11, 41),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(overlay_area, max_val_str, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 최종 출력 크기로 리사이즈
    output_frame = cv2.resize(overlay_area, (OUTPUT_WIDTH, OUTPUT_HEIGHT))
    
    return output_frame

def camera_thread():
    """카메라 캡처 및 합성 스레드"""
    global cap_thermal, cap_rgb, is_running
    
    frame_count = 0
    last_time = time.time()
    
    while is_running:
        # 카메라 연결 확인
        if not check_camera_connection(cap_thermal):
            cap_thermal.release()
            cap_thermal = cv2.VideoCapture(THERMAL_CAMERA_ID, cv2.CAP_V4L2)
        if not check_camera_connection(cap_rgb):
            cap_rgb.release()
            cap_rgb = cv2.VideoCapture(RGB_CAMERA_ID, cv2.CAP_V4L2)
            cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        ret1, frame1 = cap_thermal.read()
        ret2, frame2 = cap_rgb.read()
        
        if not ret1 or not ret2:
            time.sleep(0.1)
            continue
        
        # 프레임 합성
        try:
            # 프레임 크기 검증
            if frame1 is None or frame2 is None:
                time.sleep(0.1)
                continue
            
            if len(frame1.shape) < 2 or len(frame2.shape) < 2:
                time.sleep(0.1)
                continue
            
            composite_frame = composite_frames(frame1, frame2)
            
            if composite_frame is None:
                time.sleep(0.1)
                continue
            
            # BGR to RGB 변환 (GStreamer는 RGB 사용)
            composite_frame = cv2.cvtColor(composite_frame, cv2.COLOR_BGR2RGB)
            
            # 큐에 추가
            if not frame_queue.full():
                frame_queue.put(composite_frame)
            else:
                try:
                    frame_queue.get_nowait()
                    frame_queue.put(composite_frame)
                except:
                    pass
            
            frame_count += 1
            current_time = time.time()
            if current_time - last_time >= 5.0:
                fps = frame_count / (current_time - last_time)
                print(f"  → 카메라 FPS: {fps:.1f}")
                frame_count = 0
                last_time = current_time
        except Exception as e:
            print(f"  ⚠ 프레임 합성 오류: {e}")
            time.sleep(0.1)
    
    if cap_thermal:
        cap_thermal.release()
    if cap_rgb:
        cap_rgb.release()

# ============================================================
# 파이프라인 생성 (appsrc 사용)
# ============================================================
def get_launch_string_amlvenc():
    """VIM4 하드웨어 인코더 (appsrc)"""
    return (
        f"appsrc name=src is-live=true format=time "
        f"caps=video/x-raw,format=RGB,width={OUTPUT_WIDTH},height={OUTPUT_HEIGHT},framerate={OUTPUT_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=NV12 ! "
        f"amlvenc bitrate={BITRATE_KBPS} gop=30 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
    )

def get_launch_string_openh264():
    """OpenH264 소프트웨어 인코더 (appsrc)"""
    return (
        f"appsrc name=src is-live=true format=time "
        f"caps=video/x-raw,format=RGB,width={OUTPUT_WIDTH},height={OUTPUT_HEIGHT},framerate={OUTPUT_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"openh264enc bitrate={BITRATE_KBPS * 1000} complexity=0 ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
    )

def get_launch_string_x264():
    """x264 소프트웨어 인코더 (appsrc)"""
    return (
        f"appsrc name=src is-live=true format=time "
        f"caps=video/x-raw,format=RGB,width={OUTPUT_WIDTH},height={OUTPUT_HEIGHT},framerate={OUTPUT_FPS}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"x264enc tune=zerolatency bitrate={BITRATE_KBPS} speed-preset=ultrafast ! "
        f"video/x-h264,profile=baseline ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay name=pay0 pt=96 "
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
# RTSP 미디어 팩토리
# ============================================================
class CompositeRTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    """합성된 프레임을 RTSP로 스트리밍하는 미디어 팩토리"""
    
    def __init__(self, launch_string):
        super().__init__()
        self.launch_string = launch_string
        self.set_shared(True)
        self.appsrc = None
        self.push_thread = None
        
    def do_create_element(self, url):
        """미디어 파이프라인 생성"""
        pipeline = Gst.parse_launch(self.launch_string)
        
        # appsrc 찾기
        self.appsrc = pipeline.get_by_name("src")
        if self.appsrc:
            # appsrc 설정
            self.appsrc.set_property("format", Gst.Format.TIME)
            self.appsrc.set_property("is-live", True)
            self.appsrc.set_property("do-timestamp", True)
            
            # 프레임 푸시 스레드 시작
            if self.push_thread is None or not self.push_thread.is_alive():
                self.push_thread = threading.Thread(target=self.push_frames, daemon=True)
                self.push_thread.start()
        
        return pipeline
    
    def push_frames(self):
        """프레임을 appsrc로 푸시"""
        global frame_queue, is_running
        
        if self.appsrc is None:
            return
        
        frame_duration = Gst.SECOND // OUTPUT_FPS  # 나노초 단위
        timestamp = 0
        
        while is_running:
            try:
                frame = frame_queue.get(timeout=0.1)
                
                if frame is None:
                    continue
                
                # 프레임 검증
                if len(frame.shape) != 3 or frame.shape[2] != 3:
                    continue
                
                # GStreamer 버퍼 생성
                height, width, channels = frame.shape
                size = width * height * channels
                
                if size == 0:
                    continue
                
                # numpy 배열을 bytes로 변환
                frame_bytes = frame.tobytes()
                
                buffer = Gst.Buffer.new_allocate(None, size, None)
                buffer.fill(0, frame_bytes)
                buffer.pts = timestamp
                buffer.dts = timestamp
                buffer.duration = frame_duration
                
                # appsrc로 푸시
                ret = self.appsrc.emit("push-buffer", buffer)
                if ret != Gst.FlowReturn.OK:
                    if ret == Gst.FlowReturn.FLUSHING:
                        break
                    if ret == Gst.FlowReturn.ERROR:
                        print(f"  ⚠ appsrc 푸시 오류: {ret}")
                    time.sleep(0.01)
                
                timestamp += frame_duration
                
            except Exception as e:
                if "timeout" not in str(e).lower():
                    import traceback
                    print(f"  ⚠ 프레임 푸시 오류: {e}")
                    traceback.print_exc()
                time.sleep(0.01)

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
    print("  RGB + 열화상 합성 RTSP 스트리밍 서버")
    print("=" * 60)
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 초기화
    print("\n[초기화]")
    kill_existing_processes()
    
    # 카메라 초기화
    if not init_cameras():
        print(f"  ✗ 카메라 초기화 실패")
        sys.exit(1)
    print(f"  ✓ 카메라 초기화 완료")
    print(f"    출력 해상도: {OUTPUT_WIDTH}x{OUTPUT_HEIGHT} @ {OUTPUT_FPS}fps")
    
    # 카메라 스레드 시작
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()
    time.sleep(1)  # 카메라 초기화 대기
    
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
        
        factory = CompositeRTSPMediaFactory(launch_string)
        
        mount_points = server.get_mount_points()
        mount_points.add_factory(RTSP_MOUNT_POINT, factory)
        
        server.attach(None)
        
    except Exception as e:
        print(f"  ✗ 서버 생성 실패: {e}")
        import traceback
        traceback.print_exc()
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