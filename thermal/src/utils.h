#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

// 로컬 IP 주소 가져오기
std::string get_local_ip();

// 카메라 정보 구조체
struct CameraInfo {
    int id;
    std::string device;
    std::string name;
    std::string vid;
    std::string pid;
    std::string model;
};

// 카메라 정보 가져오기
CameraInfo get_camera_info(const std::string& device_path);

// 키워드로 카메라 찾기
std::vector<CameraInfo> find_cameras_by_keywords(
    const std::vector<std::string>& keywords,
    const std::vector<std::string>& exclude_keywords = {}
);

// 기존 프로세스 종료
void kill_existing_processes();

// 카메라 사용 프로세스 확인 (종료하지 않고 확인만)
void check_camera_processes();

// 인코더 확인 (GStreamer)
bool check_encoder(const std::string& encoder_name);

// USB 카메라 재연결 (unbind/bind)
bool reset_usb_camera(const std::string& device_path);
bool reset_all_usb_cameras();

#endif // UTILS_H
