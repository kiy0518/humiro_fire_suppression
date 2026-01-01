#include "camera_manager.h"
#include "utils.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <sys/stat.h>

CameraManager::CameraManager() 
    : thermal_camera_id_(-1), rgb_camera_id_(-1) {
}

CameraManager::~CameraManager() {
    // 카메라 리소스 명시적 해제
    if (cap_thermal_.isOpened()) {
        cap_thermal_.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 해제 대기
    }
    if (cap_rgb_.isOpened()) {
        cap_rgb_.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 해제 대기
    }
}

bool CameraManager::initialize() {
    std::cout << "  → 카메라 정보 수집..." << std::endl;
    
    // 모든 카메라 정보 수집
    std::vector<CameraInfo> all_cameras;
    for (int i = 0; i < 10; ++i) {
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) == 0) {
            CameraInfo info = get_camera_info(device);
            all_cameras.push_back(info);
            std::cout << "    → " << device << ": " << info.name << std::endl;
        }
    }
    
    // 열화상 카메라 찾기
    if (!find_thermal_camera()) {
        std::cout << "    ✗ 열화상 카메라를 찾을 수 없습니다" << std::endl;
        return false;
    }
    
    // RGB 카메라 찾기
    if (!find_rgb_camera()) {
        std::cout << "    ✗ RGB 카메라를 찾을 수 없습니다" << std::endl;
        if (cap_thermal_.isOpened()) {
            cap_thermal_.release();
        }
        return false;
    }
    
    // RGB 카메라 초기화
    cap_rgb_.open(rgb_camera_id_, cv::CAP_V4L2);
    if (!cap_rgb_.isOpened()) {
        std::cout << "    ✗ RGB 카메라 열기 실패" << std::endl;
        if (cap_thermal_.isOpened()) {
            cap_thermal_.release();
        }
        return false;
    }
    
    cap_rgb_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_rgb_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // RGB 카메라 버퍼 최소화 (지연 감소)
    try {
        cap_rgb_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    } catch (...) {
        // 일부 카메라는 버퍼 크기 설정을 지원하지 않을 수 있음
    }
    
    // 카메라 워밍업
    std::cout << "  → 카메라 워밍업..." << std::endl;
    std::cout << "    → RGB 카메라 워밍업..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        cv::Mat frame;
        cap_rgb_.read(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "    → 열화상 카메라 워밍업 (느린 응답 대비)..." << std::endl;
    std::cout << "      → Lepton 3.5는 8-9 FPS로 느려서 타임아웃 경고가 나타날 수 있습니다 (정상 동작)" << std::endl;
    for (int i = 0; i < 5; ++i) {
        cv::Mat frame;
        bool ret = cap_thermal_.read(frame);
        if (ret) {
            std::cout << "      → 프레임 " << (i+1) << "/5 읽기 성공" << std::endl;
        } else {
            std::cout << "      → 프레임 " << (i+1) << "/5 읽기 실패 (재시도 중...) - 정상 (카메라가 느림)" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    
    // 최종 테스트
    std::cout << "    → 최종 테스트..." << std::endl;
    cv::Mat rgb_frame, thermal_frame;
    bool rgb_ok = cap_rgb_.read(rgb_frame);
    bool thermal_ok = false;
    for (int i = 0; i < 3; ++i) {
        thermal_ok = cap_thermal_.read(thermal_frame);
        if (thermal_ok && !thermal_frame.empty()) {
            break;
        }
        std::cout << "      → 열화상 읽기 시도 " << (i+1) << "/3... (타임아웃 경고는 정상)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    if (!rgb_ok || rgb_frame.empty()) {
        std::cout << "    ✗ RGB 프레임 읽기 실패" << std::endl;
        return false;
    }
    
    std::cout << "    ✓ RGB: " << rgb_frame.cols << "x" << rgb_frame.rows << std::endl;
    
    if (!thermal_ok || thermal_frame.empty()) {
        std::cout << "    ⚠ 열화상 프레임 읽기 실패 (나중에 재시도됨)" << std::endl;
        std::cout << "    → RGB만 먼저 시작, 열화상은 백그라운드에서 재연결 시도" << std::endl;
    } else {
        std::cout << "    ✓ 열화상: " << thermal_frame.cols << "x" << thermal_frame.rows << std::endl;
    }
    
    return true;
}

bool CameraManager::find_thermal_camera() {
    std::cout << "\n  → 열화상 카메라 찾기 (PureThermal)..." << std::endl;
    
    // 먼저 모든 카메라 디바이스 정보 출력 (디버깅)
    std::cout << "    → 사용 가능한 카메라 디바이스 확인..." << std::endl;
    for (int i = 0; i < 20; ++i) {
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) == 0) {
            CameraInfo info = get_camera_info(device);
            std::cout << "      - " << device << ": " << info.name << std::endl;
        }
    }
    
    std::vector<std::string> keywords = {"purethermal", "thermal", "lepton", "flir"};
    std::vector<CameraInfo> thermal_cameras = find_cameras_by_keywords(keywords);
    
    if (thermal_cameras.empty()) {
        std::cout << "    ⚠ 이름으로 열화상 카메라를 찾을 수 없습니다" << std::endl;
        std::cout << "    → 해상도 기반으로 시도..." << std::endl;
        
        // 해상도 기반으로 찾기 (검색 범위 확대: 0-20)
        for (int i = 0; i < 20; ++i) {
            std::string device = "/dev/video" + std::to_string(i);
            struct stat st;
            if (stat(device.c_str(), &st) != 0) {
                continue;
            }
            
            std::cout << "      → " << device << " 테스트 중..." << std::endl;
            cv::VideoCapture test_cap(i, cv::CAP_V4L2);
            if (test_cap.isOpened()) {
                // 타임아웃 설정 (열화상 카메라는 느릴 수 있음)
                test_cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
                cv::Mat frame;
                
                // 여러 번 시도 (열화상 카메라는 첫 프레임이 느릴 수 있음)
                bool frame_read = false;
                for (int retry = 0; retry < 5; ++retry) {
                    if (test_cap.read(frame) && !frame.empty()) {
                        frame_read = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                
                if (frame_read) {
                    int h = frame.rows;
                    int w = frame.cols;
                    CameraInfo info = get_camera_info(device);
                    std::cout << "        해상도: " << w << "x" << h << ", 이름: " << info.name << std::endl;
                    
                    // 열화상 카메라는 보통 작은 해상도 (80x60, 160x120 등)
                    if (h <= 200 && w <= 200) {
                        thermal_cameras.push_back(info);
                        std::cout << "        ✓ 열화상 후보로 선택: " << device 
                                  << " (" << info.name << ", " << w << "x" << h << ")" << std::endl;
                    }
                } else {
                    std::cout << "        ⚠ 프레임 읽기 실패" << std::endl;
                }
                test_cap.release();
            } else {
                std::cout << "        ⚠ 열기 실패" << std::endl;
            }
        }
    }
    
    if (thermal_cameras.empty()) {
        std::cout << "    ✗ 열화상 카메라를 찾을 수 없습니다" << std::endl;
        std::cout << "    → 확인 사항:" << std::endl;
        std::cout << "      1. 열화상 카메라가 연결되어 있는지 확인" << std::endl;
        std::cout << "      2. 카메라가 다른 프로세스에서 사용 중인지 확인" << std::endl;
        std::cout << "      3. dmesg 또는 lsusb로 카메라 인식 여부 확인" << std::endl;
        return false;
    }
    
    thermal_camera_id_ = thermal_cameras[0].id;
    std::cout << "    ✓ 열화상 카메라: " << thermal_cameras[0].device 
              << " (" << thermal_cameras[0].name << ")" << std::endl;
    
    // 열화상 카메라 초기화
    // 다른 프로세스가 사용 중인지 확인
    std::string device_path = thermal_cameras[0].device;
    std::string check_cmd = "lsof " + device_path + " 2>/dev/null";
    FILE* pipe = popen(check_cmd.c_str(), "r");
    if (pipe) {
        char buffer[256];
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::cout << "    ⚠ 경고: " << device_path << "가 다른 프로세스에서 사용 중일 수 있습니다" << std::endl;
        }
        pclose(pipe);
    }
    
    cap_thermal_.open(thermal_camera_id_, cv::CAP_V4L2);
    if (!cap_thermal_.isOpened()) {
        std::cout << "    ✗ 열화상 카메라 열기 실패" << std::endl;
        std::cout << "    → 가능한 원인:" << std::endl;
        std::cout << "      1. 다른 프로세스가 카메라를 사용 중" << std::endl;
        std::cout << "      2. 카메라 디바이스 권한 문제 (sudo 필요할 수 있음)" << std::endl;
        std::cout << "      3. 카메라 하드웨어 문제" << std::endl;
        std::cout << "    → 확인 명령어: lsof " << device_path << std::endl;
        return false;
    }
    
    // 열화상 카메라 설정
    cap_thermal_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap_thermal_.set(cv::CAP_PROP_FPS, 9);
    
    return true;
}

bool CameraManager::find_rgb_camera() {
    std::cout << "\n  → RGB 카메라 찾기 (USB Camera)..." << std::endl;
    
    std::vector<std::string> thermal_keywords = {"purethermal", "thermal", "lepton", "flir"};
    
    for (int i = 0; i < 10; ++i) {
        if (i == thermal_camera_id_) {
            continue;  // 열화상 카메라는 제외
        }
        
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) != 0) {
            continue;
        }
        
        CameraInfo info = get_camera_info(device);
        std::string name_lower = info.name;
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        
        // 열화상 키워드 확인
        bool is_thermal = false;
        for (const auto& kw : thermal_keywords) {
            std::string kw_lower = kw;
            std::transform(kw_lower.begin(), kw_lower.end(), kw_lower.begin(), ::tolower);
            if (name_lower.find(kw_lower) != std::string::npos) {
                is_thermal = true;
                break;
            }
        }
        if (is_thermal) {
            continue;
        }
        
        std::cout << "    → " << device << " 시도 중... (" << info.name << " - RGB 후보)" << std::endl;
        
        if (test_camera(i, false)) {
            rgb_camera_id_ = i;
            std::cout << "    ✓ RGB 카메라 발견: " << device << " (" << info.name << ")" << std::endl;
            return true;
        }
    }
    
    std::cout << "    ✗ RGB 카메라를 찾을 수 없습니다" << std::endl;
    return false;
}

bool CameraManager::test_camera(int camera_id, bool is_thermal) {
    cv::VideoCapture test_cap(camera_id, cv::CAP_V4L2);
    if (!test_cap.isOpened()) {
        return false;
    }
    
    if (!is_thermal) {
        test_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        test_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }
    
    cv::Mat frame;
    for (int i = 0; i < 3; ++i) {
        if (test_cap.read(frame) && !frame.empty() && frame.total() > 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (frame.empty() || frame.total() == 0) {
        test_cap.release();
        return false;
    }
    
    int h = frame.rows;
    int w = frame.cols;
    
    if (!is_thermal && (h >= 200 && w >= 200)) {
        test_cap.release();
        return true;
    }
    
    test_cap.release();
    return false;
}

bool CameraManager::read_rgb_frame(cv::Mat& frame) {
    if (!cap_rgb_.isOpened()) {
        return false;
    }
    return cap_rgb_.read(frame) && !frame.empty();
}

bool CameraManager::read_thermal_frame(cv::Mat& frame) {
    if (!cap_thermal_.isOpened()) {
        return false;
    }
    return cap_thermal_.read(frame) && !frame.empty();
}
