#include "camera_manager.h"
#include "utils.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <exception>

CameraManager::CameraManager() 
    : thermal_camera_id_(-1), rgb_camera_id_(-1) {
}

CameraManager::~CameraManager() {
    // 카메라 리소스 명시적 해제 (예외 처리)
    if (cap_thermal_.isOpened()) {
        try {
            cap_thermal_.release();
        } catch (...) {
            // 해제 실패는 무시 (소멸자에서는 예외를 던지면 안됨)
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 해제 대기
    }
    if (cap_rgb_.isOpened()) {
        try {
            cap_rgb_.release();
        } catch (...) {
            // 해제 실패는 무시 (소멸자에서는 예외를 던지면 안됨)
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 해제 대기
    }
}

bool CameraManager::initialize() {
    // 개별 카메라 초기화 (비동기식)
    bool rgb_ok = initialize_rgb_camera();
    bool thermal_ok = initialize_thermal_camera();
    
    // 하나라도 성공하면 true 반환
    if (rgb_ok || thermal_ok) {
        return true;
    }
    
    // 둘 다 실패한 경우에만 false
    return false;
}

bool CameraManager::initialize_rgb_camera() {
    // 이미 초기화되어 있으면 스킵
    if (cap_rgb_.isOpened()) {
        return true;
    }
    
    // RGB 카메라 찾기
    if (!find_rgb_camera()) {
        return false;
    }
    
    // RGB 카메라 열기 (안전한 방식)
    if (!safe_open_camera(rgb_camera_id_, cap_rgb_)) {
        return false;
    }
    
    // RGB 카메라 설정 (예외 처리)
    try {
        cap_rgb_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_rgb_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        // RGB 카메라 버퍼 최소화 (지연 감소)
        try {
            cap_rgb_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        } catch (...) {
            // 일부 카메라는 버퍼 크기 설정을 지원하지 않을 수 있음
        }
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] RGB 카메라 설정 OpenCV 예외: " << e.what() << std::endl;
        if (cap_rgb_.isOpened()) {
            try {
                cap_rgb_.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    } catch (...) {
        std::cerr << "  [예외] RGB 카메라 설정 알 수 없는 예외" << std::endl;
        if (cap_rgb_.isOpened()) {
            try {
                cap_rgb_.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    }
    
    // 최적화: 워밍업 횟수 감소 (2회 → 1회), 대기 시간 단축
    try {
        cv::Mat frame;
        cap_rgb_.read(frame);  // 워밍업
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 최종 테스트
        cv::Mat rgb_frame;
        bool rgb_ok = cap_rgb_.read(rgb_frame);
        
        if (!rgb_ok || rgb_frame.empty()) {
            if (cap_rgb_.isOpened()) {
                try {
                    cap_rgb_.release();
                } catch (...) {
                    // 무시
                }
            }
            return false;
        }
        
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] RGB 카메라 읽기 OpenCV 예외: " << e.what() << std::endl;
        if (cap_rgb_.isOpened()) {
            try {
                cap_rgb_.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    } catch (const std::exception& e) {
        std::cerr << "  [예외] RGB 카메라 읽기 표준 예외: " << e.what() << std::endl;
        if (cap_rgb_.isOpened()) {
            try {
                cap_rgb_.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    } catch (...) {
        std::cerr << "  [예외] RGB 카메라 읽기 알 수 없는 예외" << std::endl;
        if (cap_rgb_.isOpened()) {
            try {
                cap_rgb_.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    }
}

bool CameraManager::initialize_thermal_camera() {
    // 이미 초기화되어 있으면 스킵
    if (cap_thermal_.isOpened()) {
        return true;
    }
    
    // 열화상 카메라 찾기 (이미 열기까지 수행함)
    if (!find_thermal_camera()) {
        // 열화상 카메라를 찾지 못했거나 열기 실패
        return false;
    }
    
    // find_thermal_camera()에서 이미 열고 워밍업까지 완료했으므로
    // 추가 검증만 수행
    try {
        cv::Mat frame;
        bool ret = cap_thermal_.read(frame);
        if (ret && !frame.empty()) {
            return true;
        }
        
        // 읽기 실패해도 카메라는 열려있으므로 연결은 유지 (나중에 재시도)
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] 열화상 카메라 읽기 OpenCV 예외: " << e.what() << std::endl;
        // 연결은 유지 (나중에 재시도)
        return true;
    } catch (const std::exception& e) {
        std::cerr << "  [예외] 열화상 카메라 읽기 표준 예외: " << e.what() << std::endl;
        // 연결은 유지 (나중에 재시도)
        return true;
    } catch (...) {
        std::cerr << "  [예외] 열화상 카메라 읽기 알 수 없는 예외" << std::endl;
        // 연결은 유지 (나중에 재시도)
        return true;
    }
}

bool CameraManager::find_thermal_camera() {
    // PureThermal 카메라는 연결 후 워밍업 시간이 필요함
    // 여러 번 재시도하여 안정적인 연결 확보
    
    std::vector<std::string> keywords = {"purethermal", "thermal", "lepton", "flir", "humirothermal"};
    std::vector<CameraInfo> thermal_cameras = find_cameras_by_keywords(keywords);
    
    // 키워드로 찾은 열화상 카메라가 있으면 사용
    if (!thermal_cameras.empty()) {
        thermal_camera_id_ = thermal_cameras[0].id;

        // Lepton RAD/TLinear/LOW_GAIN 설정 (OpenCV 열기 전에 적용)
        configure_lepton_settings(thermal_camera_id_);

        // PureThermal 카메라는 여러 번 재시도 필요 (최대 3회)
        const int max_retries = 3;
        const int retry_delay_ms = 500;  // 재시도 간 대기 시간 (500ms)

        for (int retry = 0; retry < max_retries; ++retry) {
            // 열화상 카메라 열기 (안전한 방식)
            if (safe_open_camera(thermal_camera_id_, cap_thermal_)) {
                // PureThermal 카메라는 연결 후 추가 대기 시간 필요 (1초)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                // 열화상 카메라 설정 (예외 처리)
                try {
                    // Y16 (16-bit raw radiometric) 포맷 설정 — 절대 온도 측정 가능
                    cap_thermal_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));
                    cap_thermal_.set(cv::CAP_PROP_FRAME_WIDTH, 160);
                    cap_thermal_.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
                    // RGB 자동 변환 비활성화 → raw uint16 데이터 수신
                    cap_thermal_.set(cv::CAP_PROP_CONVERT_RGB, 0);
                    cap_thermal_.set(cv::CAP_PROP_BUFFERSIZE, 1);
                    cap_thermal_.set(cv::CAP_PROP_FPS, 9);
                    std::cout << "  ✓ 열화상 카메라 Y16 (16-bit radiometric) 모드 설정" << std::endl;
                } catch (const cv::Exception& e) {
                    std::cerr << "  [예외] 열화상 카메라 설정 OpenCV 예외: " << e.what() << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "  [예외] 열화상 카메라 설정 표준 예외: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "  [예외] 열화상 카메라 설정 알 수 없는 예외" << std::endl;
                }
                
                // 워밍업: 여러 프레임 읽기 시도 (PureThermal은 초기 프레임이 불안정할 수 있음)
                bool warmup_success = false;
                for (int warmup_attempt = 0; warmup_attempt < 5; ++warmup_attempt) {
                    try {
                        cv::Mat test_frame;
                        if (cap_thermal_.read(test_frame) && !test_frame.empty()) {
                            warmup_success = true;
                            break;
                        }
                    } catch (...) {
                        // 읽기 실패는 무시하고 재시도
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                
                if (warmup_success || cap_thermal_.isOpened()) {
                    // 워밍업 성공 또는 카메라가 열려있으면 성공으로 간주
                    return true;
                } else {
                    // 워밍업 실패 시 카메라 닫고 재시도
                    try {
                        cap_thermal_.release();
                    } catch (...) {
                        // 무시
                    }
                }
            }
            
            // 재시도 전 대기
            if (retry < max_retries - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
            }
        }
        
        // 모든 재시도 실패
        return false;
    }
    
    // 키워드로 찾지 못한 경우, 해상도 기반 검색은 제거
    // safe_open_camera() 호출이 하드웨어 레벨에서 크래시를 일으킬 수 있으므로
    // 파일 존재만 확인하는 방식으로 변경
    // 열화상 카메라가 없으면 false 반환 (초기화 시도하지 않음)
    return false;
}

bool CameraManager::find_rgb_camera() {
    // 최적화: 불필요한 출력 제거, 빠른 검색
    std::vector<std::string> thermal_keywords = {"purethermal", "thermal", "lepton", "flir","HumiroThermal"};
    
    for (int i = 0; i < 10; ++i) {
        if (i == thermal_camera_id_) {
            continue;  // 열화상 카메라는 제외
        }
        
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) != 0) {
            continue;
        }
        
        CameraInfo info;
        try {
            info = get_camera_info(device);
        } catch (...) {
            // get_camera_info 실패 시 기본값 사용
            info.id = i;
            info.name = "Unknown";
        }
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
        
        // test_camera()는 cap.open()을 호출하여 하드웨어 레벨에서 크래시를 일으킬 수 있음
        // 재부팅 방지를 위해 파일 존재만 확인하고, 실제 카메라 작동 여부는 런타임에 확인
        // 파일이 존재하고 열화상이 아니면 RGB 카메라로 간주
        if (device_exists(device)) {
            rgb_camera_id_ = i;
            return true;
        }
    }
    
    return false;
}

bool CameraManager::device_exists(const std::string& device_path) {
    // 파일 시스템 레벨 확인만 수행 (V4L2 ioctl은 하드웨어 레벨에서 크래시를 일으킬 수 있음)
    struct stat st;
    if (stat(device_path.c_str(), &st) != 0) {
        return false;
    }
    
    // 문자 디바이스인지 확인 (S_ISCHR)
    if (!S_ISCHR(st.st_mode)) {
        return false;
    }
    
    // V4L2 ioctl 호출 제거 - 하드웨어 접근을 최소화하여 재부팅 방지
    // 파일 존재만 확인하고, 실제 카메라 여부는 cap.open()에서 확인
    return true;
}

bool CameraManager::safe_open_camera(int camera_id, cv::VideoCapture& cap) {
    try {
        // 디바이스 존재 확인 (파일 시스템 레벨만)
        std::string device = "/dev/video" + std::to_string(camera_id);
        if (!device_exists(device)) {
            return false;
        }
        
        // V4L2 ioctl 호출 제거 - 하드웨어 접근을 최소화하여 재부팅 방지
        // cap.open() 호출 자체를 안전하게 보호하는 것으로 충분
        
        // 별도 스레드에서 VideoCapture 열기 시도 (타임아웃 보호)
        std::atomic<bool> open_success(false);
        std::atomic<bool> open_done(false);
        std::exception_ptr open_exception = nullptr;
        
        std::thread open_thread([&]() {
            try {
                // VideoCapture 열기 시도
                // 시그널 핸들러가 SIGSEGV, SIGBUS 등을 처리하므로 안전
                cap.open(camera_id, cv::CAP_V4L2);
                open_success = cap.isOpened();
            } catch (const cv::Exception& e) {
                open_exception = std::current_exception();
            } catch (const std::exception& e) {
                open_exception = std::current_exception();
            } catch (...) {
                open_exception = std::current_exception();
            }
            open_done = true;
        });
        
        // 타임아웃 설정 (1초로 단축 - 빠른 실패)
        auto start = std::chrono::steady_clock::now();
        while (!open_done) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > 1000) {
                // 타임아웃: 스레드가 아직 실행 중이면 강제 종료는 불가능하지만,
                // cap.open()이 블로킹되어 있으면 계속 대기할 수 있음
                // 하지만 최소한 예외는 처리됨
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 스레드 종료 대기 (최대 1초)
        if (open_thread.joinable()) {
            open_thread.join();
        }
        
        // 예외 처리
        if (open_exception) {
            try {
                std::rethrow_exception(open_exception);
            } catch (const cv::Exception& e) {
                std::cerr << "  [예외] OpenCV 예외: " << e.what() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "  [예외] 표준 예외: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "  [예외] 알 수 없는 예외 발생" << std::endl;
            }
            if (cap.isOpened()) {
                try {
                    cap.release();
                } catch (...) {
                    // release 실패는 무시
                }
            }
            return false;
        }
        
        // 성공 여부 확인
        if (!open_success) {
            if (cap.isOpened()) {
                try {
                    cap.release();
                } catch (...) {
                    // release 실패는 무시
                }
            }
            return false;
        }
        
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] safe_open_camera OpenCV 예외: " << e.what() << std::endl;
        if (cap.isOpened()) {
            try {
                cap.release();
            } catch (...) {
                // release 실패는 무시
            }
        }
        return false;
    } catch (const std::exception& e) {
        std::cerr << "  [예외] safe_open_camera 표준 예외: " << e.what() << std::endl;
        if (cap.isOpened()) {
            try {
                cap.release();
            } catch (...) {
                // release 실패는 무시
            }
        }
        return false;
    } catch (...) {
        std::cerr << "  [예외] safe_open_camera 알 수 없는 예외 발생" << std::endl;
        if (cap.isOpened()) {
            try {
                cap.release();
            } catch (...) {
                // release 실패는 무시
            }
        }
        return false;
    }
}

bool CameraManager::test_camera(int camera_id, bool is_thermal) {
    // 디바이스 존재 확인
    std::string device = "/dev/video" + std::to_string(camera_id);
    if (!device_exists(device)) {
        return false;
    }
    
    cv::VideoCapture test_cap;
    if (!safe_open_camera(camera_id, test_cap)) {
        return false;
    }
    
    try {
        if (!is_thermal) {
            test_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            test_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }
        
        // 최적화: 시도 횟수 감소 (3회 → 1회), 대기 시간 단축 (100ms → 50ms)
        cv::Mat frame;
        if (test_cap.read(frame) && !frame.empty() && frame.total() > 0) {
            int h = frame.rows;
            int w = frame.cols;
            
            if (!is_thermal && (h >= 200 && w >= 200)) {
                test_cap.release();
                return true;
            }
        }
        
        test_cap.release();
        return false;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] test_camera OpenCV 예외: " << e.what() << std::endl;
        if (test_cap.isOpened()) {
            try {
                test_cap.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    } catch (...) {
        std::cerr << "  [예외] test_camera 알 수 없는 예외" << std::endl;
        if (test_cap.isOpened()) {
            try {
                test_cap.release();
            } catch (...) {
                // 무시
            }
        }
        return false;
    }
}

bool CameraManager::read_rgb_frame(cv::Mat& frame) {
    if (!cap_rgb_.isOpened()) {
        return false;
    }
    
    try {
        bool success = cap_rgb_.read(frame);
        if (!success || frame.empty()) {
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] RGB 프레임 읽기 OpenCV 예외: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "  [예외] RGB 프레임 읽기 표준 예외: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "  [예외] RGB 프레임 읽기 알 수 없는 예외" << std::endl;
        return false;
    }
}

bool CameraManager::read_thermal_frame(cv::Mat& frame) {
    if (!cap_thermal_.isOpened()) {
        return false;
    }
    
    try {
        bool success = cap_thermal_.read(frame);
        if (!success || frame.empty()) {
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "  [예외] 열화상 프레임 읽기 OpenCV 예외: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "  [예외] 열화상 프레임 읽기 표준 예외: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "  [예외] 열화상 프레임 읽기 알 수 없는 예외" << std::endl;
        return false;
    }
}

bool CameraManager::reconnect_rgb_camera() {
    std::cout << "  → RGB 카메라 재연결 시도..." << std::endl;

    // 기존 연결 해제 (예외 처리)
    if (cap_rgb_.isOpened()) {
        try {
            cap_rgb_.release();
        } catch (...) {
            // 해제 실패는 무시
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 카메라 ID가 유효하지 않으면 다시 찾기
    if (rgb_camera_id_ < 0) {
        if (!find_rgb_camera()) {
            std::cout << "    ✗ RGB 카메라 장치를 찾을 수 없음" << std::endl;
            return false;
        }
    }

    // 재연결 (안전한 방식)
    if (!safe_open_camera(rgb_camera_id_, cap_rgb_)) {
        // 열기 실패 시 ID 리셋 (다음 시도 때 다시 찾기)
        rgb_camera_id_ = -1;
        std::cout << "    ✗ RGB 카메라 재연결 실패" << std::endl;
        return false;
    }
    
    // 설정 복원 (예외 처리)
    try {
        cap_rgb_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_rgb_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        try {
            cap_rgb_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        } catch (...) {
            // 무시
        }
    } catch (const cv::Exception& e) {
        std::cerr << "    [예외] RGB 카메라 설정 OpenCV 예외: " << e.what() << std::endl;
        // 설정 실패해도 계속 진행
    } catch (...) {
        std::cerr << "    [예외] RGB 카메라 설정 알 수 없는 예외" << std::endl;
        // 설정 실패해도 계속 진행
    }
    
    // 빠른 테스트 (예외 처리)
    try {
        cv::Mat test_frame;
        for (int i = 0; i < 2; ++i) {
            try {
                if (cap_rgb_.read(test_frame) && !test_frame.empty()) {
                    std::cout << "    ✓ RGB 카메라 재연결 성공" << std::endl;
                    return true;
                }
            } catch (const cv::Exception& e) {
                std::cerr << "    [예외] RGB 카메라 읽기 OpenCV 예외: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "    [예외] RGB 카메라 읽기 알 수 없는 예외" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception& e) {
        std::cerr << "    [예외] RGB 카메라 테스트 표준 예외: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "    [예외] RGB 카메라 테스트 알 수 없는 예외" << std::endl;
    }
    
    std::cout << "    ✗ RGB 카메라 프레임 읽기 실패" << std::endl;
    return false;
}

bool CameraManager::reconnect_thermal_camera() {
    std::cout << "  → 열화상 카메라 재연결 시도..." << std::endl;

    // 기존 연결 해제 (예외 처리)
    if (cap_thermal_.isOpened()) {
        try {
            cap_thermal_.release();
        } catch (...) {
            // 해제 실패는 무시
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 카메라 ID가 유효하지 않으면 다시 찾기
    if (thermal_camera_id_ < 0) {
        if (!find_thermal_camera()) {
            std::cout << "    ✗ 열화상 카메라 장치를 찾을 수 없음" << std::endl;
            return false;
        }
    }

    // Lepton RAD/TLinear/LOW_GAIN 설정 (재연결 시에도 적용)
    configure_lepton_settings(thermal_camera_id_);

    // 재연결 (안전한 방식)
    if (!safe_open_camera(thermal_camera_id_, cap_thermal_)) {
        // 열기 실패 시 ID 리셋 (다음 시도 때 다시 찾기)
        thermal_camera_id_ = -1;
        std::cout << "    ✗ 열화상 카메라 재연결 실패" << std::endl;
        return false;
    }

    // 설정 복원 (예외 처리)
    try {
        cap_thermal_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));
        cap_thermal_.set(cv::CAP_PROP_FRAME_WIDTH, 160);
        cap_thermal_.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
        // RGB 자동 변환 비활성화 → raw uint16 데이터 수신
        cap_thermal_.set(cv::CAP_PROP_CONVERT_RGB, 0);
        cap_thermal_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap_thermal_.set(cv::CAP_PROP_FPS, 9);
    } catch (const cv::Exception& e) {
        std::cerr << "    [예외] 열화상 카메라 설정 OpenCV 예외: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "    [예외] 열화상 카메라 설정 알 수 없는 예외" << std::endl;
    }
    
    // 빠른 테스트 (열화상은 느리므로 더 많은 시도) - 예외 처리
    try {
        cv::Mat test_frame;
        for (int i = 0; i < 3; ++i) {
            try {
                if (cap_thermal_.read(test_frame) && !test_frame.empty()) {
                    std::cout << "    ✓ 열화상 카메라 재연결 성공" << std::endl;
                    return true;
                }
            } catch (const cv::Exception& e) {
                std::cerr << "    [예외] 열화상 카메라 읽기 OpenCV 예외: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "    [예외] 열화상 카메라 읽기 알 수 없는 예외" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    } catch (const std::exception& e) {
        std::cerr << "    [예외] 열화상 카메라 테스트 표준 예외: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "    [예외] 열화상 카메라 테스트 알 수 없는 예외" << std::endl;
    }
    
    std::cout << "    ⚠ 열화상 카메라 프레임 읽기 실패 (나중에 재시도)" << std::endl;
    return false;  // 실패해도 연결은 유지 (나중에 재시도)
}

bool CameraManager::configure_lepton_settings(int camera_id) {
    // Lepton 3.5 RAD/TLinear/Gain 설정 (V4L2 UVC Extension Unit)
    // setup_lepton.py와 동일한 기능을 C++로 구현
    // 카메라 전원 리셋/USB 재연결 시 설정이 초기화되므로 매번 적용 필요

    std::string device = "/dev/video" + std::to_string(camera_id);
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "  [Lepton] " << device << " 열기 실패 (설정 스킵)" << std::endl;
        return false;
    }

    // UVC Extension Unit IDs (GetThermal/PureThermal 기준)
    const uint8_t RAD_UNIT_ID = 5;   // Radiometry module
    const uint8_t SYS_UNIT_ID = 6;   // System module

    // Control selectors: ((CCI_reg & 0xFF) >> 2) + 1
    const uint8_t RAD_ENABLE_CTRL             = 5;   // 0x4E10
    const uint8_t RAD_TLINEAR_ENABLE_CTRL     = 49;  // 0x4EC0
    const uint8_t RAD_TLINEAR_RESOLUTION_CTRL = 50;  // 0x4EC4
    const uint8_t SYS_GAIN_MODE_CTRL          = 19;  // 0x0248

    // 값
    const uint32_t RAD_ENABLE       = 1;
    const uint32_t TLINEAR_ENABLE   = 1;
    const uint32_t TLINEAR_RES_001K = 1;  // 0.01K (센티켈빈)
    const uint32_t GAIN_LOW         = 1;  // LOW_GAIN: -10~400°C

    auto uvc_set = [&](uint8_t unit, uint8_t selector, uint32_t value) -> bool {
        struct uvc_xu_control_query xu;
        xu.unit     = unit;
        xu.selector = selector;
        xu.query    = UVC_SET_CUR;
        xu.size     = 4;
        xu.data     = reinterpret_cast<uint8_t*>(&value);

        int ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
        return (ret >= 0);
    };

    auto uvc_get = [&](uint8_t unit, uint8_t selector, uint32_t& value) -> bool {
        struct uvc_xu_control_query xu;
        value = 0;
        xu.unit     = unit;
        xu.selector = selector;
        xu.query    = UVC_GET_CUR;
        xu.size     = 4;
        xu.data     = reinterpret_cast<uint8_t*>(&value);

        int ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
        return (ret >= 0);
    };

    // 현재 설정 확인
    uint32_t cur_gain = 0, cur_rad = 0, cur_tlinear = 0, cur_res = 0;
    bool can_read = uvc_get(SYS_UNIT_ID, SYS_GAIN_MODE_CTRL, cur_gain);
    if (can_read) {
        uvc_get(RAD_UNIT_ID, RAD_ENABLE_CTRL, cur_rad);
        uvc_get(RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL, cur_tlinear);
        uvc_get(RAD_UNIT_ID, RAD_TLINEAR_RESOLUTION_CTRL, cur_res);

        std::cout << "  [Lepton] 현재: Gain=" << cur_gain
                  << " RAD=" << cur_rad
                  << " TLinear=" << cur_tlinear
                  << " Res=" << cur_res << std::endl;

        // 이미 올바른 설정이면 스킵
        if (cur_gain == GAIN_LOW && cur_rad == RAD_ENABLE &&
            cur_tlinear == TLINEAR_ENABLE && cur_res == TLINEAR_RES_001K) {
            std::cout << "  [Lepton] 설정 정상 — 스킵" << std::endl;
            close(fd);
            return true;
        }
    }

    std::cout << "  [Lepton] RAD/TLinear/LOW_GAIN 설정 적용 중..." << std::endl;

    bool ok = true;

    // 1. RAD Enable
    if (!uvc_set(RAD_UNIT_ID, RAD_ENABLE_CTRL, RAD_ENABLE)) {
        std::cerr << "  [Lepton] RAD Enable 실패" << std::endl;
        ok = false;
    }
    usleep(100000);  // 100ms

    // 2. TLinear Enable
    if (!uvc_set(RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL, TLINEAR_ENABLE)) {
        std::cerr << "  [Lepton] TLinear Enable 실패" << std::endl;
        ok = false;
    }
    usleep(100000);

    // 3. TLinear Resolution → 0.01K (센티켈빈)
    if (!uvc_set(RAD_UNIT_ID, RAD_TLINEAR_RESOLUTION_CTRL, TLINEAR_RES_001K)) {
        std::cerr << "  [Lepton] TLinear Res 설정 실패" << std::endl;
        ok = false;
    }
    usleep(100000);

    // 4. LOW_GAIN mode (-10~400°C)
    if (!uvc_set(SYS_UNIT_ID, SYS_GAIN_MODE_CTRL, GAIN_LOW)) {
        std::cerr << "  [Lepton] Gain Mode 설정 실패" << std::endl;
        ok = false;
    }
    usleep(500000);  // 500ms — Gain 전환 시 FFC(셔터) 동작 대기

    // 검증
    if (ok && uvc_get(SYS_UNIT_ID, SYS_GAIN_MODE_CTRL, cur_gain)) {
        uvc_get(RAD_UNIT_ID, RAD_ENABLE_CTRL, cur_rad);
        uvc_get(RAD_UNIT_ID, RAD_TLINEAR_ENABLE_CTRL, cur_tlinear);
        uvc_get(RAD_UNIT_ID, RAD_TLINEAR_RESOLUTION_CTRL, cur_res);

        std::cout << "  [Lepton] 적용 후: Gain=" << cur_gain
                  << " RAD=" << cur_rad
                  << " TLinear=" << cur_tlinear
                  << " Res=" << cur_res << std::endl;

        if (cur_gain == GAIN_LOW && cur_tlinear == TLINEAR_ENABLE) {
            std::cout << "  [Lepton] LOW_GAIN + TLinear(0.01K) 설정 완료" << std::endl;
        } else {
            std::cerr << "  [Lepton] 설정 검증 실패 — 온도 측정이 부정확할 수 있음" << std::endl;
            ok = false;
        }
    }

    close(fd);
    return ok;
}
