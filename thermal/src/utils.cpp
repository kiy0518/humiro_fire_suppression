#include "utils.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <gst/gst.h>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <thread>
#include <chrono>

std::string get_local_ip() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return "127.0.0.1";
    }
    
    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("8.8.8.8");
    server.sin_port = htons(80);
    
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        close(sock);
        return "127.0.0.1";
    }
    
    struct sockaddr_in local;
    socklen_t len = sizeof(local);
    getsockname(sock, (struct sockaddr*)&local, &len);
    
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &local.sin_addr, ip, INET_ADDRSTRLEN);
    
    close(sock);
    return std::string(ip);
}

CameraInfo get_camera_info(const std::string& device_path) {
    CameraInfo info;
    info.device = device_path;
    
    // /sys/class/video4linux에서 이름 가져오기
    std::string dev_name = device_path.substr(device_path.find_last_of('/') + 1);
    std::string sys_path = "/sys/class/video4linux/" + dev_name + "/name";
    
    std::ifstream name_file(sys_path);
    if (name_file.is_open()) {
        std::getline(name_file, info.name);
        name_file.close();
    }
    
    // udevadm으로 VID/PID 가져오기
    std::string cmd = "udevadm info --name " + device_path + " 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            if (line.find("ID_VENDOR_ID=") != std::string::npos) {
                info.vid = line.substr(line.find('=') + 1);
                // 개행 문자 제거
                info.vid.erase(std::remove(info.vid.begin(), info.vid.end(), '\n'), info.vid.end());
            } else if (line.find("ID_MODEL_ID=") != std::string::npos) {
                info.pid = line.substr(line.find('=') + 1);
                info.pid.erase(std::remove(info.pid.begin(), info.pid.end(), '\n'), info.pid.end());
            } else if (line.find("ID_MODEL=") != std::string::npos) {
                info.model = line.substr(line.find('=') + 1);
                info.model.erase(std::remove(info.model.begin(), info.model.end(), '\n'), info.model.end());
            }
        }
        pclose(pipe);
    }
    
    // ID 추출
    std::string::size_type pos = device_path.find_last_of('/');
    if (pos != std::string::npos) {
        std::string id_str = device_path.substr(pos + 1);
        if (id_str.find("video") == 0) {
            id_str = id_str.substr(5);
            try {
                info.id = std::stoi(id_str);
            } catch (...) {
                info.id = -1;
            }
        }
    }
    
    return info;
}

std::vector<CameraInfo> find_cameras_by_keywords(
    const std::vector<std::string>& keywords,
    const std::vector<std::string>& exclude_keywords) {
    
    std::vector<CameraInfo> cameras;
    
    for (int i = 0; i < 10; ++i) {
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
        
        // 제외 키워드 확인
        bool excluded = false;
        for (const auto& kw : exclude_keywords) {
            std::string kw_lower = kw;
            std::transform(kw_lower.begin(), kw_lower.end(), kw_lower.begin(), ::tolower);
            if (name_lower.find(kw_lower) != std::string::npos) {
                excluded = true;
                break;
            }
        }
        if (excluded) {
            continue;
        }
        
        // 키워드 매칭 확인
        bool matched = false;
        if (keywords.empty()) {
            matched = !info.name.empty();
        } else {
            for (const auto& kw : keywords) {
                std::string kw_lower = kw;
                std::transform(kw_lower.begin(), kw_lower.end(), kw_lower.begin(), ::tolower);
                if (name_lower.find(kw_lower) != std::string::npos) {
                    matched = true;
                    break;
                }
            }
        }
        
        if (matched) {
            cameras.push_back(info);
        }
    }
    
    return cameras;
}

void kill_existing_processes() {
    pid_t my_pid = getpid();
    std::cout << "  → 기존 프로세스 정리... (현재 PID: " << my_pid << ")" << std::endl;
    
    // thermal_rgb_streaming 프로세스 종료 (자기 자신 제외)
    std::string cmd = "ps aux | grep thermal_rgb_streaming | grep -v grep";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            std::istringstream iss(line);
            std::string token;
            int col = 0;
            pid_t pid = 0;
            while (iss >> token && col < 2) {
                if (col == 1) {
                    try {
                        pid = std::stoi(token);
                        if (pid != my_pid && pid > 0) {
                            std::cout << "    → 기존 thermal_rgb_streaming 프로세스 종료: PID " << pid << std::endl;
                            kill(pid, SIGTERM);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            // 여전히 실행 중이면 강제 종료
                            if (kill(pid, 0) == 0) {
                                kill(pid, SIGKILL);
                            }
                        }
                    } catch (...) {
                    }
                    break;
                }
                col++;
            }
        }
        pclose(pipe);
    }
    
    // 카메라를 사용 중인 프로세스 확인 및 종료
    std::cout << "  → 카메라 사용 프로세스 확인..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) != 0) {
            continue;
        }
        
        std::string cmd = "lsof " + device + " 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            bool first_line = true;
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                if (first_line) {
                    first_line = false;
                    continue;  // 헤더 스킵
                }
                
                std::string line(buffer);
                std::istringstream iss(line);
                std::string token;
                int col = 0;
                pid_t pid = 0;
                while (iss >> token && col < 2) {
                    if (col == 1) {
                        try {
                            pid = std::stoi(token);
                            if (pid != my_pid && pid > 0) {
                                std::cout << "    → " << device << " 사용 중인 프로세스 종료: PID " << pid << std::endl;
                                kill(pid, SIGKILL);
                            }
                        } catch (...) {
                        }
                        break;
                    }
                    col++;
                }
            }
            pclose(pipe);
        }
    }
    
    // 일반 프로세스 종료
    const char* processes[] = {"gst-launch-1.0", "gst-launch"};
    for (const char* proc : processes) {
        std::string cmd = "ps aux | grep " + std::string(proc) + " | grep -v grep";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string line(buffer);
                std::istringstream iss(line);
                std::string token;
                int col = 0;
                pid_t pid = 0;
                while (iss >> token && col < 2) {
                    if (col == 1) {
                        try {
                            pid = std::stoi(token);
                            if (pid != my_pid && pid > 0) {
                                std::cout << "    → 프로세스 종료: " << proc << " (PID " << pid << ")" << std::endl;
                                kill(pid, SIGKILL);
                            }
                        } catch (...) {
                        }
                        break;
                    }
                    col++;
                }
            }
            pclose(pipe);
        }
    }
    
    sleep(1);
}

void check_camera_processes() {
    std::cout << "\n[카메라 프로세스 확인]" << std::endl;
    pid_t my_pid = getpid();
    bool found_any = false;
    
    // 카메라를 사용 중인 프로세스 확인
    std::cout << "  → 카메라 디바이스 사용 프로세스:" << std::endl;
    for (int i = 0; i < 10; ++i) {
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) != 0) {
            continue;
        }
        
        std::string cmd = "lsof " + device + " 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            bool first_line = true;
            bool found_device = false;
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                if (first_line) {
                    first_line = false;
                    continue;  // 헤더 스킵
                }
                
                std::string line(buffer);
                // lsof 출력 파싱: COMMAND PID USER FD TYPE DEVICE SIZE/OFF NODE NAME
                std::istringstream iss(line);
                std::vector<std::string> tokens;
                std::string token;
                while (iss >> token) {
                    tokens.push_back(token);
                }
                
                if (tokens.size() >= 2) {
                    try {
                        pid_t pid = std::stoi(tokens[1]);
                        // 자기 자신은 제외
                        if (pid == my_pid) {
                            continue;
                        }
                        
                        if (!found_device) {
                            std::cout << "    " << device << ":" << std::endl;
                            found_device = true;
                            found_any = true;
                        }
                        
                        std::string command = tokens[0];
                        std::string user = tokens.size() > 2 ? tokens[2] : "unknown";
                        std::cout << "      - PID: " << pid 
                                  << ", 사용자: " << user 
                                  << ", 프로세스: " << command << std::endl;
                    } catch (...) {
                        // PID 파싱 실패 시 무시
                    }
                }
            }
            pclose(pipe);
        }
    }
    if (!found_any) {
        std::cout << "    (카메라를 사용 중인 다른 프로세스 없음)" << std::endl;
    }
    
    // thermal_rgb_streaming 프로세스 확인 (자기 자신 제외)
    std::cout << "  → thermal_rgb_streaming 프로세스:" << std::endl;
    std::string cmd = "ps aux | grep thermal_rgb_streaming | grep -v grep";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
        char buffer[256];
        bool found_proc = false;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            std::istringstream iss(line);
            std::vector<std::string> tokens;
            std::string token;
            while (iss >> token) {
                tokens.push_back(token);
            }
            if (tokens.size() >= 11) {
                try {
                    pid_t pid = std::stoi(tokens[1]);
                    // 자기 자신은 제외
                    if (pid == my_pid) {
                        continue;
                    }
                    
                    if (!found_proc) {
                        found_proc = true;
                        found_any = true;
                    }
                    std::cout << "      - PID: " << pid 
                              << ", 사용자: " << tokens[0] 
                              << ", CPU: " << tokens[2] << "%"
                              << ", 메모리: " << tokens[3] << "%" << std::endl;
                } catch (...) {
                    // PID 파싱 실패 시 무시
                }
            }
        }
        pclose(pipe);
        if (!found_proc) {
            std::cout << "      (다른 thermal_rgb_streaming 프로세스 없음 - 현재 PID: " << my_pid << ")" << std::endl;
        }
    }
    
    // GStreamer 프로세스 확인
    std::cout << "  → GStreamer 프로세스:" << std::endl;
    const char* processes[] = {"gst-launch-1.0", "gst-launch"};
    bool found_gst = false;
    for (const char* proc : processes) {
        std::string cmd = "ps aux | grep " + std::string(proc) + " | grep -v grep";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                if (!found_gst) {
                    found_gst = true;
                    found_any = true;
                }
                std::string line(buffer);
                std::istringstream iss(line);
                std::vector<std::string> tokens;
                std::string token;
                while (iss >> token) {
                    tokens.push_back(token);
                }
                if (tokens.size() >= 11) {
                    std::cout << "      - " << proc << " (PID: " << tokens[1] 
                              << ", 사용자: " << tokens[0] << ")" << std::endl;
                } else {
                    std::cout << "      " << line;
                }
            }
            pclose(pipe);
        }
    }
    if (!found_gst) {
        std::cout << "      (실행 중인 프로세스 없음)" << std::endl;
    }
    
    if (!found_any) {
        std::cout << "  ✓ 카메라를 사용 중인 프로세스가 없습니다." << std::endl;
    }
    std::cout << std::endl;
}

bool check_encoder(const std::string& encoder_name) {
    GstRegistry* registry = gst_registry_get();
    GstPluginFeature* feature = gst_registry_lookup_feature(registry, encoder_name.c_str());
    if (feature) {
        gst_object_unref(feature);
        return true;
    }
    return false;
}

bool reset_usb_camera(const std::string& device_path) {
    // device_path 예: /dev/video0
    std::string dev_name = device_path.substr(device_path.find_last_of('/') + 1);
    
    // /sys/class/video4linux/videoX/device 경로 찾기
    std::string sys_path = "/sys/class/video4linux/" + dev_name + "/device";
    char link_target[256];
    ssize_t len = readlink(sys_path.c_str(), link_target, sizeof(link_target) - 1);
    if (len == -1) {
        std::cerr << "  ⚠ " << device_path << ": USB 디바이스 경로를 찾을 수 없습니다" << std::endl;
        return false;
    }
    link_target[len] = '\0';
    
    // 절대 경로로 변환
    std::string device_sys_path;
    if (link_target[0] == '/') {
        device_sys_path = link_target;
    } else {
        device_sys_path = "/sys/class/video4linux/" + dev_name + "/" + std::string(link_target);
    }
    
    // USB 디바이스 경로 찾기 (usb 디렉토리까지)
    std::string usb_device_path = device_sys_path;
    size_t usb_pos = usb_device_path.find("/usb");
    if (usb_pos != std::string::npos) {
        // usb 뒤의 경로 추출 (예: /usb1/1-1/1-1.1)
        usb_device_path = usb_device_path.substr(usb_pos);
        // 마지막 부분만 사용 (예: 1-1.1)
        size_t last_slash = usb_device_path.find_last_of('/');
        if (last_slash != std::string::npos) {
            usb_device_path = usb_device_path.substr(last_slash + 1);
        }
    } else {
        // 다른 방법: device 경로에서 직접 추출
        std::string cmd = "udevadm info --name " + device_path + " 2>/dev/null | grep 'DEVPATH=' | head -1";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[256];
            if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string line(buffer);
                size_t devpath_pos = line.find("DEVPATH=");
                if (devpath_pos != std::string::npos) {
                    std::string devpath = line.substr(devpath_pos + 8);
                    // 개행 제거
                    devpath.erase(std::remove(devpath.begin(), devpath.end(), '\n'), devpath.end());
                    // usb 경로에서 마지막 부분 추출
                    size_t usb_pos2 = devpath.find("/usb");
                    if (usb_pos2 != std::string::npos) {
                        std::string usb_part = devpath.substr(usb_pos2 + 4);
                        size_t first_slash = usb_part.find('/');
                        if (first_slash != std::string::npos) {
                            usb_device_path = usb_part.substr(first_slash + 1);
                            size_t last_slash2 = usb_device_path.find_last_of('/');
                            if (last_slash2 != std::string::npos) {
                                usb_device_path = usb_device_path.substr(last_slash2 + 1);
                            }
                        }
                    }
                }
            }
            pclose(pipe);
        }
    }
    
    if (usb_device_path.empty() || usb_device_path == device_sys_path) {
        std::cerr << "  ⚠ " << device_path << ": USB 디바이스 ID를 찾을 수 없습니다" << std::endl;
        return false;
    }
    
    std::cout << "  → " << device_path << " USB 재연결 중... (디바이스: " << usb_device_path << ")" << std::endl;
    
    // 더 간단한 방법: v4l2 디바이스를 직접 unbind/bind
    std::string unbind_path = "/sys/bus/v4l2/drivers/v4l2-video0/unbind";
    std::string bind_path = "/sys/bus/v4l2/drivers/v4l2-video0/bind";
    
    // 또는 더 직접적인 방법: USB 디바이스 자체를 unbind/bind
    // USB 디바이스 경로 찾기
    std::string usb_sys_path = "/sys/bus/usb/devices/" + usb_device_path;
    struct stat st;
    if (stat(usb_sys_path.c_str(), &st) != 0) {
        // 다른 방법 시도: lsusb로 VID:PID 찾기
        std::string cmd = "udevadm info --name " + device_path + " 2>/dev/null | grep 'ID_VENDOR_ID\\|ID_MODEL_ID'";
        FILE* pipe = popen(cmd.c_str(), "r");
        std::string vid, pid;
        if (pipe) {
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string line(buffer);
                if (line.find("ID_VENDOR_ID=") != std::string::npos) {
                    vid = line.substr(line.find('=') + 1);
                    vid.erase(std::remove(vid.begin(), vid.end(), '\n'), vid.end());
                } else if (line.find("ID_MODEL_ID=") != std::string::npos) {
                    pid = line.substr(line.find('=') + 1);
                    pid.erase(std::remove(pid.begin(), pid.end(), '\n'), pid.end());
                }
            }
            pclose(pipe);
        }
        
        if (!vid.empty() && !pid.empty()) {
            // lsusb로 디바이스 찾기
            cmd = "lsusb | grep -i '" + vid + ":" + pid + "'";
            pipe = popen(cmd.c_str(), "r");
            if (pipe) {
                char buffer[256];
                if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                    std::string line(buffer);
                    // Bus 001 Device 003 형식에서 디바이스 번호 추출
                    // 실제로는 더 복잡한 경로가 필요
                }
                pclose(pipe);
            }
        }
        
        std::cerr << "  ⚠ " << device_path << ": USB 디바이스 경로를 찾을 수 없습니다" << std::endl;
        return false;
    }
    
    // 권한 확인 (root 권한 필요할 수 있음)
    // unbind
    std::string unbind_file = usb_sys_path + "/driver/unbind";
    std::ofstream unbind_stream(unbind_file);
    if (unbind_stream.is_open()) {
        unbind_stream << usb_device_path;
        unbind_stream.close();
        std::cout << "    ✓ USB 디바이스 해제 완료" << std::endl;
    } else {
        std::cerr << "    ⚠ USB 디바이스 해제 실패 (권한 필요할 수 있음)" << std::endl;
        return false;
    }
    
    // 잠시 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // bind
    std::string bind_file = "/sys/bus/usb/drivers/usb/bind";
    std::ofstream bind_stream(bind_file);
    if (bind_stream.is_open()) {
        bind_stream << usb_device_path;
        bind_stream.close();
        std::cout << "    ✓ USB 디바이스 재연결 완료" << std::endl;
        
        // 재연결 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return true;
    } else {
        std::cerr << "    ⚠ USB 디바이스 재연결 실패 (권한 필요할 수 있음)" << std::endl;
        return false;
    }
}

bool reset_all_usb_cameras() {
    std::cout << "\n[USB 카메라 재연결]" << std::endl;
    
    // 모든 video 디바이스 찾기
    std::vector<std::string> camera_devices;
    for (int i = 0; i < 20; ++i) {
        std::string device = "/dev/video" + std::to_string(i);
        struct stat st;
        if (stat(device.c_str(), &st) == 0) {
            // USB 카메라인지 확인
            std::string cmd = "udevadm info --name " + device + " 2>/dev/null | grep -q 'ID_BUS=usb'";
            int result = system(cmd.c_str());
            if (result == 0) {
                camera_devices.push_back(device);
            }
        }
    }
    
    if (camera_devices.empty()) {
        std::cout << "  → USB 카메라를 찾을 수 없습니다" << std::endl;
        return false;
    }
    
    std::cout << "  → " << camera_devices.size() << "개의 USB 카메라 발견" << std::endl;
    
    bool all_success = true;
    for (const auto& device : camera_devices) {
        if (!reset_usb_camera(device)) {
            all_success = false;
        }
    }
    
    return all_success;
}
