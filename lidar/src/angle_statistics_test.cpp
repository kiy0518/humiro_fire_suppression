/**
 * 라이다 각도별 데이터 수신 통계 확인 프로그램
 * 공식 SDK 방식으로 수정된 코드에서 각도별 데이터 수신 상태를 확인
 */

#include "lidar_interface.h"
#include "lidar_config.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <atomic>

std::atomic<bool> is_running(true);
LidarInterface* lidar_interface = nullptr;

void signal_handler(int /*sig*/) {
    std::cout << "\n\n[종료 요청]" << std::endl;
    is_running = false;
    if (lidar_interface) {
        lidar_interface->stop();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "  라이다 각도별 데이터 수신 통계 확인\n";
    std::cout << "========================================\n\n";
    
    // 시그널 핸들러 등록
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 시리얼 포트 설정
    std::string device = "/dev/ttyS4";
    int baudrate = 230400;
    
    if (argc >= 2) {
        device = argv[1];
    }
    if (argc >= 3) {
        baudrate = std::stoi(argv[2]);
    }
    
    std::cout << "시리얼 포트: " << device << "\n";
    std::cout << "보드레이트: " << baudrate << "\n";
    std::cout << "종료: Ctrl+C\n\n";
    
    // 라이다 인터페이스 생성 및 시작
    LidarConfig config = LidarConfig::createGPIOUartConfig(device);
    config.baudrate = baudrate;
    lidar_interface = new LidarInterface(config);
    
    if (!lidar_interface->start()) {
        std::cerr << "라이다 시작 실패: " << lidar_interface->getLastError() << std::endl;
        delete lidar_interface;
        return 1;
    }
    
    std::cout << "라이다 연결 성공\n";
    std::cout << "데이터 수집 중... (5초 후 통계 출력)\n\n";
    
    // 각도별 통계 (10도씩 36개 구간)
    const int ANGLE_BINS = 36;  // 360도 / 10도 = 36개 구간
    const float BIN_SIZE = 360.0f / ANGLE_BINS;  // 10도
    
    // 통계 변수
    std::vector<int> angle_counts(ANGLE_BINS, 0);  // 각 구간별 포인트 개수
    std::vector<float> angle_min_dist(ANGLE_BINS, 999.0f);  // 각 구간별 최소 거리
    std::vector<float> angle_max_dist(ANGLE_BINS, 0.0f);  // 각 구간별 최대 거리
    std::vector<float> angle_avg_dist(ANGLE_BINS, 0.0f);  // 각 구간별 평균 거리
    std::vector<int> angle_total_points(ANGLE_BINS, 0);  // 각 구간별 총 포인트 수
    
    int total_scans = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    while (is_running) {
        LidarScan scan;
        if (lidar_interface->getLatestScan(scan)) {
            total_scans++;
            
            // 각도별 통계 업데이트
            for (const auto& point : scan.points) {
                // 각도를 0~360도 범위로 정규화
                float angle = point.angle;
                while (angle >= 360.0f) angle -= 360.0f;
                while (angle < 0.0f) angle += 360.0f;
                
                // 구간 인덱스 계산 (0~35)
                int bin_index = static_cast<int>(angle / BIN_SIZE);
                if (bin_index >= ANGLE_BINS) bin_index = ANGLE_BINS - 1;
                
                // 유효 범위 체크
                if (point.distance >= 0.05f && point.distance <= 12.0f) {
                    angle_counts[bin_index]++;
                    angle_total_points[bin_index]++;
                    
                    if (point.distance < angle_min_dist[bin_index]) {
                        angle_min_dist[bin_index] = point.distance;
                    }
                    if (point.distance > angle_max_dist[bin_index]) {
                        angle_max_dist[bin_index] = point.distance;
                    }
                    angle_avg_dist[bin_index] += point.distance;
                }
            }
        }
        
        // 5초마다 통계 출력
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - start_time).count();
        
        if (elapsed >= 5) {
            std::cout << "\n========================================\n";
            std::cout << "  각도별 데이터 수신 통계 (5초간)\n";
            std::cout << "========================================\n";
            std::cout << "총 스캔 수: " << total_scans << "\n";
            std::cout << "총 포인트 수: " << lidar_interface->getPointCount() << "\n\n";
            
            // 평균 거리 계산
            for (int i = 0; i < ANGLE_BINS; i++) {
                if (angle_total_points[i] > 0) {
                    angle_avg_dist[i] /= angle_total_points[i];
                }
            }
            
            // 각도별 통계 출력 (10도씩)
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "각도 범위    | 포인트 수 | 최소 거리 | 최대 거리 | 평균 거리\n";
            std::cout << "------------------------------------------------------------\n";
            
            for (int i = 0; i < ANGLE_BINS; i++) {
                float start_angle = i * BIN_SIZE;
                float end_angle = (i + 1) * BIN_SIZE;
                
                std::cout << std::setw(5) << start_angle << "~" 
                          << std::setw(5) << end_angle << "° | "
                          << std::setw(8) << angle_counts[i] << " | ";
                
                if (angle_total_points[i] > 0) {
                    std::cout << std::setw(8) << angle_min_dist[i] << "m | "
                              << std::setw(8) << angle_max_dist[i] << "m | "
                              << std::setw(8) << angle_avg_dist[i] << "m";
                } else {
                    std::cout << std::setw(8) << "N/A" << " | "
                              << std::setw(8) << "N/A" << " | "
                              << std::setw(8) << "N/A";
                }
                std::cout << "\n";
            }
            
            // 데이터 수신 상태 요약
            std::cout << "\n========================================\n";
            std::cout << "  데이터 수신 상태 요약\n";
            std::cout << "========================================\n";
            
            int bins_with_data = 0;
            int bins_without_data = 0;
            int total_points_in_bins = 0;
            
            for (int i = 0; i < ANGLE_BINS; i++) {
                if (angle_counts[i] > 0) {
                    bins_with_data++;
                    total_points_in_bins += angle_counts[i];
                } else {
                    bins_without_data++;
                }
            }
            
            std::cout << "데이터 수신된 구간: " << bins_with_data << " / " << ANGLE_BINS 
                      << " (" << (bins_with_data * 100.0f / ANGLE_BINS) << "%)\n";
            std::cout << "데이터 없는 구간: " << bins_without_data << " / " << ANGLE_BINS 
                      << " (" << (bins_without_data * 100.0f / ANGLE_BINS) << "%)\n";
            std::cout << "총 포인트 수: " << total_points_in_bins << "\n";
            std::cout << "구간당 평균 포인트 수: " 
                      << (bins_with_data > 0 ? total_points_in_bins / bins_with_data : 0) << "\n";
            
            // 데이터 없는 구간 목록
            if (bins_without_data > 0) {
                std::cout << "\n데이터 없는 구간:\n";
                for (int i = 0; i < ANGLE_BINS; i++) {
                    if (angle_counts[i] == 0) {
                        float start_angle = i * BIN_SIZE;
                        float end_angle = (i + 1) * BIN_SIZE;
                        std::cout << "  " << start_angle << "~" << end_angle << "°\n";
                    }
                }
            }
            
            std::cout << "\n계속 수집 중... (다음 통계는 5초 후)\n";
            std::cout << "종료하려면 Ctrl+C를 누르세요\n\n";
            
            // 통계 초기화
            std::fill(angle_counts.begin(), angle_counts.end(), 0);
            std::fill(angle_min_dist.begin(), angle_min_dist.end(), 999.0f);
            std::fill(angle_max_dist.begin(), angle_max_dist.end(), 0.0f);
            std::fill(angle_avg_dist.begin(), angle_avg_dist.end(), 0.0f);
            std::fill(angle_total_points.begin(), angle_total_points.end(), 0);
            total_scans = 0;
            start_time = std::chrono::steady_clock::now();
        }
        
        usleep(10000);  // 10ms 대기
    }
    
    // 정리
    if (lidar_interface) {
        lidar_interface->stop();
        delete lidar_interface;
    }
    
    std::cout << "\n프로그램 종료\n";
    return 0;
}

