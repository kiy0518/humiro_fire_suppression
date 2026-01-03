#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <gst/gst.h>
#include <opencv2/core/utils/logger.hpp>

#include "config.h"
#include "src/application_manager.h"

int main(int argc, char* argv[]) {
    // OpenCV 경고 억제 (열화상 카메라 타임아웃은 정상 동작)
    setenv("OPENCV_FFMPEG_LOGLEVEL", "-8", 0);
    setenv("OPENCV_LOG_LEVEL", "ERROR", 0);
    
    // OpenCV 로그 레벨 설정 시도 (OpenCV 4.5+)
#if CV_VERSION_MAJOR >= 4
    #if CV_VERSION_MINOR >= 5
        try {
            cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
        } catch (...) {
            // 함수가 없으면 무시
        }
    #endif
#endif
    
    // GStreamer 초기화
    gst_init(&argc, &argv);
    
    // ApplicationManager 생성 및 초기화
    ApplicationManager app;
    
    if (!app.initialize(argc, argv)) {
        std::cerr << "Failed to initialize application" << std::endl;
        return 1;
    }
    
    // 애플리케이션 실행
    app.run();
    
    // 종료 (소멸자에서 자동으로 리소스 정리)
    app.shutdown();
    
    std::cout << "\n[완료] 모든 리소스 해제 완료" << std::endl;
    
    return 0;
}
