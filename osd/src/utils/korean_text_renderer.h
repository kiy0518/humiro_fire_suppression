#ifndef KOREAN_TEXT_RENDERER_H
#define KOREAN_TEXT_RENDERER_H

#include <opencv2/opencv.hpp>
#include <string>

/**
 * 한글 텍스트 렌더링 유틸리티
 * OpenCV의 putText는 한글을 지원하지 않으므로, 
 * FreeType 또는 PIL을 사용하여 한글 텍스트를 이미지로 렌더링
 */
class KoreanTextRenderer {
public:
    /**
     * 한글 텍스트를 이미지로 렌더링
     * @param text 한글 텍스트
     * @param font_size 폰트 크기
     * @param color 텍스트 색상 (BGR)
     * @param thickness 폰트 두께
     * @param font_path 폰트 파일 경로 (기본값: 시스템 기본 한글 폰트)
     * @return 렌더링된 텍스트 이미지 (cv::Mat)
     */
    static cv::Mat renderText(
        const std::string& text,
        double font_size = 1.0,
        const cv::Scalar& color = cv::Scalar(255, 255, 255),
        int thickness = 1,
        const std::string& font_path = ""
    );
    
    /**
     * 한글 텍스트 크기 계산
     * @param text 한글 텍스트
     * @param font_size 폰트 크기
     * @param thickness 폰트 두께
     * @return 텍스트 크기 (width, height)
     */
    static cv::Size getTextSize(
        const std::string& text,
        double font_size = 1.0,
        int thickness = 1
    );
    
    /**
     * 프레임에 한글 텍스트 그리기
     * @param frame 출력 프레임
     * @param text 한글 텍스트
     * @param position 텍스트 위치 (좌상단)
     * @param font_size 폰트 크기
     * @param color 텍스트 색상 (BGR)
     * @param thickness 폰트 두께
     * @param font_path 폰트 파일 경로
     */
    static void putText(
        cv::Mat& frame,
        const std::string& text,
        const cv::Point& position,
        double font_size = 1.0,
        const cv::Scalar& color = cv::Scalar(255, 255, 255),
        int thickness = 1,
        const std::string& font_path = ""
    );
    
private:
    /**
     * 시스템 기본 한글 폰트 경로 찾기
     * @return 폰트 파일 경로
     */
    static std::string findDefaultKoreanFont();
    
    /**
     * Python PIL을 사용하여 한글 텍스트 렌더링 (fallback)
     * @param text 한글 텍스트
     * @param font_size 폰트 크기
     * @param color 텍스트 색상 (RGB)
     * @return 렌더링된 텍스트 이미지 (cv::Mat)
     */
    static cv::Mat renderTextWithPIL(
        const std::string& text,
        double font_size,
        const cv::Scalar& color
    );
};

#endif // KOREAN_TEXT_RENDERER_H

