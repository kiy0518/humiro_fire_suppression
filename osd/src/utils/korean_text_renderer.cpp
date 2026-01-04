#include "korean_text_renderer.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <algorithm>

#ifdef HAVE_FREETYPE
#include <ft2build.h>
#include FT_FREETYPE_H
#endif

cv::Mat KoreanTextRenderer::renderText(
    const std::string& text,
    double font_size,
    const cv::Scalar& color,
    int thickness,
    const std::string& font_path
) {
    if (text.empty()) {
        return cv::Mat::zeros(1, 1, CV_8UC3);
    }
    
    std::string actual_font_path = font_path;
    if (actual_font_path.empty()) {
        actual_font_path = findDefaultKoreanFont();
    }
    
#ifdef HAVE_FREETYPE
    // FreeType을 사용한 렌더링 시도
    if (!actual_font_path.empty() && std::ifstream(actual_font_path).good()) {
        try {
            FT_Library library;
            if (FT_Init_FreeType(&library) != 0) {
                std::cerr << "  [KoreanTextRenderer] FreeType 초기화 실패, PIL 사용" << std::endl;
                return renderTextWithPIL(text, font_size, color);
            }
            
            FT_Face face;
            if (FT_New_Face(library, actual_font_path.c_str(), 0, &face) != 0) {
                FT_Done_FreeType(library);
                std::cerr << "  [KoreanTextRenderer] 폰트 로드 실패: " << actual_font_path << ", PIL 사용" << std::endl;
                return renderTextWithPIL(text, font_size, color);
            }
            
            // 폰트 크기 설정
            int pixel_size = static_cast<int>(font_size * 32);  // FreeType은 1/64 픽셀 단위
            FT_Set_Pixel_Sizes(face, 0, pixel_size);
            
            // 텍스트 크기 계산
            int max_width = 0;
            int max_height = 0;
            int baseline = 0;
            
            for (size_t i = 0; i < text.length(); ) {
                unsigned int codepoint;
                int bytes = 0;
                
                // UTF-8 디코딩
                if ((text[i] & 0x80) == 0) {
                    codepoint = text[i];
                    bytes = 1;
                } else if ((text[i] & 0xE0) == 0xC0) {
                    codepoint = ((text[i] & 0x1F) << 6) | (text[i+1] & 0x3F);
                    bytes = 2;
                } else if ((text[i] & 0xF0) == 0xE0) {
                    codepoint = ((text[i] & 0x0F) << 12) | ((text[i+1] & 0x3F) << 6) | (text[i+2] & 0x3F);
                    bytes = 3;
                } else if ((text[i] & 0xF8) == 0xF0) {
                    codepoint = ((text[i] & 0x07) << 18) | ((text[i+1] & 0x3F) << 12) | 
                                ((text[i+2] & 0x3F) << 6) | (text[i+3] & 0x3F);
                    bytes = 4;
                } else {
                    i++;
                    continue;
                }
                
                if (FT_Load_Char(face, codepoint, FT_LOAD_RENDER) != 0) {
                    i += bytes;
                    continue;
                }
                
                max_width += face->glyph->advance.x >> 6;
                max_height = std::max(max_height, static_cast<int>(face->glyph->bitmap.rows));
                baseline = std::max(baseline, static_cast<int>(face->glyph->bitmap_top));
                
                i += bytes;
            }
            
            // 이미지 생성
            cv::Mat img(max_height + baseline, max_width, CV_8UC3, cv::Scalar(0, 0, 0));
            
            int x = 0;
            for (size_t i = 0; i < text.length(); ) {
                unsigned int codepoint;
                int bytes = 0;
                
                // UTF-8 디코딩
                if ((text[i] & 0x80) == 0) {
                    codepoint = text[i];
                    bytes = 1;
                } else if ((text[i] & 0xE0) == 0xC0) {
                    codepoint = ((text[i] & 0x1F) << 6) | (text[i+1] & 0x3F);
                    bytes = 2;
                } else if ((text[i] & 0xF0) == 0xE0) {
                    codepoint = ((text[i] & 0x0F) << 12) | ((text[i+1] & 0x3F) << 6) | (text[i+2] & 0x3F);
                    bytes = 3;
                } else if ((text[i] & 0xF8) == 0xF0) {
                    codepoint = ((text[i] & 0x07) << 18) | ((text[i+1] & 0x3F) << 12) | 
                                ((text[i+2] & 0x3F) << 6) | (text[i+3] & 0x3F);
                    bytes = 4;
                } else {
                    i++;
                    continue;
                }
                
                if (FT_Load_Char(face, codepoint, FT_LOAD_RENDER) != 0) {
                    i += bytes;
                    continue;
                }
                
                FT_Bitmap& bitmap = face->glyph->bitmap;
                int glyph_top = baseline - face->glyph->bitmap_top;
                
                for (unsigned int row = 0; row < bitmap.rows; ++row) {
                    for (unsigned int col = 0; col < bitmap.width; ++col) {
                        int y = glyph_top + row;
                        if (y >= 0 && y < img.rows && (x + col) < img.cols) {
                            unsigned char alpha = bitmap.buffer[row * bitmap.width + col];
                            if (alpha > 0) {
                                cv::Vec3b& pixel = img.at<cv::Vec3b>(y, x + col);
                                pixel[0] = static_cast<unsigned char>(color[0] * alpha / 255.0 + pixel[0] * (255 - alpha) / 255.0);
                                pixel[1] = static_cast<unsigned char>(color[1] * alpha / 255.0 + pixel[1] * (255 - alpha) / 255.0);
                                pixel[2] = static_cast<unsigned char>(color[2] * alpha / 255.0 + pixel[2] * (255 - alpha) / 255.0);
                            }
                        }
                    }
                }
                
                x += face->glyph->advance.x >> 6;
                i += bytes;
            }
            
            FT_Done_Face(face);
            FT_Done_FreeType(library);
            
            return img;
        } catch (...) {
            std::cerr << "  [KoreanTextRenderer] FreeType 렌더링 예외 발생, PIL 사용" << std::endl;
            return renderTextWithPIL(text, font_size, color);
        }
    }
#endif
    
    // FreeType이 없거나 실패한 경우 PIL 사용
    return renderTextWithPIL(text, font_size, color);
}

cv::Size KoreanTextRenderer::getTextSize(
    const std::string& text,
    double font_size,
    int thickness
) {
    cv::Mat rendered = renderText(text, font_size, cv::Scalar(255, 255, 255), thickness);
    return cv::Size(rendered.cols, rendered.rows);
}

void KoreanTextRenderer::putText(
    cv::Mat& frame,
    const std::string& text,
    const cv::Point& position,
    double font_size,
    const cv::Scalar& color,
    int thickness,
    const std::string& font_path
) {
    if (text.empty() || frame.empty()) {
        return;
    }
    
    cv::Mat text_img = renderText(text, font_size, color, thickness, font_path);
    
    if (text_img.empty()) {
        return;
    }
    
    // 프레임에 텍스트 이미지 합성
    cv::Rect roi(position.x, position.y, 
                 std::min(text_img.cols, frame.cols - position.x),
                 std::min(text_img.rows, frame.rows - position.y));
    
    if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame.cols && roi.y + roi.height <= frame.rows) {
        cv::Mat text_roi = text_img(cv::Rect(0, 0, roi.width, roi.height));
        cv::Mat frame_roi = frame(roi);
        
        // 알파 블렌딩 (텍스트가 투명 배경인 경우)
        if (text_img.channels() == 4) {
            // RGBA 이미지인 경우
            for (int y = 0; y < text_roi.rows; ++y) {
                for (int x = 0; x < text_roi.cols; ++x) {
                    cv::Vec4b text_pixel = text_roi.at<cv::Vec4b>(y, x);
                    if (text_pixel[3] > 0) {
                        double alpha = text_pixel[3] / 255.0;
                        cv::Vec3b& frame_pixel = frame_roi.at<cv::Vec3b>(y, x);
                        frame_pixel[0] = static_cast<unsigned char>(text_pixel[0] * alpha + frame_pixel[0] * (1 - alpha));
                        frame_pixel[1] = static_cast<unsigned char>(text_pixel[1] * alpha + frame_pixel[1] * (1 - alpha));
                        frame_pixel[2] = static_cast<unsigned char>(text_pixel[2] * alpha + frame_pixel[2] * (1 - alpha));
                    }
                }
            }
        } else {
            // BGR 이미지인 경우 (단순 복사)
            text_roi.copyTo(frame_roi);
        }
    }
}

std::string KoreanTextRenderer::findDefaultKoreanFont() {
    // Ubuntu/Debian 시스템에서 일반적인 한글 폰트 경로
    std::vector<std::string> font_paths = {
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
        "/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf",
        "/System/Library/Fonts/AppleGothic.ttf",  // macOS
        "C:/Windows/Fonts/malgun.ttf",  // Windows
    };
    
    for (const auto& path : font_paths) {
        std::ifstream file(path);
        if (file.good()) {
            file.close();
            return path;
        }
    }
    
    // 환경 변수에서 폰트 경로 찾기
    const char* font_dir = std::getenv("FONT_DIR");
    if (font_dir) {
        std::string custom_path = std::string(font_dir) + "/NanumGothic.ttf";
        std::ifstream file(custom_path);
        if (file.good()) {
            file.close();
            return custom_path;
        }
    }
    
    return "";
}

cv::Mat KoreanTextRenderer::renderTextWithPIL(
    const std::string& text,
    double font_size,
    const cv::Scalar& color
) {
    // Python PIL을 사용하여 한글 텍스트 렌더링
    // 임시 파일에 Python 스크립트 작성
    std::string script_path = "/tmp/render_korean_text.py";
    std::ofstream script(script_path);
    
    if (!script.is_open()) {
        std::cerr << "  [KoreanTextRenderer] Python 스크립트 파일 생성 실패" << std::endl;
        return cv::Mat::zeros(1, 1, CV_8UC3);
    }
    
    // 텍스트를 이스케이프 처리
    std::string escaped_text = text;
    std::replace(escaped_text.begin(), escaped_text.end(), '\'', '\\\'');
    std::replace(escaped_text.begin(), escaped_text.end(), '\"', '\\\"');
    
    script << "#!/usr/bin/env python3\n";
    script << "# -*- coding: utf-8 -*-\n";
    script << "import sys\n";
    script << "import numpy as np\n";
    script << "from PIL import Image, ImageDraw, ImageFont\n";
    script << "import cv2\n\n";
    script << "text = '" << escaped_text << "'\n";
    script << "font_size = " << static_cast<int>(font_size * 40) << "\n";
    script << "color_rgb = (" << static_cast<int>(color[2]) << ", " 
           << static_cast<int>(color[1]) << ", " << static_cast<int>(color[0]) << ")\n\n";
    script << "# 한글 폰트 찾기\n";
    script << "font_paths = [\n";
    script << "    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',\n";
    script << "    '/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf',\n";
    script << "    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',\n";
    script << "    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',\n";
    script << "]\n\n";
    script << "font = None\n";
    script << "for path in font_paths:\n";
    script << "    try:\n";
    script << "        font = ImageFont.truetype(path, font_size)\n";
    script << "        break\n";
    script << "    except:\n";
    script << "        continue\n\n";
    script << "if font is None:\n";
    script << "    font = ImageFont.load_default()\n\n";
    script << "# 텍스트 크기 계산\n";
    script << "img_temp = Image.new('RGB', (1, 1))\n";
    script << "draw_temp = ImageDraw.Draw(img_temp)\n";
    script << "bbox = draw_temp.textbbox((0, 0), text, font=font)\n";
    script << "text_width = bbox[2] - bbox[0]\n";
    script << "text_height = bbox[3] - bbox[1]\n\n";
    script << "# 이미지 생성\n";
    script << "img = Image.new('RGB', (text_width + 10, text_height + 10), (0, 0, 0))\n";
    script << "draw = ImageDraw.Draw(img)\n";
    script << "draw.text((5, 5), text, fill=color_rgb, font=font)\n\n";
    script << "# NumPy 배열로 변환\n";
    script << "img_array = np.array(img)\n";
    script << "img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)\n\n";
    script << "# 바이너리로 출력\n";
    script << "sys.stdout.buffer.write(img_bgr.tobytes())\n";
    script << "sys.stdout.buffer.write(np.array([img_bgr.shape[0], img_bgr.shape[1], img_bgr.shape[2]], dtype=np.int32).tobytes())\n";
    
    script.close();
    
    // Python 스크립트 실행
    std::string command = "python3 " + script_path + " 2>/dev/null";
    FILE* pipe = popen(command.c_str(), "r");
    
    if (!pipe) {
        std::cerr << "  [KoreanTextRenderer] Python 실행 실패" << std::endl;
        return cv::Mat::zeros(1, 1, CV_8UC3);
    }
    
    // 결과 읽기
    std::vector<unsigned char> buffer;
    char chunk[4096];
    while (fread(chunk, 1, sizeof(chunk), pipe) > 0) {
        buffer.insert(buffer.end(), chunk, chunk + sizeof(chunk));
    }
    
    int status = pclose(pipe);
    
    if (status != 0 || buffer.size() < 12) {
        std::cerr << "  [KoreanTextRenderer] PIL 렌더링 실패 (Python 오류 또는 데이터 부족)" << std::endl;
        return cv::Mat::zeros(1, 1, CV_8UC3);
    }
    
    // 마지막 12바이트는 shape 정보
    int32_t shape[3];
    memcpy(shape, buffer.data() + buffer.size() - 12, 12);
    int height = shape[0];
    int width = shape[1];
    int channels = shape[2];
    
    if (buffer.size() < static_cast<size_t>(width * height * channels + 12)) {
        std::cerr << "  [KoreanTextRenderer] 데이터 크기 불일치" << std::endl;
        return cv::Mat::zeros(1, 1, CV_8UC3);
    }
    
    // 이미지 데이터 추출
    std::vector<unsigned char> img_data(buffer.begin(), buffer.end() - 12);
    cv::Mat result(height, width, CV_8UC3, img_data.data());
    
    return result.clone();
}

