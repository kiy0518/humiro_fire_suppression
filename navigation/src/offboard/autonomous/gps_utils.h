#ifndef GPS_UTILS_H
#define GPS_UTILS_H

#include <cmath>

namespace gps_utils {

// GPS 관련 상수
static constexpr double EARTH_RADIUS = 6371000.0;  // 지구 평균 반지름 (미터)
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

/**
 * @brief Haversine 공식을 사용한 두 GPS 좌표 간의 거리 계산
 * @param lat1 시작점 위도 (degrees)
 * @param lon1 시작점 경도 (degrees)
 * @param lat2 도착점 위도 (degrees)
 * @param lon2 도착점 경도 (degrees)
 * @return 두 점 사이의 거리 (미터)
 */
inline double haversineDistance(double lat1, double lon1,
                                double lat2, double lon2)
{
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return EARTH_RADIUS * c;
}

/**
 * @brief 두 GPS 좌표 간의 방위각(bearing) 계산
 * @param lat1 시작점 위도 (degrees)
 * @param lon1 시작점 경도 (degrees)
 * @param lat2 도착점 위도 (degrees)
 * @param lon2 도착점 경도 (degrees)
 * @return 방위각 (radians, 북쪽 기준 시계방향)
 */
inline double calculateBearing(double lat1, double lon1,
                               double lat2, double lon2)
{
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;

    double x = std::sin(dlon) * std::cos(lat2_rad);
    double y = std::cos(lat1_rad) * std::sin(lat2_rad) -
               std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon);

    return std::atan2(x, y);
}

/**
 * @brief GPS 좌표를 Local NED 좌표로 변환
 * @param home_lat Home 위도 (degrees)
 * @param home_lon Home 경도 (degrees)
 * @param home_alt Home 고도 (meters, AMSL)
 * @param target_lat 목표 위도 (degrees)
 * @param target_lon 목표 경도 (degrees)
 * @param target_alt 목표 고도 (meters, AMSL)
 * @param local_x 출력: Local X (북쪽, 미터)
 * @param local_y 출력: Local Y (동쪽, 미터)
 * @param local_z 출력: Local Z (아래, 미터, NED 좌표계)
 */
inline void gpsToLocalNED(double home_lat, double home_lon, float home_alt,
                          double target_lat, double target_lon, float target_alt,
                          float& local_x, float& local_y, float& local_z)
{
    // 위도 차이 (북쪽 방향)
    double lat_diff = target_lat - home_lat;
    local_x = static_cast<float>(lat_diff * DEG_TO_RAD * EARTH_RADIUS);

    // 경도 차이 (동쪽 방향)
    // cos(위도)로 보정 (위도가 높을수록 경도 간격이 좁아짐)
    double lon_diff = target_lon - home_lon;
    double lat_avg = (target_lat + home_lat) / 2.0;
    local_y = static_cast<float>(lon_diff * DEG_TO_RAD * EARTH_RADIUS * std::cos(lat_avg * DEG_TO_RAD));

    // 고도 차이 (NED에서는 아래 방향이 양수)
    local_z = -(target_alt - home_alt);
}

/**
 * @brief 3D 거리 계산 (수평 거리 + 고도 차이)
 * @param horizontal_distance 수평 거리 (미터)
 * @param altitude_diff 고도 차이 (미터)
 * @return 3D 거리 (미터)
 */
inline double calculate3DDistance(double horizontal_distance, double altitude_diff)
{
    return std::sqrt(horizontal_distance * horizontal_distance +
                     altitude_diff * altitude_diff);
}

}  // namespace gps_utils

#endif // GPS_UTILS_H
