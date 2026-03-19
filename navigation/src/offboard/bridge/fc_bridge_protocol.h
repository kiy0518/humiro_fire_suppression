/**
 * @file fc_bridge_protocol.h
 * @brief FC Bridge IPC 프로토콜 정의
 *
 * fc_bridge (Domain 0) ↔ 메인앱 (Domain 1) 간 로컬 UDP 통신 프로토콜.
 * packed 구조체로 직렬화 오버헤드 없이 직접 전송.
 */

#ifndef FC_BRIDGE_PROTOCOL_H
#define FC_BRIDGE_PROTOCOL_H

#include <cstdint>
#include <cmath>

#pragma pack(push, 1)

/**
 * fc_bridge → 메인앱 (50Hz, UDP 127.0.0.1:17001)
 * FC 상태 데이터 패킷
 */
struct FCState {
    uint32_t magic = 0x46435354;  // "FCST"
    uint64_t timestamp_us;

    // 위치 (GPS)
    double latitude;
    double longitude;
    float altitude_amsl;

    // 위치 (로컬 NED)
    float local_x;
    float local_y;
    float local_z;

    // 속도 (NED m/s)
    float vx;
    float vy;
    float vz;

    // 자세 (radians)
    float roll;
    float pitch;
    float yaw;

    // FC 상태
    uint8_t nav_state;
    uint8_t arming_state;

    // 배터리
    float battery_voltage;
    float battery_remaining;  // 0.0~1.0 (-1 if invalid)

    // GPS
    uint8_t gps_fix_type;     // 0=No, 2=2D, 3=3D, 4=DGPS, 5=RTK
    uint8_t gps_satellites;
    float gps_hdop;

    // 하방 거리계
    float dist_bottom;        // 지면까지 거리 (m), 무효 시 -1
    uint8_t dist_bottom_valid; // 0 or 1 (실제 거리센서 유효 여부)

    // Home 고도 (QGC alt(rel) 기준)
    float home_alt;           // Home AMSL 고도 (m), 미설정 시 NAN
    uint8_t home_alt_valid;   // 0 or 1

    // PX4 착지 감지 (vehicle_land_detected)
    uint8_t land_detected;    // 1 = PX4가 착지 확인 (msg->landed)
    uint8_t maybe_landed;     // 1 = 착지 가능성 있음 (msg->maybe_landed)

    // 연결 상태
    uint8_t fc_connected;     // 0 or 1

    /** magic 필드 검증 */
    bool isValid() const { return magic == 0x46435354; }
};

/**
 * 메인앱 → fc_bridge (10Hz+이벤트, UDP 127.0.0.1:17002)
 * FC 명령 패킷
 */
enum class FCCommandType : uint8_t {
    HEARTBEAT = 1,    // OffboardControlMode
    SETPOINT = 2,     // TrajectorySetpoint
    VEHICLE_CMD = 3,  // VehicleCommand (ARM, 모드전환 등)
};

struct FCCommand {
    uint32_t magic = 0x46434D44;  // "FCMD"
    uint8_t command_type;

    // HEARTBEAT 필드 (OffboardControlMode)
    bool position;
    bool velocity;
    bool acceleration;

    // SETPOINT 필드 (TrajectorySetpoint)
    float x, y, z;           // NAN = 미사용
    float vx, vy, vz;        // NAN = 미사용
    float yaw, yaw_speed;    // NAN = 미사용

    // VEHICLE_CMD 필드 (VehicleCommand)
    uint16_t command;
    float param1, param2, param3;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;

    /** magic 필드 검증 */
    bool isValid() const { return magic == 0x46434D44; }

    /** HEARTBEAT 명령 생성 */
    static FCCommand makeHeartbeat(bool pos = true, bool vel = false, bool acc = false) {
        FCCommand cmd{};
        cmd.command_type = static_cast<uint8_t>(FCCommandType::HEARTBEAT);
        cmd.position = pos;
        cmd.velocity = vel;
        cmd.acceleration = acc;
        cmd.x = cmd.y = cmd.z = NAN;
        cmd.vx = cmd.vy = cmd.vz = NAN;
        cmd.yaw = cmd.yaw_speed = NAN;
        return cmd;
    }

    /** SETPOINT 명령 생성 (position) */
    static FCCommand makePositionSetpoint(float x, float y, float z, float yaw) {
        FCCommand cmd{};
        cmd.command_type = static_cast<uint8_t>(FCCommandType::SETPOINT);
        cmd.x = x;
        cmd.y = y;
        cmd.z = z;
        cmd.yaw = yaw;
        cmd.vx = cmd.vy = cmd.vz = NAN;
        cmd.yaw_speed = NAN;
        return cmd;
    }

    /** SETPOINT 명령 생성 (velocity) */
    static FCCommand makeVelocitySetpoint(float vx, float vy, float vz, float yaw) {
        FCCommand cmd{};
        cmd.command_type = static_cast<uint8_t>(FCCommandType::SETPOINT);
        cmd.x = cmd.y = cmd.z = NAN;
        cmd.vx = vx;
        cmd.vy = vy;
        cmd.vz = vz;
        cmd.yaw = yaw;
        cmd.yaw_speed = NAN;
        return cmd;
    }

    /** SETPOINT 명령 생성 (full TrajectorySetpoint) */
    static FCCommand makeSetpoint(float x, float y, float z,
                                   float vx, float vy, float vz,
                                   float yaw, float yaw_speed) {
        FCCommand cmd{};
        cmd.command_type = static_cast<uint8_t>(FCCommandType::SETPOINT);
        cmd.x = x; cmd.y = y; cmd.z = z;
        cmd.vx = vx; cmd.vy = vy; cmd.vz = vz;
        cmd.yaw = yaw; cmd.yaw_speed = yaw_speed;
        return cmd;
    }

    /** VEHICLE_CMD 명령 생성 */
    static FCCommand makeVehicleCommand(uint16_t command, float p1, float p2, float p3,
                                         uint8_t target_sys, uint8_t target_comp = 1,
                                         uint8_t source_sys = 1) {
        FCCommand cmd{};
        cmd.command_type = static_cast<uint8_t>(FCCommandType::VEHICLE_CMD);
        cmd.command = command;
        cmd.param1 = p1;
        cmd.param2 = p2;
        cmd.param3 = p3;
        cmd.target_system = target_sys;
        cmd.target_component = target_comp;
        cmd.source_system = source_sys;
        cmd.x = cmd.y = cmd.z = NAN;
        cmd.vx = cmd.vy = cmd.vz = NAN;
        cmd.yaw = cmd.yaw_speed = NAN;
        return cmd;
    }
};

#pragma pack(pop)

// IPC 포트 기본값
static constexpr uint16_t FC_BRIDGE_STATE_PORT = 17001;   // fc_bridge → 메인앱
static constexpr uint16_t FC_BRIDGE_COMMAND_PORT = 17002;  // 메인앱 → fc_bridge
static constexpr uint32_t FC_BRIDGE_TIMEOUT_MS = 500;      // 연결 끊김 판정 타임아웃

#endif // FC_BRIDGE_PROTOCOL_H
