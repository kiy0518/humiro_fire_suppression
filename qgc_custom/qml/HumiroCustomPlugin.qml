import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0

// Humiro Fire Suppression 커스텀 플러그인
Item {
    id: humiroPlugin
    anchors.fill: parent

    // QGC 연동 (필요시 추가)
    property var activeVehicle: QGroundControl.multiVehicleManager.activeVehicle

    // 메인 레이아웃
    RowLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth
        spacing: ScreenTools.defaultFontPixelWidth

        // 왼쪽: 미션 상태 모니터링
        FormationStatusPanel {
            id: formationStatus
            Layout.fillHeight: true
            Layout.preferredWidth: 320
        }

        // 중앙: 지도 뷰 (QGC 기본 지도 사용)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1E1E1E"
            radius: 8

            Text {
                anchors.centerIn: parent
                text: "QGC Map View\n(Use default QGC map)"
                font.pixelSize: 16
                color: "#CCCCCC"
                horizontalAlignment: Text.AlignHCenter
            }
        }

        // 오른쪽: 화재 지점 및 격발 제어
        ColumnLayout {
            Layout.fillHeight: true
            Layout.preferredWidth: 320
            spacing: ScreenTools.defaultFontPixelWidth

            // 화재 지점 관리
            FirePointPanel {
                id: firePointPanel
                Layout.fillWidth: true
                Layout.fillHeight: true

                onAddFirePointClicked: {
                    console.log("Add fire point clicked")
                    // TODO: QGC 지도에서 클릭 대기
                }

                onMissionStartRequested: {
                    console.log("Mission start requested: Target=" + targetId + ", Lat=" + lat + ", Lon=" + lon)
                    // FIRE_MISSION_START 메시지 전송
                    sendFireMissionStart(lat, lon, 10.0, false, 6)  // alt=10m, auto=false, max=6
                }
            }

            // 격발 제어
            FireControlPanel {
                id: fireControlPanel
                Layout.fillWidth: true
                Layout.preferredHeight: 300

                onFireCommandSent: {
                    console.log("Fire command sent: Confirm=" + confirm)
                    // FIRE_LAUNCH_CONTROL 메시지 전송 (command: 0=CONFIRM, 1=ABORT)
                    sendMAVLinkFireCommand(confirm)
                }

                onStatusRequested: {
                    console.log("Status requested")
                    // FIRE_LAUNCH_CONTROL 메시지 전송 (command: 2=REQUEST_STATUS)
                    sendMAVLinkStatusRequest()
                }
            }
        }
    }

    // MAVLink 메시지 핸들러
    Connections {
        target: activeVehicle

        // FIRE_MISSION_STATUS 수신 (ID: 50001)
        function onMessageReceived(message) {
            if (message.id === 50001) {  // FIRE_MISSION_STATUS
                var phase = message.phase  // FIRE_MISSION_PHASE (0-6)
                var progress = message.progress  // 0-100%
                var remaining_projectiles = message.remaining_projectiles
                var distance_to_target = message.distance_to_target
                var thermal_max_temp = message.thermal_max_temp / 10.0  // °C * 10 → °C
                var status_text = message.status_text

                // 미션 상태 업데이트
                formationStatus.updateMissionStatus(phase, progress, remaining_projectiles, distance_to_target, thermal_max_temp, status_text)

                // 격발 제어 상태 업데이트 (phase 기반)
                var isFiring = (phase === 4)  // FIRE_PHASE_SUPPRESSING
                fireControlPanel.updateFireState(phase, isFiring)
            }
            // FIRE_SUPPRESSION_RESULT 수신 (ID: 50003)
            else if (message.id === 50003) {
                var shot_number = message.shot_number
                var temp_before = message.temp_before / 10.0  // °C * 10 → °C
                var temp_after = message.temp_after / 10.0  // °C * 10 → °C
                var success = message.success  // 0=failed, 1=success

                // 진압 결과 업데이트
                firePointPanel.updateSuppressionResult(shot_number, temp_before, temp_after, success)
            }
        }
    }

    // MAVLink 메시지 전송 함수
    function sendMAVLinkFireCommand(confirm) {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // FIRE_LAUNCH_CONTROL (ID: 50002) 전송
        var message = activeVehicle.createMAVLinkMessage(50002)
        message.target_system = 1  // VIM4 시스템 ID
        message.target_component = 1  // VIM4 컴포넌트 ID
        message.command = confirm ? 0 : 1  // 0=CONFIRM, 1=ABORT

        activeVehicle.sendMessage(message)
        console.log("Sent FIRE_LAUNCH_CONTROL: Command=" + (confirm ? "CONFIRM" : "ABORT"))
    }

    function sendMAVLinkStatusRequest() {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // FIRE_LAUNCH_CONTROL (ID: 50002) 전송 - REQUEST_STATUS
        var message = activeVehicle.createMAVLinkMessage(50002)
        message.target_system = 1  // VIM4 시스템 ID
        message.target_component = 1  // VIM4 컴포넌트 ID
        message.command = 2  // REQUEST_STATUS

        activeVehicle.sendMessage(message)
        console.log("Sent FIRE_LAUNCH_CONTROL: Command=REQUEST_STATUS")
    }

    function sendFireMissionStart(lat, lon, alt, autoFire, maxProjectiles) {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // FIRE_MISSION_START (ID: 50000) 전송
        var message = activeVehicle.createMAVLinkMessage(50000)
        message.target_system = 1  // VIM4 시스템 ID
        message.target_component = 1  // VIM4 컴포넌트 ID
        message.target_lat = lat * 1e7  // degrees * 1e7
        message.target_lon = lon * 1e7  // degrees * 1e7
        message.target_alt = alt  // meters MSL
        message.auto_fire = autoFire ? 1 : 0  // 0=manual, 1=auto
        message.max_projectiles = maxProjectiles  // Max projectiles to use

        activeVehicle.sendMessage(message)
        console.log("Sent FIRE_MISSION_START: Lat=" + lat + ", Lon=" + lon + ", Alt=" + alt)
    }

    function sendFireSetMode(px4Mode) {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // FIRE_SET_MODE (ID: 50004) 전송
        var message = activeVehicle.createMAVLinkMessage(50004)
        message.target_system = 1  // FC 시스템 ID
        message.target_component = 1  // FC 컴포넌트 ID
        message.px4_mode = px4Mode  // PX4 mode (1-8)

        activeVehicle.sendMessage(message)
        console.log("Sent FIRE_SET_MODE: PX4_Mode=" + px4Mode)
    }

    // 테스트 데이터 (개발용)
    Component.onCompleted: {
        // 테스트 미션 상태 (FIRE_MISSION_STATUS)
        formationStatus.updateMissionStatus(1, 45, 5, 25.5, 85.0, "Flying to target")

        // 테스트 목표 지점 추가
        firePointPanel.addFirePoint(1, 37.5672, 126.9788, 0.8)

        // 테스트 진압 결과
        firePointPanel.updateSuppressionResult(1, 120.0, 45.0, 1)  // 성공

        // 격발 제어 상태
        fireControlPanel.updateFireState(3, false)  // FIRE_PHASE_READY_TO_FIRE
    }
}
