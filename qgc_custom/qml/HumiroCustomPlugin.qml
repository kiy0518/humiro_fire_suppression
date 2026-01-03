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

        // 왼쪽: 편대 상태 모니터링
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
            }

            // 격발 제어
            FireControlPanel {
                id: fireControlPanel
                Layout.fillWidth: true
                Layout.preferredHeight: 300

                onFireCommandSent: {
                    console.log("Fire command sent: Drone=" + droneId + ", Enable=" + enable)
                    // TODO: MAVLink FIRE_COMMAND 메시지 전송
                    sendMAVLinkFireCommand(droneId, enable)
                }
            }
        }
    }

    // MAVLink 메시지 핸들러
    Connections {
        target: activeVehicle

        // FORMATION_MEMBER_STATUS 수신
        function onMessageReceived(message) {
            if (message.id === 12920) {  // FORMATION_MEMBER_STATUS
                var droneId = message.drone_id
                var lat = message.lat / 1e7
                var lon = message.lon / 1e7
                var battery = message.battery_percent
                var ammo = message.ammo_count
                var state = message.mission_state
                var targetId = message.target_id
                var isLeader = (droneId === 1)  // Drone 1이 리더

                // 편대 상태 업데이트
                formationStatus.updateDroneStatus(droneId, lat, lon, battery, ammo, state, targetId, isLeader)

                // 격발 제어 상태 업데이트
                var isFiring = (state === 6)  // FIRING
                fireControlPanel.updateDroneFireState(droneId, state, isFiring)
            }
            // MISSION_PROGRESS 수신
            else if (message.id === 12923) {
                var targetId = message.target_id
                var assignedDrone = message.drone_id
                var progress = message.progress_status

                // 화재 지점 진행 상황 업데이트
                firePointPanel.updateFirePointProgress(targetId, assignedDrone, progress)
            }
        }
    }

    // MAVLink 메시지 전송 함수
    function sendMAVLinkFireCommand(droneId, enable) {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // FIRE_COMMAND (ID: 12922) 전송
        var message = activeVehicle.createMAVLinkMessage(12922)
        message.drone_id = droneId
        message.fire_enable = enable ? 1 : 0
        message.timestamp = Date.now() * 1000

        activeVehicle.sendMessage(message)
        console.log("Sent FIRE_COMMAND: Drone=" + droneId + ", Enable=" + enable)
    }

    function sendTargetAssignment(droneId, targetId, lat, lon, priority) {
        if (!activeVehicle) {
            console.log("No active vehicle")
            return
        }

        // TARGET_ASSIGNMENT (ID: 12921) 전송
        var message = activeVehicle.createMAVLinkMessage(12921)
        message.drone_id = droneId
        message.target_id = targetId
        message.lat = lat * 1e7
        message.lon = lon * 1e7
        message.alt = 0  // 고도는 0으로 설정 (지상 화재)
        message.priority = priority
        message.timestamp = Date.now() * 1000

        activeVehicle.sendMessage(message)
        console.log("Sent TARGET_ASSIGNMENT: Drone=" + droneId + ", Target=" + targetId)
    }

    // 테스트 데이터 (개발용)
    Component.onCompleted: {
        // 테스트 드론 데이터 추가
        formationStatus.updateDroneStatus(1, 37.5665, 126.9780, 87, 6, 3, 1, true)
        formationStatus.updateDroneStatus(2, 37.5670, 126.9785, 92, 6, 4, 2, false)
        formationStatus.updateDroneStatus(3, 37.5668, 126.9782, 28, 3, 5, 3, false)

        // 테스트 화재 지점 추가
        firePointPanel.addFirePoint(1, 37.5672, 126.9788, 0.8)
        firePointPanel.addFirePoint(2, 37.5675, 126.9790, 0.5)
        firePointPanel.addFirePoint(3, 37.5670, 126.9785, 0.9)

        // 화재 지점 진행 상황
        firePointPanel.updateFirePointProgress(1, 1, 1)  // IN_PROGRESS
        firePointPanel.updateFirePointProgress(2, 2, 2)  // COMPLETED

        // 격발 제어 상태
        fireControlPanel.updateDroneFireState(1, 3, false)  // NAVIGATING
        fireControlPanel.updateDroneFireState(2, 5, false)  // FIRE_READY
        fireControlPanel.updateDroneFireState(3, 5, false)  // FIRE_READY
    }
}
