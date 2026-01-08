import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// 미션 상태 모니터링 패널
Rectangle {
    id: missionStatusPanel
    width: 300
    height: 500
    color: "#2C2C2C"
    radius: 8
    border.color: "#4A90E2"
    border.width: 2

    property int phase: 0  // FIRE_MISSION_PHASE (0-6)
    property int progress: 0  // Progress 0-100%
    property int remaining_projectiles: 0
    property real distance_to_target: 0.0
    property real thermal_max_temp: 0.0
    property string status_text: ""

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // 헤더
        Rectangle {
            Layout.fillWidth: true
            height: 40
            color: "#3A3A3A"
            radius: 4

            Text {
                anchors.centerIn: parent
                text: "미션 상태 모니터링"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 미션 상태 정보
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#3A3A3A"
            radius: 4
            border.color: getPhaseColor(phase)
            border.width: 2

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 15
                spacing: 10

                // 미션 단계
                RowLayout {
                    Layout.fillWidth: true

                    Text {
                        text: "미션 단계:"
                        font.pixelSize: 14
                        color: "#CCCCCC"
                    }

                    Item { Layout.fillWidth: true }

                    Text {
                        text: getPhaseName(phase)
                        font.pixelSize: 14
                        font.bold: true
                        color: getPhaseColor(phase)
                    }
                }

                // 진행률
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 5

                    RowLayout {
                        Layout.fillWidth: true

                        Text {
                            text: "진행률:"
                            font.pixelSize: 12
                            color: "#CCCCCC"
                        }

                        Item { Layout.fillWidth: true }

                        Text {
                            text: progress + "%"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#4A90E2"
                        }
                    }

                    // 진행률 바
                    Rectangle {
                        Layout.fillWidth: true
                        height: 20
                        color: "#1E1E1E"
                        radius: 10
                        border.color: "#666666"
                        border.width: 1

                        Rectangle {
                            anchors.left: parent.left
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            anchors.margins: 2
                            width: (parent.width - 4) * (progress / 100.0)
                            color: getProgressColor(progress)
                            radius: 8
                        }
                    }
                }

                // 남은 소화탄
                RowLayout {
                    Layout.fillWidth: true

                    Text {
                        text: "남은 소화탄:"
                        font.pixelSize: 12
                        color: "#CCCCCC"
                    }

                    Item { Layout.fillWidth: true }

                    Text {
                        text: remaining_projectiles + " 개"
                        font.pixelSize: 12
                        font.bold: true
                        color: remaining_projectiles > 0 ? "#00FF00" : "#FF0000"
                    }
                }

                // 목표까지 거리
                RowLayout {
                    Layout.fillWidth: true

                    Text {
                        text: "목표까지 거리:"
                        font.pixelSize: 12
                        color: "#CCCCCC"
                    }

                    Item { Layout.fillWidth: true }

                    Text {
                        text: distance_to_target.toFixed(1) + " m"
                        font.pixelSize: 12
                        color: "#4A90E2"
                    }
                }

                // 열화상 최고 온도
                RowLayout {
                    Layout.fillWidth: true

                    Text {
                        text: "열화상 최고 온도:"
                        font.pixelSize: 12
                        color: "#CCCCCC"
                    }

                    Item { Layout.fillWidth: true }

                    Text {
                        text: thermal_max_temp.toFixed(1) + " °C"
                        font.pixelSize: 12
                        font.bold: true
                        color: getTemperatureColor(thermal_max_temp)
                    }
                }

                // 상태 메시지
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 60
                    color: "#1E1E1E"
                    radius: 4
                    border.color: "#666666"
                    border.width: 1

                    ScrollView {
                        anchors.fill: parent
                        anchors.margins: 5

                        Text {
                            text: status_text || "대기 중..."
                            font.pixelSize: 11
                            color: "#CCCCCC"
                            wrapMode: Text.Wrap
                        }
                    }
                }
            }
        }

        // 범례
        Rectangle {
            Layout.fillWidth: true
            height: 30
            color: "#3A3A3A"
            radius: 4

            RowLayout {
                anchors.centerIn: parent
                spacing: 15

                Text { text: "IDLE"; font.pixelSize: 10; color: "#CCCCCC" }
                Text { text: "NAV"; font.pixelSize: 10; color: "#4A90E2" }
                Text { text: "READY"; font.pixelSize: 10; color: "#00FF00" }
                Text { text: "FIRING"; font.pixelSize: 10; color: "#FF5722" }
            }
        }
    }

    // 헬퍼 함수들
    function getPhaseName(phase) {
        var names = [
            "IDLE",              // 0
            "NAVIGATING",        // 1
            "SCANNING",          // 2
            "READY_TO_FIRE",     // 3
            "SUPPRESSING",       // 4
            "VERIFYING",         // 5
            "COMPLETE"           // 6
        ]
        return names[phase] || "UNKNOWN"
    }

    function getPhaseColor(phase) {
        var colors = [
            "#CCCCCC",  // IDLE
            "#4A90E2",  // NAVIGATING
            "#FFAA00",  // SCANNING
            "#00FF00",  // READY_TO_FIRE
            "#FF5722",  // SUPPRESSING
            "#9C27B0",  // VERIFYING
            "#4CAF50"   // COMPLETE
        ]
        return colors[phase] || "#CCCCCC"
    }

    function getProgressColor(progress) {
        if (progress < 30) return "#FF0000"
        if (progress < 70) return "#FFAA00"
        return "#00FF00"
    }

    function getTemperatureColor(temp) {
        if (temp < 50) return "#00FF00"  // 낮음
        if (temp < 100) return "#FFAA00"  // 중간
        return "#FF0000"  // 높음
    }

    // MAVLink 메시지 핸들러
    function updateMissionStatus(newPhase, newProgress, newRemaining, newDistance, newTemp, newStatusText) {
        phase = newPhase
        progress = newProgress
        remaining_projectiles = newRemaining
        distance_to_target = newDistance
        thermal_max_temp = newTemp
        status_text = newStatusText
    }
}
