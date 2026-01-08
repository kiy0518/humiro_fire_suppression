import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// 격발 제어 패널
Rectangle {
    id: fireControlPanel
    width: 300
    height: 300
    color: "#2C2C2C"
    radius: 8
    border.color: "#F44336"
    border.width: 2

    property int phase: 0  // FIRE_MISSION_PHASE (0-6)
    property bool isFiring: false  // SUPPRESSING phase인지 여부

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
                text: "격발 제어"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 현재 상태 표시
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            color: "#3A3A3A"
            radius: 4
            border.color: getPhaseColor(phase)
            border.width: 2

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 5

                Text {
                    text: "현재 상태:"
                    font.pixelSize: 12
                    color: "#CCCCCC"
                }

                Text {
                    text: getPhaseName(phase)
                    font.pixelSize: 16
                    font.bold: true
                    color: getPhaseColor(phase)
                }
            }
        }

        // 격발 제어 버튼
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            // 확인 버튼 (CONFIRM)
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 50
                text: "발사 확인"
                font.pixelSize: 14
                font.bold: true
                enabled: canConfirm()

                background: Rectangle {
                    color: parent.enabled ? "#4CAF50" : "#666666"
                    radius: 4
                }

                contentItem: Text {
                    text: parent.text
                    font: parent.font
                    color: "#FFFFFF"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: {
                    sendFireCommand(true)  // CONFIRM (command=0)
                }
            }

            // 중단 버튼 (ABORT)
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 50
                text: "발사 중단"
                font.pixelSize: 14
                font.bold: true
                enabled: canAbort()

                background: Rectangle {
                    color: parent.enabled ? "#F44336" : "#666666"
                    radius: 4
                }

                contentItem: Text {
                    text: parent.text
                    font: parent.font
                    color: "#FFFFFF"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: {
                    sendFireCommand(false)  // ABORT (command=1)
                }
            }

            // 상태 요청 버튼 (REQUEST_STATUS)
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                text: "상태 요청"
                font.pixelSize: 12

                background: Rectangle {
                    color: "#4A90E2"
                    radius: 4
                }

                contentItem: Text {
                    text: parent.text
                    font: parent.font
                    color: "#FFFFFF"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: {
                    requestStatus()  // REQUEST_STATUS (command=2)
                }
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

    function canConfirm() {
        // READY_TO_FIRE (3) 또는 SUPPRESSING (4) 단계에서만 확인 가능
        return phase === 3 || phase === 4
    }

    function canAbort() {
        // SUPPRESSING (4) 또는 VERIFYING (5) 단계에서만 중단 가능
        return phase === 4 || phase === 5
    }

    // 공개 함수
    function updateFireState(newPhase, newIsFiring) {
        phase = newPhase
        isFiring = newIsFiring
    }

    function sendFireCommand(confirm) {
        // confirm=true: CONFIRM (command=0)
        // confirm=false: ABORT (command=1)
        fireCommandSent(confirm)
    }

    function requestStatus() {
        // REQUEST_STATUS (command=2) 전송
        statusRequested()
    }

    // 시그널
    signal fireCommandSent(bool confirm)  // true=CONFIRM, false=ABORT
    signal statusRequested()  // REQUEST_STATUS
}
