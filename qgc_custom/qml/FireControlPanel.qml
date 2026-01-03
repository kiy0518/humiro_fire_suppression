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

    property var droneList: []

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
                text: "Fire Control"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 드론별 격발 컨트롤
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true

            ListView {
                id: fireControlListView
                model: droneList
                spacing: 8

                delegate: Rectangle {
                    width: fireControlListView.width - 20
                    height: 60
                    color: "#3A3A3A"
                    radius: 4
                    border.color: modelData.isFiring ? "#FF5722" : "#666666"
                    border.width: 2

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 10

                        // 드론 ID
                        Text {
                            text: "Drone " + modelData.id
                            font.pixelSize: 14
                            font.bold: true
                            color: "#FFFFFF"
                            Layout.preferredWidth: 80
                        }

                        // 상태 표시
                        Text {
                            text: getFireState(modelData.state)
                            font.pixelSize: 12
                            color: getFireStateColor(modelData.state)
                            Layout.fillWidth: true
                        }

                        // 격발 버튼
                        Button {
                            text: modelData.isFiring ? "STOP" : "FIRE"
                            font.pixelSize: 12
                            font.bold: true
                            Layout.preferredWidth: 80
                            Layout.preferredHeight: 40

                            background: Rectangle {
                                color: modelData.isFiring ? "#F44336" : "#4CAF50"
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
                                if (modelData.isFiring) {
                                    stopFire(modelData.id)
                                } else {
                                    startFire(modelData.id)
                                }
                            }

                            enabled: canFire(modelData.state)
                        }
                    }
                }
            }
        }

        // 전체 제어 버튼
        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Button {
                text: "Fire All"
                font.pixelSize: 14
                Layout.fillWidth: true
                Layout.preferredHeight: 50

                background: Rectangle {
                    color: "#4CAF50"
                    radius: 4
                }

                contentItem: Text {
                    text: parent.text
                    font: parent.font
                    color: "#FFFFFF"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: fireAllDrones()
            }

            Button {
                text: "Stop All"
                font.pixelSize: 14
                Layout.fillWidth: true
                Layout.preferredHeight: 50

                background: Rectangle {
                    color: "#F44336"
                    radius: 4
                }

                contentItem: Text {
                    text: parent.text
                    font: parent.font
                    color: "#FFFFFF"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: stopAllDrones()
            }
        }
    }

    // 헬퍼 함수
    function getFireState(state) {
        if (state === 5) return "READY"
        if (state === 6) return "FIRING..."
        if (state === 9) return "COMPLETE"
        return "NOT_READY"
    }

    function getFireStateColor(state) {
        if (state === 5) return "#00FF00"  // FIRE_READY
        if (state === 6) return "#FF5722"  // FIRING
        if (state === 9) return "#4A90E2"  // COMPLETED
        return "#CCCCCC"
    }

    function canFire(state) {
        return state === 5 || state === 6  // FIRE_READY or FIRING
    }

    // 공개 함수
    function updateDroneFireState(droneId, state, isFiring) {
        for (var i = 0; i < droneList.length; i++) {
            if (droneList[i].id === droneId) {
                droneList[i].state = state
                droneList[i].isFiring = isFiring
                fireControlListView.model = droneList
                return
            }
        }

        // 새 드론 추가
        droneList.push({
            id: droneId,
            state: state,
            isFiring: isFiring
        })
        fireControlListView.model = droneList
    }

    function startFire(droneId) {
        sendFireCommand(droneId, true)
    }

    function stopFire(droneId) {
        sendFireCommand(droneId, false)
    }

    function fireAllDrones() {
        for (var i = 0; i < droneList.length; i++) {
            if (canFire(droneList[i].state)) {
                sendFireCommand(droneList[i].id, true)
            }
        }
    }

    function stopAllDrones() {
        for (var i = 0; i < droneList.length; i++) {
            sendFireCommand(droneList[i].id, false)
        }
    }

    // MAVLink 메시지 전송 (QGC에서 구현)
    function sendFireCommand(droneId, enable) {
        console.log("Send FIRE_COMMAND: Drone=" + droneId + ", Enable=" + enable)
        // QGC MAVLink 메시지 전송
        fireCommandSent(droneId, enable)
    }

    // 시그널
    signal fireCommandSent(int droneId, bool enable)
}
