import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// 편대 상태 모니터링 패널
Rectangle {
    id: formationPanel
    width: 300
    height: 500
    color: "#2C2C2C"
    radius: 8
    border.color: "#4A90E2"
    border.width: 2

    property var droneList: []  // MAVLink에서 업데이트될 드론 목록

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
                text: "편대 상태 모니터링"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 드론 리스트
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true

            ListView {
                id: droneListView
                model: droneList
                spacing: 8

                delegate: Rectangle {
                    width: droneListView.width - 20
                    height: 140
                    color: "#3A3A3A"
                    radius: 4
                    border.color: getBorderColor(modelData.battery, modelData.state)
                    border.width: 2

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 5

                        // 드론 ID 및 역할
                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "Drone " + modelData.id
                                font.pixelSize: 16
                                font.bold: true
                                color: "#FFFFFF"
                            }

                            Text {
                                text: modelData.isLeader ? "(Leader)" : ""
                                font.pixelSize: 14
                                color: "#FFD700"
                            }

                            Item { Layout.fillWidth: true }

                            Text {
                                text: getStatusIcon(modelData.battery, modelData.state)
                                font.pixelSize: 20
                            }
                        }

                        // 위치
                        Text {
                            text: "Pos: " + modelData.lat.toFixed(6) + "N, " +
                                  modelData.lon.toFixed(6) + "E"
                            font.pixelSize: 11
                            color: "#CCCCCC"
                        }

                        // 배터리 및 소화탄
                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "Battery: " + modelData.battery + "%"
                                font.pixelSize: 12
                                color: getBatteryColor(modelData.battery)
                            }

                            Item { Layout.fillWidth: true }

                            Text {
                                text: "Ammo: " + modelData.ammo + "/6"
                                font.pixelSize: 12
                                color: modelData.ammo > 0 ? "#00FF00" : "#FF0000"
                            }
                        }

                        // 상태
                        Text {
                            text: "State: " + getStateName(modelData.state)
                            font.pixelSize: 12
                            color: "#4A90E2"
                        }

                        // 할당된 목표
                        Text {
                            text: "Target: " + (modelData.targetId > 0 ? "#" + modelData.targetId : "None")
                            font.pixelSize: 12
                            color: "#FFAA00"
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

                Text { text: "OK"; font.pixelSize: 11; color: "#00FF00" }
                Text { text: "WARN"; font.pixelSize: 11; color: "#FFFF00" }
                Text { text: "ERROR"; font.pixelSize: 11; color: "#FF0000" }
            }
        }
    }

    // 헬퍼 함수들
    function getBorderColor(battery, state) {
        if (state === 10) return "#FF0000"  // FAILED
        if (battery < 20) return "#FF0000"  // 위험
        if (battery < 30) return "#FFFF00"  // 경고
        return "#00FF00"  // 정상
    }

    function getStatusIcon(battery, state) {
        if (state === 10) return "X"
        if (battery < 20) return "X"
        if (battery < 30) return "!"
        return "OK"
    }

    function getBatteryColor(battery) {
        if (battery < 20) return "#FF0000"
        if (battery < 30) return "#FFFF00"
        return "#00FF00"
    }

    function getStateName(state) {
        var states = [
            "IDLE", "ARMING", "TAKEOFF", "NAVIGATING",
            "DEST_REACHED", "FIRE_READY", "FIRING",
            "RETURNING", "LANDING", "COMPLETED", "FAILED"
        ]
        return states[state] || "UNKNOWN"
    }

    // MAVLink 메시지 핸들러 (QGC에서 연결)
    function updateDroneStatus(droneId, lat, lon, battery, ammo, state, targetId, isLeader) {
        // 기존 드론 찾기
        for (var i = 0; i < droneList.length; i++) {
            if (droneList[i].id === droneId) {
                droneList[i] = {
                    id: droneId,
                    lat: lat,
                    lon: lon,
                    battery: battery,
                    ammo: ammo,
                    state: state,
                    targetId: targetId,
                    isLeader: isLeader
                }
                droneListView.model = droneList
                return
            }
        }

        // 새 드론 추가
        droneList.push({
            id: droneId,
            lat: lat,
            lon: lon,
            battery: battery,
            ammo: ammo,
            state: state,
            targetId: targetId,
            isLeader: isLeader
        })
        droneListView.model = droneList
    }
}
