import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// 화재 지점 관리 패널
Rectangle {
    id: firePointPanel
    width: 300
    height: 400
    color: "#2C2C2C"
    radius: 8
    border.color: "#FF5722"
    border.width: 2

    property var firePointList: []

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
                text: "Fire Point Management"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 화재 지점 리스트
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true

            ListView {
                id: firePointListView
                model: firePointList
                spacing: 8

                delegate: Rectangle {
                    width: firePointListView.width - 20
                    height: 120
                    color: "#3A3A3A"
                    radius: 4
                    border.color: "#FF5722"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 5

                        // 목표 ID
                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "Target #" + modelData.id + " 🔥"
                                font.pixelSize: 16
                                font.bold: true
                                color: "#FF5722"
                            }

                            Item { Layout.fillWidth: true }

                            Button {
                                text: "Delete"
                                font.pixelSize: 10
                                onClicked: deleteFirePoint(modelData.id)
                            }
                        }

                        // 위치
                        Text {
                            text: "Position: " + modelData.lat.toFixed(6) + "N, " +
                                  modelData.lon.toFixed(6) + "E"
                            font.pixelSize: 11
                            color: "#CCCCCC"
                        }

                        // 우선순위
                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "Priority: "
                                font.pixelSize: 12
                                color: "#CCCCCC"
                            }

                            Text {
                                text: getPriorityName(modelData.priority)
                                font.pixelSize: 12
                                color: getPriorityColor(modelData.priority)
                            }
                        }

                        // 할당 및 상태
                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "Assigned: " + (modelData.assignedDrone > 0 ? "Drone " + modelData.assignedDrone : "None")
                                font.pixelSize: 11
                                color: "#4A90E2"
                            }

                            Item { Layout.fillWidth: true }

                            Text {
                                text: getProgressName(modelData.progress)
                                font.pixelSize: 11
                                color: getProgressColor(modelData.progress)
                            }
                        }
                    }
                }
            }
        }

        // 추가 버튼
        Button {
            Layout.fillWidth: true
            height: 40
            text: "+ Add Fire Point"
            font.pixelSize: 14

            onClicked: {
                // QGC 지도에서 클릭하여 추가하도록 신호 전송
                addFirePointClicked()
            }
        }
    }

    // 헬퍼 함수
    function getPriorityName(priority) {
        if (priority >= 0.7) return "HIGH"
        if (priority >= 0.4) return "MEDIUM"
        return "LOW"
    }

    function getPriorityColor(priority) {
        if (priority >= 0.7) return "#FF0000"
        if (priority >= 0.4) return "#FFAA00"
        return "#00FF00"
    }

    function getProgressName(progress) {
        var names = ["ASSIGNED", "IN_PROGRESS", "COMPLETED", "FAILED"]
        return names[progress] || "UNKNOWN"
    }

    function getProgressColor(progress) {
        var colors = ["#FFAA00", "#4A90E2", "#00FF00", "#FF0000"]
        return colors[progress] || "#CCCCCC"
    }

    // 공개 함수
    function addFirePoint(id, lat, lon, priority) {
        firePointList.push({
            id: id,
            lat: lat,
            lon: lon,
            priority: priority,
            assignedDrone: 0,
            progress: 0  // ASSIGNED
        })
        firePointListView.model = firePointList
    }

    function deleteFirePoint(id) {
        for (var i = 0; i < firePointList.length; i++) {
            if (firePointList[i].id === id) {
                firePointList.splice(i, 1)
                firePointListView.model = firePointList
                return
            }
        }
    }

    function updateFirePointProgress(id, assignedDrone, progress) {
        for (var i = 0; i < firePointList.length; i++) {
            if (firePointList[i].id === id) {
                firePointList[i].assignedDrone = assignedDrone
                firePointList[i].progress = progress
                firePointListView.model = firePointList
                return
            }
        }
    }

    // 시그널
    signal addFirePointClicked()
}
