import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// 진압 결과 및 목표 관리 패널
Rectangle {
    id: firePointPanel
    width: 300
    height: 400
    color: "#2C2C2C"
    radius: 8
    border.color: "#FF5722"
    border.width: 2

    property var firePointList: []
    property var suppressionResults: []  // FIRE_SUPPRESSION_RESULT 메시지 결과

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
                text: "목표 및 진압 결과"
                font.pixelSize: 18
                font.bold: true
                color: "#FFFFFF"
            }
        }

        // 탭 바
        TabBar {
            id: tabBar
            Layout.fillWidth: true

            TabButton {
                text: "목표 지점"
            }

            TabButton {
                text: "진압 결과"
            }
        }

        // 스택 뷰
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex

            // 목표 지점 탭
            ScrollView {
                ListView {
                    id: firePointListView
                    model: firePointList
                    spacing: 8

                    delegate: Rectangle {
                        width: firePointListView.width - 20
                        height: 100
                        color: "#3A3A3A"
                        radius: 4
                        border.color: "#FF5722"
                        border.width: 1

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 5

                            // 목표 ID 및 삭제 버튼
                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    text: "Target #" + modelData.id + " 🔥"
                                    font.pixelSize: 14
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

                            // 미션 시작 버튼
                            Button {
                                Layout.fillWidth: true
                                text: "미션 시작"
                                font.pixelSize: 11
                                enabled: true

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
                                    startMission(modelData.id, modelData.lat, modelData.lon)
                                }
                            }
                        }
                    }
                }
            }

            // 진압 결과 탭
            ScrollView {
                ListView {
                    id: resultListView
                    model: suppressionResults
                    spacing: 8

                    delegate: Rectangle {
                        width: resultListView.width - 20
                        height: 120
                        color: "#3A3A3A"
                        radius: 4
                        border.color: modelData.success ? "#00FF00" : "#FF0000"
                        border.width: 2

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 5

                            // 발사 번호 및 성공 여부
                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    text: "발사 #" + modelData.shot_number
                                    font.pixelSize: 14
                                    font.bold: true
                                    color: "#FFFFFF"
                                }

                                Item { Layout.fillWidth: true }

                                Text {
                                    text: modelData.success ? "✓ 성공" : "✗ 실패"
                                    font.pixelSize: 12
                                    font.bold: true
                                    color: modelData.success ? "#00FF00" : "#FF0000"
                                }
                            }

                            // 온도 변화
                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    text: "온도 변화:"
                                    font.pixelSize: 11
                                    color: "#CCCCCC"
                                }

                                Text {
                                    text: modelData.temp_before.toFixed(1) + "°C"
                                    font.pixelSize: 11
                                    color: "#FF5722"
                                }

                                Text {
                                    text: "→"
                                    font.pixelSize: 11
                                    color: "#CCCCCC"
                                }

                                Text {
                                    text: modelData.temp_after.toFixed(1) + "°C"
                                    font.pixelSize: 11
                                    color: modelData.success ? "#00FF00" : "#FF5722"
                                }

                                Item { Layout.fillWidth: true }

                                Text {
                                    text: "Δ " + (modelData.temp_before - modelData.temp_after).toFixed(1) + "°C"
                                    font.pixelSize: 11
                                    font.bold: true
                                    color: modelData.success ? "#00FF00" : "#FF0000"
                                }
                            }

                            // 온도 변화 바
                            Rectangle {
                                Layout.fillWidth: true
                                height: 15
                                color: "#1E1E1E"
                                radius: 7

                                Rectangle {
                                    anchors.left: parent.left
                                    anchors.top: parent.top
                                    anchors.bottom: parent.bottom
                                    anchors.margins: 2
                                    width: (parent.width - 4) * (modelData.temp_after / modelData.temp_before)
                                    color: modelData.success ? "#00FF00" : "#FF0000"
                                    radius: 5
                                }
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
            text: "+ 목표 지점 추가"
            font.pixelSize: 14

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
                // QGC 지도에서 클릭하여 추가하도록 신호 전송
                addFirePointClicked()
            }
        }
    }

    // 공개 함수
    function addFirePoint(id, lat, lon, priority) {
        firePointList.push({
            id: id,
            lat: lat,
            lon: lon,
            priority: priority || 0.5
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

    function updateSuppressionResult(shotNumber, tempBefore, tempAfter, success) {
        // 기존 결과 찾기
        for (var i = 0; i < suppressionResults.length; i++) {
            if (suppressionResults[i].shot_number === shotNumber) {
                suppressionResults[i] = {
                    shot_number: shotNumber,
                    temp_before: tempBefore,
                    temp_after: tempAfter,
                    success: success
                }
                resultListView.model = suppressionResults
                return
            }
        }

        // 새 결과 추가
        suppressionResults.push({
            shot_number: shotNumber,
            temp_before: tempBefore,
            temp_after: tempAfter,
            success: success
        })
        resultListView.model = suppressionResults
    }

    function startMission(targetId, lat, lon) {
        // FIRE_MISSION_START 메시지 전송 신호
        missionStartRequested(targetId, lat, lon)
    }

    // 시그널
    signal addFirePointClicked()
    signal missionStartRequested(int targetId, real lat, real lon)
}
