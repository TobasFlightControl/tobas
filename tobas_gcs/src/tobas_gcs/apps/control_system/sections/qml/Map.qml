import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Rectangle {
  id: rectangle

  Plugin {
    id: osmPlugin
    name: "osm"
    PluginParameter {
      name: "osm.mapping.providersrepository.disabled"
      value: "true"
    }
  }

  Map {
    id: map
    anchors.fill: parent
    plugin: osmPlugin
    center: QtPositioning.coordinate(35.6580992222, 139.7413574722)  // 日本経緯度原点
    zoomLevel: 18  // 0 ~ 20

    MapQuickItem {
      coordinate: map.center // 中央に配置
      sourceItem: Image {
        id: iconImage
        source: "gps_arrow.png" // アイコン画像のパス
        width: 32
        height: 32
        transform: Rotation {
          id: iconImageRotation
          origin.x: iconImage.width / 2
          origin.y: iconImage.height / 2
          axis { x: 0; y: 0; z: 1 }
          angle: 0
        }
      }
    }
  }

  // 関数呼び出し用シグナル
  // エラーを防ぐために，関数の呼び出しには必ずシグナルスロット接続を挟む．
  signal setCenter(double latitude, double longitude)
  signal setArrowRotation(double angle)

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
    setArrowRotation.connect(onSetArrowRotation);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }

  function onSetArrowRotation(angle)
  {
    iconImageRotation.angle = angle;  // ユニークなIDを直接参照する
  }
}
