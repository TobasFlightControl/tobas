import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Rectangle {
  id: rectangle
  height: 640  // QMLに定義できる値はなるべくQMLに定義

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
    center: QtPositioning.coordinate(35., 150.)  // 日本で一般的に販売されている世界地図の中心座標
    zoomLevel: 0  // 最小

    MapItemView {
      model: MarkerModel
      delegate: MapQuickItem {
        id: markerItem
        coordinate: model.position_marker
        anchorPoint.x: markerImage.width
        anchorPoint.y: markerImage.height
        sourceItem: Image {
          id: markerImage
          source: model.source_marker
        }
        MouseArea {
          id: markerMouseArea
          anchors.fill: parent
          drag.target: parent
          onReleased: {
            markerDropped
            var coord = map.toCoordinate(Qt.point(markerItem.coordinate));
            markerItem.coordinate = coord;
          }
        }
      }
    }
  }

  // イベント通知用シグナル
  signal markerDropped(double latitude, double longitude)

  // 関数呼び出し用シグナル
  // エラーを防ぐために，関数の呼び出しには必ずシグナルスロット接続を挟む．
  signal setCenter(double latitude, double longitude)

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
}
