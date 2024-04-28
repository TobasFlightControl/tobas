import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Rectangle {
  id: rectangle
  height: 500  // QMLに定義できる値はなるべくQMLに定義

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
    objectName: "map"  // Python側からアクセスするためのオブジェクト名
    anchors.fill: parent
    plugin: osmPlugin
    center: QtPositioning.coordinate(35., 150.)  // 日本で一般的に販売されている世界地図の中心座標
    zoomLevel: 0  // 最小

    MapItemView {
      model: MarkerModel
      delegate: MapQuickItem {
        id: markerItem
        coordinate: model.position
        anchorPoint.x: markerImage.width
        anchorPoint.y: markerImage.height
        sourceItem: Image {
          id: markerImage
          source: model.source
        }
        MouseArea {
          id: markerMouseArea
          anchors.fill: parent
          drag.target: parent
          onReleased: {
            markerDropped()
          }
        }
      }
    }

    MapItemView {
      model: LineModel
      delegate: MapPolyline {
        line.width: 3
        line.color:"green"
        path: [
        {latitude: model.latitude_1, longitude: model.longitude_1},
        {latitude: model.latitude_2, longitude: model.longitude_2},
        ]
      }
    }
  }

  // イベント通知用シグナル
  signal markerDropped()

  // 関数呼び出し用シグナル
  signal setCenter(double latitude, double longitude)

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
}
