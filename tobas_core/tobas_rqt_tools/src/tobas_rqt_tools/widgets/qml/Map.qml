import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Rectangle {
  id: rectangle

  Plugin {
    id: osmPlugin
    name: "osm"
    PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: "true" }  // これが必須
  }

  Map {
    id: map
    anchors.fill: parent
    plugin: osmPlugin
    center: QtPositioning.coordinate(35.6580992222, 139.7413574722)  // 日本経緯度原点
    zoomLevel: 10  // 0 ~ 20

    MapItemView {
      model: markermodel
      delegate: MapQuickItem {
        coordinate: model.position_marker
        anchorPoint.x: image.width
        anchorPoint.y: image.height
        sourceItem:
        Image { id: image; source: model.source_marker }
      }
    }
  }

  // エラーを防ぐために，関数の呼び出しには必ずシグナルスロット接続を挟む．
  signal setCenter(double latitude, double longitude)
  signal setZoomLevel(int level)

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
    setZoomLevel.connect(onSetZoomLevel);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }

  function onSetZoomLevel(level)
  {
    map.zoomLevel = level;
  }
}
