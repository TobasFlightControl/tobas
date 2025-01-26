import QtQuick 2.15
import QtLocation 5.15
import QtPositioning 5.15
import "./map_constants.js" as Constants

Rectangle {
  id: rectangle

  Plugin {
    id: osmPlugin
    name: "osm"

    // OSMプラグインのパラメータはTobasの全てのMapオブジェクトで同一にする
    PluginParameter {
      name: "osm.mapping.providersrepository.disabled"
      value: "true"
    }
    PluginParameter {
      name: "osm.mapping.cache.directory"
      value: SystemInfo.homeDirectory + "/.cache/tobas/tiles/online/"
    }
    PluginParameter {
      name: "osm.mapping.offline.directory"
      value: SystemInfo.homeDirectory + "/.cache/tobas/tiles/offline/"
    }
    PluginParameter {
      name: "osm.mapping.cache.disk.cost_strategy"
      value: "bytesize"
    }
    PluginParameter {
      name: "osm.mapping.cache.disk.size"
      value: 1 << 30  // 1GiB
    }
  }

  // Map QML Type: https://doc.qt.io/qt-5/qml-qtlocation-map.html#supportedMapTypes-prop
  Map {
    id: map
    anchors.fill: parent
    center: QtPositioning.coordinate(Constants.defaultLatitude, Constants.defaultLongitude)
    copyrightsVisible: false
    plugin: osmPlugin
    zoomLevel: 0  // 最小

    // GPS Arrow
    MapQuickItem {
      id: gpsArrow
      coordinate: QtPositioning.coordinate(0, 0)
      sourceItem: Image {
        id: gpsArrowImage
        source: "./gps_arrow.png" // アイコン画像の相対パス
        width: 30
        height: 30
        transform: Rotation {
          id: gpsArrowRotation
          origin.x: gpsArrowImage.width / 2
          origin.y: gpsArrowImage.height / 2
          axis { x: 0; y: 0; z: 1 }
          angle: 0 // Clock-wise [degree]
        }
      }
    }

    // WaypointModel
    MapItemView {
      model: WaypointModel

      // Rectangleはdelegateに設定できないため，MapQuickItemを使う
      delegate: MapQuickItem {
        id: waypoint
        coordinate: model.coordinate
        anchorPoint.x: circle.width / 2
        anchorPoint.y: circle.height / 2

        sourceItem: Rectangle {
          id: circle
          width: 30
          height: 30
          radius: 15  // 半径を正方形の辺長の半分に設定することで，正方形から円を作ることができる
          color: model.marker_color
          border.color: "black"
          border.width: 2

          // 親オブジェクトに対する相対座標
          x: 0
          y: 0

          // 円の中心に番号を表示
          Text {
            anchors.centerIn: parent
            text: model.index
            color: "black"
            font.pixelSize: 16
          }

          // 円をドラッグ・アンド・ドロップできるようにするための設定
          MouseArea {
            anchors.fill: parent
            drag.target: parent
            onReleased: {
              // ドラッグ・アンド・ドロップによって発生した，親オブジェクトに対する子オブジェクトの移動量
              let offset_x = circle.x;
              let offset_y = circle.y;

              // 子オブジェクトの移動分を親オブジェクトに反映させる
              let old_coord = waypoint.coordinate;
              let old_point = map.fromCoordinate(old_coord);
              let new_x = old_point.x + circle.x;
              let new_y = old_point.y + circle.y;
              let new_coord = map.toCoordinate(Qt.point(new_x, new_y));
              waypoint.coordinate = new_coord;

              // 子オブジェクトのオフセットをリセット
              circle.x = 0;
              circle.y = 0;

              waypointMoved(model.index, new_coord.latitude, new_coord.longitude);
            }
          }
        }
      }
    }

    MapItemView {
      model: WaypointModel  // 1つのモデルに対して複数のMapItemViewを定義できる

      delegate: MapCircle {
        center: model.coordinate
        radius: model.acceptance_radius  // [m]
        color: "transparent"
        border.color: "yellow"
        border.width: 2
      }
    }

    // LineModel
    MapItemView {
      model: LineModel
      delegate: MapPolyline {
        line.width: 3
        line.color: "green"
        path: [
        {latitude: model.latitude_1, longitude: model.longitude_1},
        {latitude: model.latitude_2, longitude: model.longitude_2},
        ]
      }
    }
  }

  // イベント通知用シグナル
  signal waypointMoved(int index, double latitude, double longitude)

  // 関数呼び出し用シグナル
  signal setCenter(double latitude, double longitude)
  signal setGPSArrowPosition(double latitude, double longitude)
  signal setGPSArrowRotation(double angle)

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
    setGPSArrowPosition.connect(onSetGPSArrowPosition);
    setGPSArrowRotation.connect(onSetGPSArrowRotation);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }

  function onSetGPSArrowPosition(latitude, longitude)
  {
    gpsArrow.coordinate = QtPositioning.coordinate(latitude, longitude);
  }

  function onSetGPSArrowRotation(angle)
  {
    gpsArrowRotation.angle = angle;  // ユニークなIDを直接参照する
  }
}
