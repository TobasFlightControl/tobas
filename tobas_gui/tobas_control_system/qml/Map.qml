import QtQuick 2.15
import QtLocation 5.15
import QtPositioning 5.15

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

  Map {
    id: map
    objectName: "map"  // Python側からアクセスするためのオブジェクト名
    anchors.fill: parent
    plugin: osmPlugin
    center: QtPositioning.coordinate(35., 150.)  // 日本で一般的に販売されている世界地図の中心座標
    zoomLevel: 0  // 最小

    // WaypointModel
    MapItemView {
      model: WaypointModel

      // Rectangleはdelegateに設定できないため，MapQuickItemを使う
      delegate: MapQuickItem {
        id: wayPoint
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
              let old_coord = wayPoint.coordinate;
              let old_point = map.fromCoordinate(old_coord);
              let new_x = old_point.x + circle.x;
              let new_y = old_point.y + circle.y;
              let new_coord = map.toCoordinate(Qt.point(new_x, new_y));
              wayPoint.coordinate = new_coord;

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

  Component.onCompleted: {
    setCenter.connect(onSetCenter);
  }

  function onSetCenter(latitude, longitude)
  {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
}
