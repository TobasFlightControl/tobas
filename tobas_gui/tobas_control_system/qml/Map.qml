import QtQuick 2.15
import QtLocation 5.15
import QtPositioning 5.15
import "./map_constants.js" as Constants

// Qt Location: https://doc.qt.io/archives/qt-5.15/qtlocation-index.html
Rectangle {
  id: rectangle

  property real requested_zoom: 0 // ユーザが要求しているズーム（上限超えOK）
  property real visual_scale: 1.0 // overzoom分の見た目スケール

  // 関数呼び出し用シグナル
  signal setArrowPosition(double latitude, double longitude)
  signal setArrowRotation(double angle)
  signal setMapCenter(double latitude, double longitude)

  // イベント通知用シグナル
  signal waypointMoved(int index, double latitude, double longitude)

  function clamp(v, lb, ub) {
    return Math.min(ub, Math.max(lb, v));
  }
  function mapObjectScale() {
    return Math.min(map.scale, Constants.maxMapObjectScale);
  }
  function onSetArrowPosition(latitude, longitude) {
    arrow.coordinate = QtPositioning.coordinate(latitude, longitude);
  }
  function onSetArrowRotation(angle) {
    arrowRotation.angle = angle; // ユニークなIDを直接参照する
  }
  function onSetMapCenter(latitude, longitude) {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
  function updateZoom() {
    var base_z = Math.min(requested_zoom, map.maximumZoomLevel);
    map.zoomLevel = base_z;
    var extra = Math.max(0, requested_zoom - base_z); // 上限超え分
    visual_scale = Math.pow(2, extra);
    map.scale = visual_scale;
  }

  Component.onCompleted: {
    console.log("Available map service providers:", mapPlugin.availableServiceProviders);
    setMapCenter.connect(onSetMapCenter);
    setArrowPosition.connect(onSetArrowPosition);
    setArrowRotation.connect(onSetArrowRotation);
  }

  // Qt Location Open Street Map Plugin: https://doc.qt.io/archives/qt-5.15/location-plugin-osm.html
  Plugin {
    id: mapPlugin
    name: "osm" // itemsoverlay, mapbox, here, esri, osm

    PluginParameter {
      name: "osm.mapping.custom.host"
      value: "http://127.0.0.1:8080/tiles/" // ローカルサーバを指定
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
      value: 1 << 30 // 1GiB
    }
  }

  // Map QML Type: https://doc.qt.io/qt-5/qml-qtlocation-map.html
  Map {
    id: map
    activeMapType: map.supportedMapTypes[map.supportedMapTypes.length - 1] // タイルサーバを指定する場合に必要
    anchors.fill: parent
    center: QtPositioning.coordinate(Constants.defaultLatitude, Constants.defaultLongitude)
    copyrightsVisible: false
    maximumZoomLevel: 22 // タイルサーバに合わせて調整する (大きすぎるのは問題ない)
    minimumZoomLevel: 3 // 地図全体が見える最大値に設定
    objectName: "map" // Qt側からアクセスするためのオブジェクト名
    plugin: mapPlugin
    zoomLevel: 3

    Component.onCompleted: {
      requested_zoom = zoomLevel;
      updateZoom();
    }

    // ホイールイベントでスケールを調整しながらズーム
    WheelHandler {
      target: null

      onWheel: e => {
        const p = point.position; // カーソル位置 (2次元座標)
        const anchor = map.toCoordinate(p); // カーソル位置 (地理座標)
        const dz = (e.angleDelta.y / 120.0) * 0.5; // ズーム値の変化量
        requested_zoom = clamp(requested_zoom + dz, map.minimumZoomLevel, Constants.maximumZoomLevel); // ズーム値の目標値を更新
        // console.log("Zoom Level:", requested_zoom);
        updateZoom(); // ズームとスケールを更新
        map.alignCoordinateToPoint(anchor, p); // 元々の地理座標を新しいカーソル位置に合わせる
        e.accepted = true;
      }
    }

    // Arrow
    MapQuickItem {
      id: arrow
      anchorPoint.x: arrowImage.width / 2
      anchorPoint.y: arrowImage.height / 2
      coordinate: QtPositioning.coordinate(0, 0)
      objectName: "arrow"

      sourceItem: Image {
        id: arrowImage
        height: 32 / mapObjectScale()
        source: "./arrow.png" // アイコン画像の相対パス
        width: 32 / mapObjectScale()

        transform: Rotation {
          id: arrowRotation
          angle: 0 // Clock-wise [degree]
          objectName: "arrowRotation"
          origin.x: arrowImage.width / 2
          origin.y: arrowImage.height / 2

          axis {
            x: 0
            y: 0
            z: 1
          }
        }
      }
    }

    // WaypointModel
    MapItemView {
      model: WaypointModel

      // Rectangleはdelegateに設定できないため，MapQuickItemを使う
      delegate: MapQuickItem {
        id: waypoint
        anchorPoint.x: circle.width / 2
        anchorPoint.y: circle.height / 2
        coordinate: model.coordinate

        sourceItem: Rectangle {
          id: circle
          border.color: "black"
          border.pixelAligned: false // 小数値の枠線幅を許容
          border.width: 2 / mapObjectScale()
          color: model.marker_color
          height: 32 / mapObjectScale()
          radius: 16 / mapObjectScale() // 半径を正方形の辺長の半分に設定することで，正方形から円を作ることができる
          width: 32 / mapObjectScale()
          x: 0
          y: 0

          // 円の中心に番号を表示
          Text {
            color: "black"
            font.pixelSize: 16 / mapObjectScale() // 2以上じゃないとオーバーズームした際にアラインメントが崩れる
            text: model.index
            x: (circle.width - width) / 2
            y: (circle.height - height) / 2
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
      model: WaypointModel // 1つのモデルに対して複数のMapItemViewを定義できる

      delegate: MapCircle {
        border.color: "yellow"
        border.width: 2 / mapObjectScale()
        center: model.coordinate
        color: "transparent"
        radius: model.acceptance_radius // [m]
      }
    }

    // LineModel
    MapItemView {
      model: LineModel

      delegate: MapPolyline {
        line.color: "green"
        line.width: 3 / mapObjectScale()
        path: [{
            "latitude": model.latitude_1,
            "longitude": model.longitude_1
          }, {
            "latitude": model.latitude_2,
            "longitude": model.longitude_2
          },]
      }
    }
  }
}
