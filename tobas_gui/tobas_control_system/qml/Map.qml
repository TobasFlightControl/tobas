import QtQuick 2.15
import QtLocation 5.15
import QtPositioning 5.15
import "./map_constants.js" as Constants

// Qt Location: https://doc.qt.io/archives/qt-5.15/qtlocation-index.html
Rectangle {
  id: rectangle

  property real requested_zoom: 0 // Zoom requested by the user; exceeding the limit is allowed.
  property real visual_scale: 1.0 // Visual scale for overzoom.

  // Signal for function calls.
  signal setArrowPosition(double latitude, double longitude)
  signal setArrowRotation(double angle)
  signal setMapCenter(double latitude, double longitude)

  // Signal for event notifications.
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
    arrowRotation.angle = angle; // Reference the unique ID directly.
  }
  function onSetMapCenter(latitude, longitude) {
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
  function updateZoom() {
    var base_z = Math.min(requested_zoom, map.maximumZoomLevel);
    map.zoomLevel = base_z;
    var extra = Math.max(0, requested_zoom - base_z); // Amount over the limit.
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
      value: "http://127.0.0.1:8080/tiles/" // Specify the local server.
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
    activeMapType: map.supportedMapTypes[map.supportedMapTypes.length - 1] // Required when specifying a tile server.
    anchors.fill: parent
    center: QtPositioning.coordinate(Constants.defaultLatitude, Constants.defaultLongitude)
    copyrightsVisible: false
    maximumZoomLevel: 22 // Adjust to the tile server; too large is not a problem.
    minimumZoomLevel: 3 // Set this to the maximum value that shows the whole map.
    objectName: "map" // Object name for access from Qt.
    plugin: mapPlugin
    zoomLevel: 3

    Component.onCompleted: {
      requested_zoom = zoomLevel;
      updateZoom();
    }

    // Zoom while adjusting scale on wheel events.
    WheelHandler {
      target: null

      onWheel: e => {
        const p = point.position; // Cursor position in 2D coordinates.
        const anchor = map.toCoordinate(p); // Cursor position in geographic coordinates.
        const dz = (e.angleDelta.y / 120.0) * 0.5; // Zoom value change.
        requested_zoom = clamp(requested_zoom + dz, map.minimumZoomLevel, Constants.maximumZoomLevel); // Update the target zoom value.
        // console.log("Zoom Level:", requested_zoom);
        updateZoom(); // Update zoom and scale.
        map.alignCoordinateToPoint(anchor, p); // Align the original geographic coordinate to the new cursor position.
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
        source: "./arrow.png" // Relative path to the icon image.
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
      z: 3 // Priority when overlapping other objects.

      // `Rectangle` cannot be set as a delegate, so use `MapQuickItem`.
      delegate: MapQuickItem {
        id: waypoint
        anchorPoint.x: circle.width / 2
        anchorPoint.y: circle.height / 2
        coordinate: model.coordinate

        sourceItem: Rectangle {
          id: circle
          border.color: "black"
          border.pixelAligned: false // Allow fractional border widths.
          border.width: 2 / mapObjectScale()
          color: model.marker_color
          height: 32 / mapObjectScale()
          radius: 16 / mapObjectScale() // Setting the radius to half the square side length makes a circle from a square.
          width: 32 / mapObjectScale()
          x: 0
          y: 0

          // Show the number at the center of the circle.
          Text {
            color: "black"
            font.pixelSize: 16 / mapObjectScale() // Alignment breaks during overzoom unless this is 2 or larger.
            text: model.index
            x: (circle.width - width) / 2
            y: (circle.height - height) / 2
          }

          // Settings that allow the circle to be dragged and dropped.
          MouseArea {
            anchors.fill: parent
            cursorShape: pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor
            drag.smoothed: false // Move the target to the cursor position immediately.
            drag.target: parent
            drag.threshold: 0 // Start moving as soon as dragging begins.

            onReleased: {
              // Child-object movement relative to the parent object caused by drag and drop.
              let offset_x = circle.x;
              let offset_y = circle.y;

              // Apply the child-object movement to the parent object.
              let old_coord = waypoint.coordinate;
              let old_point = map.fromCoordinate(old_coord);
              let new_x = old_point.x + circle.x;
              let new_y = old_point.y + circle.y;
              let new_coord = map.toCoordinate(Qt.point(new_x, new_y));
              waypoint.coordinate = new_coord;

              // Reset the child-object offset.
              circle.x = 0;
              circle.y = 0;

              // Notify that the waypoint coordinate has changed.
              waypointMoved(model.index, new_coord.latitude, new_coord.longitude);
            }
          }
        }
      }
    }
    MapItemView {
      model: WaypointModel // Multiple `MapItemView` instances can be defined for one model.
      z: 2

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
      z: 1

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
