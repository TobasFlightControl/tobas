pragma ComponentBehavior: Bound

import QtQuick
import QtLocation
import QtPositioning
import "./map_constants.js" as Constants

// Qt Location: https://doc.qt.io/qt-6/qtlocation-index.html
Rectangle {
  id: rectangle

  property real requested_zoom: 0 // Zoom requested by the user; exceeding the limit is allowed.
  property real visual_scale: 1.0 // Visual scale for overzoom.

  // Signal for function calls.
  signal setArrowPosition(double latitude, double longitude)
  signal setArrowRotation(double angle)
  signal setMapCenter(double latitude, double longitude)
  signal setZoomLevel(double zoom_level)

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

  function onSetZoomLevel(zoom_level) {
    requested_zoom = rectangle.clamp(zoom_level, map.minimumZoomLevel, Constants.maximumZoomLevel);
    rectangle.updateZoom();
  }

  function updateZoom() {
    var base_z = Math.min(requested_zoom, map.maximumZoomLevel);
    map.zoomLevel = base_z;
    var extra = Math.max(0, requested_zoom - base_z); // Amount over the limit.
    visual_scale = Math.pow(2, extra);
    map.scale = visual_scale;
  }

  Component.onCompleted: {
    setMapCenter.connect(onSetMapCenter);
    setZoomLevel.connect(onSetZoomLevel);
    setArrowPosition.connect(onSetArrowPosition);
    setArrowRotation.connect(onSetArrowRotation);
  }

  // Qt Location Open Street Map Plugin: https://doc.qt.io/qt-6/location-plugin-osm.html
  Plugin {
    id: mapPlugin

    name: "osm" // itemsoverlay, mapbox, here, esri, osm

    PluginParameter {
      name: "osm.mapping.custom.host"
      value: "http://127.0.0.1:8080/tiles/" // Specify the local server.
    }

    PluginParameter {
      name: "osm.mapping.cache.directory"
      value: SystemInfo.homeDirectory + "/.cache/tobas/tiles/online/" // qmllint disable unqualified
    }

    PluginParameter {
      name: "osm.mapping.offline.directory"
      value: SystemInfo.homeDirectory + "/.cache/tobas/tiles/offline/" // qmllint disable unqualified
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

  // Assign the write-once plugin after QQuickWidget attaches the map to its QQuickWindow.
  Timer {
    interval: 0
    repeat: false
    running: true

    onTriggered: {
      map.plugin = mapPlugin;
      console.debug("Map plugin set:", mapPlugin.name);
    }
  }

  // Map QML Type: https://doc.qt.io/qt-6/qml-qtlocation-map.html
  Map {
    id: map

    anchors.fill: parent
    center: QtPositioning.coordinate(Constants.defaultLatitude, Constants.defaultLongitude)
    copyrightsVisible: false
    maximumZoomLevel: 22 // Adjust to the tile server; too large is not a problem.
    minimumZoomLevel: 3 // Set this to the maximum value that shows the whole map.
    objectName: "map" // Object name for access from Qt.
    zoomLevel: 3

    Component.onCompleted: {
      rectangle.requested_zoom = map.zoomLevel;
      rectangle.updateZoom();
    }
    onSupportedMapTypesChanged: {
      if (supportedMapTypes.length > 0) {
        // The custom tile server is always the last supported map type.
        const custom_map_type = supportedMapTypes[supportedMapTypes.length - 1];
        if (activeMapType !== custom_map_type) {
          activeMapType = custom_map_type;
          console.debug("Active map type set:", activeMapType.name);
        }
      }
    }

    DragHandler {
      target: null

      onTranslationChanged: delta => {
        // Pan opposite to the drag delta so the map content follows the pointer, compensating for visual scaling.
        map.pan(-delta.x / map.scale, -delta.y / map.scale);
      }
    }

    PinchHandler {
      id: pinch

      property var anchorCoordinate

      grabPermissions: PointerHandler.TakeOverForbidden
      target: null

      onActiveChanged: {
        if (active) {
          // Save the geographic coordinate under the finger midpoint when the pinch starts.
          anchorCoordinate = map.toCoordinate(centroid.position);
        }
      }
      onScaleChanged: delta => {
        // Convert the incremental scale factor to a zoom-level change (2x means +1), then clamp the requested zoom.
        const next_zoom = rectangle.requested_zoom + Math.log2(delta);
        rectangle.requested_zoom = rectangle.clamp(next_zoom, map.minimumZoomLevel, Constants.maximumZoomLevel);
        // Update the map zoom and apply visual scaling beyond the map's maximum zoom level.
        rectangle.updateZoom();
        // Keep the saved geographic coordinate under the current finger midpoint while zooming.
        map.alignCoordinateToPoint(anchorCoordinate, centroid.position);
      }
    }

    WheelHandler {
      acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
      target: null

      onWheel: e => {
        // Get the cursor position in 2D coordinates.
        const p = point.position;
        // Compute the cursor position in geographic coordinates.
        const anchor = map.toCoordinate(p);
        // Touchpad scrolling may provide pixel deltas without wheel angle deltas.
        const delta_y = e.angleDelta.y !== 0 ? e.angleDelta.y : e.pixelDelta.y;
        // Compute the zoom value change.
        const delta_zoom = (delta_y / 120.0) * 0.5;
        // Update the target zoom value.
        const next_zoom = rectangle.requested_zoom + delta_zoom;
        rectangle.requested_zoom = rectangle.clamp(next_zoom, map.minimumZoomLevel, Constants.maximumZoomLevel);
        // Update zoom and scale.
        rectangle.updateZoom();
        // Align the original geographic coordinate to the new cursor position.
        map.alignCoordinateToPoint(anchor, p);
        // Accept the wheel event.
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

        height: 32 / rectangle.mapObjectScale()
        source: "./arrow.png" // Relative path to the icon image.
        width: 32 / rectangle.mapObjectScale()

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
      model: WaypointModel // qmllint disable unqualified
      z: 3 // Priority when overlapping other objects.

      // `Rectangle` cannot be set as a delegate, so use `MapQuickItem`.
      delegate: MapQuickItem {
        id: waypoint

        required property var model

        anchorPoint.x: circle.width / 2
        anchorPoint.y: circle.height / 2
        coordinate: waypoint.model.coordinate

        sourceItem: Rectangle {
          id: circle

          border.color: "black"
          border.pixelAligned: false // Allow fractional border widths.
          border.width: 2 / rectangle.mapObjectScale()
          color: waypoint.model.marker_color
          height: 32 / rectangle.mapObjectScale()
          radius: 16 / rectangle.mapObjectScale() // Setting the radius to half the square side length makes a circle from a square.
          width: 32 / rectangle.mapObjectScale()
          x: 0
          y: 0

          // Show the number at the center of the circle.
          Text {
            color: "black"
            font.pixelSize: 16 / rectangle.mapObjectScale() // Alignment breaks during overzoom unless this is 2 or larger.
            text: waypoint.model.index
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
              const offset_x = circle.x;
              const offset_y = circle.y;

              // Apply the child-object movement to the parent object.
              const old_coord = waypoint.coordinate;
              const old_point = map.fromCoordinate(old_coord);
              const new_x = old_point.x + circle.x;
              const new_y = old_point.y + circle.y;
              const new_coord = map.toCoordinate(Qt.point(new_x, new_y));
              waypoint.coordinate = new_coord;

              // Reset the child-object offset.
              circle.x = 0;
              circle.y = 0;

              // Notify that the waypoint coordinate has changed.
              rectangle.waypointMoved(waypoint.model.index, new_coord.latitude, new_coord.longitude);
            }
          }
        }
      }
    }

    MapItemView {
      model: WaypointModel // qmllint disable unqualified
      z: 2

      delegate: MapCircle {
        id: acceptanceCircle

        required property var model

        border.color: "yellow"
        border.width: 2 / rectangle.mapObjectScale()
        center: acceptanceCircle.model.coordinate
        color: "transparent"
        radius: acceptanceCircle.model.acceptance_radius // [m]
      }
    }

    // LineModel
    MapItemView {
      model: LineModel // qmllint disable unqualified
      z: 1

      delegate: MapPolyline {
        id: missionLine

        required property var model

        line.color: "green"
        line.width: 3 / rectangle.mapObjectScale()
        path: [
          {
            "latitude": missionLine.model.latitude_1,
            "longitude": missionLine.model.longitude_1
          },
          {
            "latitude": missionLine.model.latitude_2,
            "longitude": missionLine.model.longitude_2
          },
        ]
      }
    }
  }
}
