import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Rectangle {
  id: rectangle
  Plugin {
    id: osmPlugin
    name: "osm"
    PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: "true" }
  }
  property variant locationTC: QtPositioning.coordinate(35.6580992222, 139.7413574722)
  Map {
    id: map
    anchors.fill: parent
    plugin: osmPlugin
    center: locationTC
    zoomLevel: 18  // 0 ~ 20
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
}
