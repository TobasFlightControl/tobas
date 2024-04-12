import QtQuick 2.12
import QtLocation 5.12
import QtPositioning 5.12

Map {
  id: map
  anchors.fill: parent
  plugin: Plugin { name: "osm" } // OpenStreetMapプラグインを使用
  center: QtPositioning.coordinate(0., 0.)
  zoomLevel: 15

  function setCenter(latitude, longitude)
  {
    console.debug("Map::setCenter");
    map.center = QtPositioning.coordinate(latitude, longitude);
  }
}
