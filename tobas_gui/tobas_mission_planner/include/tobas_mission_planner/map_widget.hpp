#pragma once

#include <QtQuickWidgets/QQuickWidget>

#include "./map_items/map_items.hpp"

namespace gui
{
namespace mission_planner
{
/**
 * @brief Open Street Mapを埋め込んだウィジェット．
 * cf. https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
 */
class MapWidget : public QQuickWidget
{
  Q_OBJECT

  using self = MapWidget;
  using super = QQuickWidget;

Q_SIGNALS:
  void waypointMoved(int index, double latitude, double longitude);

public:
  explicit MapWidget();

  void clear();

  void addWaypoint(int index, const QGeoCoordinate& coord, double acceptance_radius, const QString& marker_color);
  void addLine(double latitude_1, double longitude_1, double latitude_2, double longitude_2);

  std::pair<double, double> getCenter();
  void setCenter(double latitude, double longitude);

private:
  map::WaypointModel* waypoint_;
  map::LineModel* line_;

private Q_SLOTS:
  void onWaypointMoved(int index, double latitude, double longitude);
};
}  // namespace mission_planner
}  // namespace gui
