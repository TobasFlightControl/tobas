#pragma once

#include <QtQuickWidgets/QQuickWidget>

#include "./items/line.hpp"
#include "./items/waypoint.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
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

  QGeoCoordinate getCenter() const;
  QGeoCoordinate getArrowPosition() const;
  double getArrowRotation() const;  // [deg]

  void setMapCenter(double latitude, double longitude);
  void setArrowPosition(double latitude, double longitude);
  void setArrowRotation(double angle_deg);

private:
  map::WaypointModel* waypoint_;
  map::LineModel* line_;

  QObject* getMapObject() const;
  QObject* getArrowObject() const;
  QObject* getArrowRotationObject() const;

private Q_SLOTS:
  void onWaypointMoved(int index, double latitude, double longitude);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
