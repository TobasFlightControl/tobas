// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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
 * Widget embedding OpenStreetMap.
 *
 * @see https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
 */
class MapWidget : public QQuickWidget
{
  Q_OBJECT

Q_SIGNALS:
  void waypointMoved(int _index, double _latitude, double _longitude);

public:
  explicit MapWidget();
  ~MapWidget() override;

  void clear();

  void addWaypoint(int _index, const QGeoCoordinate& _coord, double _acceptance_radius, const QString& _marker_color);
  void addLine(double _latitude_1, double _longitude_1, double _latitude_2, double _longitude_2);

  QGeoCoordinate getCenter() const;
  double getZoomLevel() const;
  QGeoCoordinate getArrowPosition() const;
  double getArrowRotation() const;  // [deg]

  void setMapCenter(double _latitude, double _longitude);
  void setZoomLevel(double _zoom_level);
  void setArrowPosition(double _latitude, double _longitude);
  void setArrowRotation(double _angle_deg);

private:
  map::WaypointModel* waypoint_;
  map::LineModel* line_;

  void restoreView();
  void saveView() const;

  QObject* getMapObject() const;
  QObject* getArrowObject() const;
  QObject* getArrowRotationObject() const;

private Q_SLOTS:
  void onWaypointMoved(int _index, double _latitude, double _longitude);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
