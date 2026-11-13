// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/map/map.hpp"

#include <cmath>

#include <QSettings>
#include <QtQml/QQmlContext>
#include <QtQuick/QQuickItem>

#include <tobas_std_tools/check.hpp>

#include "tobas_control_system/mission_planner/system_info.hpp"
#include "tobas_control_system/util.hpp"

namespace tobas::gui::ctrl
{
namespace
{
constexpr char kSettingsGroup[] = "mission_planner/map";
constexpr char kLatitudeKey[] = "latitude";
constexpr char kLongitudeKey[] = "longitude";
constexpr char kZoomLevelKey[] = "zoom_level";
}  // namespace

MapWidget::MapWidget()
{
  // Set the size policy and resize mode; otherwise the widget collapses.
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setResizeMode(QQuickWidget::SizeRootObjectToView);

  // Set model objects.
  // This must be done before loading the QML file.
  waypoint_ = new map::WaypointModel();
  line_ = new map::LineModel();
  rootContext()->setContextProperty(waypoint_->modelName(), waypoint_);
  rootContext()->setContextProperty(line_->modelName(), line_);

  // Set constructor arguments.
  const auto system_info = new SystemInfo();
  rootContext()->setContextProperty(system_info->modelName(), system_info);

  // Load QML.
  const auto qml_path = getPkgShareDir() / "qml/Map.qml";
  setSource(QUrl::fromLocalFile(QString::fromStdString(qml_path)));

  // Connection
  connect(rootObject(), SIGNAL(waypointMoved(int, double, double)), this, SLOT(onWaypointMoved(int, double, double)));

  restoreView();
}

MapWidget::~MapWidget()
{
  saveView();
}

void MapWidget::clear()
{
  waypoint_->clear();
  line_->clear();
}

void MapWidget::addWaypoint(
  int _index,
  const QGeoCoordinate& _coord,
  double _acceptance_radius,
  const QString& _marker_color)
{
  waypoint_->add(_index, _coord, _acceptance_radius, _marker_color);
}

void MapWidget::addLine(double _latitude_1, double _longitude_1, double _latitude_2, double _longitude_2)
{
  line_->add(_latitude_1, _longitude_1, _latitude_2, _longitude_2);
}

QGeoCoordinate MapWidget::getCenter() const
{
  return getMapObject()->property("center").value<QGeoCoordinate>();
}

double MapWidget::getZoomLevel() const
{
  return rootObject()->property("requested_zoom").toDouble();
}

QGeoCoordinate MapWidget::getArrowPosition() const
{
  return getArrowObject()->property("coordinate").value<QGeoCoordinate>();
}

double MapWidget::getArrowRotation() const
{
  return getArrowRotationObject()->property("angle").value<double>();
}

void MapWidget::setMapCenter(double _latitude, double _longitude)
{
  QMetaObject::invokeMethod(rootObject(), "setMapCenter", Q_ARG(double, _latitude), Q_ARG(double, _longitude));
}

void MapWidget::setZoomLevel(double _zoom_level)
{
  QMetaObject::invokeMethod(rootObject(), "setZoomLevel", Q_ARG(double, _zoom_level));
}

void MapWidget::setArrowPosition(double _latitude, double _longitude)
{
  QMetaObject::invokeMethod(rootObject(), "setArrowPosition", Q_ARG(double, _latitude), Q_ARG(double, _longitude));
}

void MapWidget::setArrowRotation(double _angle_deg)
{
  QMetaObject::invokeMethod(rootObject(), "setArrowRotation", Q_ARG(double, _angle_deg));
}

void MapWidget::restoreView()
{
  QSettings settings;
  settings.beginGroup(kSettingsGroup);

  const auto current_center = getCenter();
  const auto latitude = settings.value(kLatitudeKey, current_center.latitude()).toDouble();
  const auto longitude = settings.value(kLongitudeKey, current_center.longitude()).toDouble();
  const QGeoCoordinate stored_center(latitude, longitude);
  if (stored_center.isValid()) {
    setMapCenter(latitude, longitude);
  }

  const auto zoom_level = settings.value(kZoomLevelKey, getZoomLevel()).toDouble();
  if (std::isfinite(zoom_level)) {
    setZoomLevel(zoom_level);
  }
}

void MapWidget::saveView() const
{
  QSettings settings;
  settings.beginGroup(kSettingsGroup);

  const auto center = getCenter();
  if (center.isValid()) {
    settings.setValue(kLatitudeKey, center.latitude());
    settings.setValue(kLongitudeKey, center.longitude());
  }

  const auto zoom_level = getZoomLevel();
  if (std::isfinite(zoom_level)) {
    settings.setValue(kZoomLevelKey, zoom_level);
  }

  settings.sync();
}

QObject* MapWidget::getMapObject() const
{
  const auto res = rootObject()->findChild<QObject*>("map");
  TOBAS_CHECK(res);
  return res;
}

QObject* MapWidget::getArrowObject() const
{
  const auto res = rootObject()->findChild<QObject*>("arrow");
  TOBAS_CHECK(res);
  return res;
}

QObject* MapWidget::getArrowRotationObject() const
{
  const auto res = rootObject()->findChild<QObject*>("arrowRotation");
  TOBAS_CHECK(res);
  return res;
}

void MapWidget::onWaypointMoved(int _index, double _latitude, double _longitude)
{
  Q_EMIT waypointMoved(_index, _latitude, _longitude);
}
}  // namespace tobas::gui::ctrl
