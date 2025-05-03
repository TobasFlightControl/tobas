#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QtQuick/QQuickItem>
#include <QtQml/QQmlContext>

#include <tobas_std_tools/check.hpp>

#include "tobas_control_system/constants.hpp"
#include "tobas_control_system/mission_planner/map_widget.hpp"
#include "tobas_control_system/mission_planner/system_info.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace gcs
{
MapWidget::MapWidget()
{
  // サイズポリシーとリサイズモードの指定 (しないとウィジェットが潰れてしまう)
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setResizeMode(QQuickWidget::SizeRootObjectToView);

  // モデルオブジェクトの設定
  // QMLファイルのロード前に行う必要がある
  waypoint_ = new map::WaypointModel();
  line_ = new map::LineModel();
  rootContext()->setContextProperty(waypoint_->modelName(), waypoint_);
  rootContext()->setContextProperty(line_->modelName(), line_);

  // コンストラクタ引数を設定
  const auto system_info = new SystemInfo();
  rootContext()->setContextProperty(system_info->modelName(), system_info);

  // QMLを読み込む
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPkgName));
  const auto qml_path = pkg_path / "qml/Map.qml";
  setSource(QUrl::fromLocalFile(QString::fromStdString(qml_path)));

  // Connection
  connect(rootObject(), SIGNAL(waypointMoved(int, double, double)), this, SLOT(onWaypointMoved(int, double, double)));
}

void MapWidget::clear()
{
  waypoint_->clear();
  line_->clear();
}

void MapWidget::addWaypoint(int index, const QGeoCoordinate& coord, double acceptance_radius, const QString& marker_color)
{
  waypoint_->add(index, coord, acceptance_radius, marker_color);
}

void MapWidget::addLine(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
{
  line_->add(latitude_1, longitude_1, latitude_2, longitude_2);
}

QGeoCoordinate MapWidget::getCenter() const
{
  return getMapObject()->property("center").value<QGeoCoordinate>();
}

QGeoCoordinate MapWidget::getArrowPosition() const
{
  return getArrowObject()->property("coordinate").value<QGeoCoordinate>();
}

double MapWidget::getArrowRotation() const
{
  return getArrowRotationObject()->property("angle").value<double>();
}

void MapWidget::setMapCenter(double latitude, double longitude)
{
  QMetaObject::invokeMethod(rootObject(), "setMapCenter", Q_ARG(double, latitude), Q_ARG(double, longitude));
}

void MapWidget::setArrowPosition(double latitude, double longitude)
{
  QMetaObject::invokeMethod(rootObject(), "setArrowPosition", Q_ARG(double, latitude), Q_ARG(double, longitude));
}

void MapWidget::setArrowRotation(double angle_deg)
{
  QMetaObject::invokeMethod(rootObject(), "setArrowRotation", Q_ARG(double, angle_deg));
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

void MapWidget::onWaypointMoved(int index, double latitude, double longitude)
{
  Q_EMIT waypointMoved(index, latitude, longitude);
}
}  // namespace gcs
}  // namespace gui
