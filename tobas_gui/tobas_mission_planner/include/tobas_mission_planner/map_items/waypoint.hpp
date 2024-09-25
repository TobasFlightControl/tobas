#pragma once

#include <QtPositioning/QGeoCoordinate>

#include "./base.hpp"

namespace gui
{
namespace mission_planner
{
namespace map
{
class WaypointModel : public MapItemModel<int, QGeoCoordinate, double, QString>
{
public:
  QString modelName() const override;
  QByteArrayList argNames() const override;
};
}  // namespace map
}  // namespace mission_planner
}  // namespace gui
