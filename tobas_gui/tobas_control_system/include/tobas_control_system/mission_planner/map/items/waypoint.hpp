#pragma once

#include <QtPositioning/QGeoCoordinate>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
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
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
