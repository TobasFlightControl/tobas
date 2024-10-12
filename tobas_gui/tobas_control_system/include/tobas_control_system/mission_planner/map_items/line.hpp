#pragma once

#include <QtPositioning/QGeoCoordinate>

#include "./base.hpp"

namespace gui
{
namespace control_system
{
namespace map
{
class LineModel : public MapItemModel<double, double, double, double>
{
public:
  QString modelName() const override;
  QByteArrayList argNames() const override;
};
}  // namespace map
}  // namespace control_system
}  // namespace gui
