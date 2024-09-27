#include "tobas_control_system/map_items/line.hpp"

namespace gui
{
namespace control_system
{
namespace map
{
QString LineModel::modelName() const
{
  return "LineModel";
}

QByteArrayList LineModel::argNames() const
{
  return { "latitude_1", "longitude_1", "latitude_2", "longitude_2" };
}
}  // namespace map
}  // namespace control_system
}  // namespace gui
