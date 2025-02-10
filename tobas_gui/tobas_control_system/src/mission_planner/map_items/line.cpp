#include "tobas_control_system/mission_planner/map_items/line.hpp"

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
