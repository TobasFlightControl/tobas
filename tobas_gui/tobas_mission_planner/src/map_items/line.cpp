#include "tobas_mission_planner/map_items/line.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace mission_planner
}  // namespace gui
