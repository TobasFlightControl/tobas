#include "tobas_control_system/mission_planner/map/items/waypoint.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace map
{
QString WaypointModel::modelName() const
{
  return "WaypointModel";
}

QByteArrayList WaypointModel::argNames() const
{
  return { "index", "coordinate", "acceptance_radius", "marker_color" };
}
}  // namespace map
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
