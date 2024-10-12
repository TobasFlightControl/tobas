#include "tobas_control_system/mission_planner/map_items/waypoint.hpp"

namespace gui
{
namespace control_system
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
}  // namespace control_system
}  // namespace gui
