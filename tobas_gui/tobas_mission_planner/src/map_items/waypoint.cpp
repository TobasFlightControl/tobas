#include "tobas_mission_planner/map_items/waypoint.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace mission_planner
}  // namespace gui
