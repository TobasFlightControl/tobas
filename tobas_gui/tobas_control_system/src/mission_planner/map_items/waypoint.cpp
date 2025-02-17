#include "tobas_control_system/mission_planner/map_items/waypoint.hpp"

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
