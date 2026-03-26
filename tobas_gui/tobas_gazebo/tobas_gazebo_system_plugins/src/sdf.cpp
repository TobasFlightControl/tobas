#include "tobas_gazebo_system_plugins/sdf.hpp"

#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

namespace tobas
{
namespace gazebo
{
bool getTurningDirection(const sdf::ElementConstPtr& sdf, int& dst)
{
  static constexpr char kDirectionKey[] = "turningDirection";

  if (!sdf->HasElement(kDirectionKey)) {
    return false;
  }
  const auto direction_text = sdf->Get<std::string>(kDirectionKey);

  TurningDirection direction_enum;
  if (!enumFromText(direction_text, direction_enum)) {
    return false;
  }

  dst = sign(direction_enum);
  return true;
}
}  // namespace gazebo
}  // namespace tobas
