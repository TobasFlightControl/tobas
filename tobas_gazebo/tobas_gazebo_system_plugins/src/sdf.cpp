#include "tobas_gazebo_system_plugins/sdf.hpp"

#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

using namespace std;

namespace gazebo
{
bool getTurningDirection(const sdf::ElementConstPtr& sdf, int& dst)
{
  static constexpr char kDirectionKey[] = "turningDirection";

  if (!sdf->HasElement(kDirectionKey)) {
    return false;
  }
  const auto direction_text = sdf->Get<string>(kDirectionKey);

  tobas::turning_direction_t direction_enum;
  if (!tobas::enumFromText(direction_text, direction_enum)) {
    return false;
  }

  dst = tobas::sign(direction_enum);
  return true;
}
}  // namespace gazebo
