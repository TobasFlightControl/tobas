#include "../include/tobas_gazebo_tools/utils.hpp"

using namespace std;
using namespace gz::sim;

namespace gazebo
{
bool belongsTo(const Entity& entity, const Entity& target, const EntityComponentManager& ecm)
{
  const auto parent = ecm.ParentEntity(entity);
  if (parent == kNullEntity)
    return false;
  else if (parent == target)
    return true;
  else
    return belongsTo(parent, target, ecm);
}
}  // namespace gazebo
