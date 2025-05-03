#include "../include/tobas_gazebo_tools/utils.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
bool belongsTo(const gz::sim::Entity& entity, const gz::sim::Entity& target, const gz::sim::EntityComponentManager& ecm)
{
  const auto parent = ecm.ParentEntity(entity);
  if (parent == gz::sim::kNullEntity) {
    return false;
  }
  else if (parent == target) {
    return true;
  }
  else {
    return belongsTo(parent, target, ecm);
  }
}

optional<gz::sim::Entity> findJointWithChildLink(const gz::sim::EntityComponentManager& ecm, const string& link_name)
{
  optional<gz::sim::Entity> res;

  ecm.Each<cmp::Joint, cmp::ChildLinkName>(
    [&](const gz::sim::Entity& entity, const cmp::Joint*, const cmp::ChildLinkName* child_link_name) -> bool
    {
      if (child_link_name->Data() == link_name) {
        res = entity;
        return false;
      }
      return true;
    });

  return res;
}
}  // namespace gazebo
