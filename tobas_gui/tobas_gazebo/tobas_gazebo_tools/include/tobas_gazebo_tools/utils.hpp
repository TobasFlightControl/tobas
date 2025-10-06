#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace gazebo
{
/* エンティティ直下のコンポーネントを取得する．存在しない場合は新規作成する． */
template <typename CompType>
CompType* getComponent(const gz::sim::Entity& entity, gz::sim::EntityComponentManager& ecm)
{
  if (ecm.EntityHasComponentType(entity, CompType().TypeId())) {
    return ecm.Component<CompType>(entity);
  }
  else {
    return ecm.CreateComponent(entity, CompType());
  }
}

bool belongsTo(const gz::sim::Entity& entity, const gz::sim::Entity& target, const gz::sim::EntityComponentManager& ecm);

std::optional<gz::sim::Entity>
findJointWithChildLink(const gz::sim::EntityComponentManager& ecm, const std::string& link_name);
}  // namespace gazebo
