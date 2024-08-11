#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace gazebo
{
/* エンティティ直下のコンポーネントを取得する．存在しない場合は新規作成する． */
template <typename CompType>
CompType* getComponent(const gz::sim::Entity& entity, gz::sim::EntityComponentManager& ecm)
{
  if (ecm.EntityHasComponentType(entity, CompType().TypeId()))
    return ecm.Component<CompType>(entity);
  else
    return ecm.CreateComponent(entity, CompType());
}
}  // namespace gazebo
