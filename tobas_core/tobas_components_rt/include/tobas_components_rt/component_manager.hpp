#pragma once

#include <rclcpp_components/component_manager.hpp>

namespace ros2
{
class ThreadSafeComponentManager : public rclcpp_components::ComponentManager
{
public:
  using ComponentManager::ComponentManager;

  virtual std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource& resource) override;
};
}  // namespace ros2
