#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <tobas_linux/process_settings.hpp>

int main(int argc, char* argv[])
{
  linux::ProcessSettings settings;
  if (!settings.init(argc, argv))
    return EXIT_FAILURE;

  rclcpp::init(argc, argv);

  if (!settings.configureProcess())
    RCLCPP_WARN(rclcpp::get_logger("component_container"), "Failed to configure process.");

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}
