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
    RCLCPP_WARN(rclcpp::get_logger("component_container_mt"), "Failed to configure process.");

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>();
  if (node->has_parameter("thread_num"))
  {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  }
  else
  {
    node->set_executor(exec);
  }
  exec->add_node(node);
  exec->spin();
}
