#include "../include/tobas_components_rt/common.hpp"

int main(int argc, char* argv[])
{
  if (!init(argc, argv))
    return EXIT_FAILURE;

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}
