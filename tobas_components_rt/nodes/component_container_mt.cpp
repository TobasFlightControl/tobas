#include "../include/tobas_components_rt/common.hpp"

int main(int argc, char* argv[])
{
  if (!init(argc, argv))
    return EXIT_FAILURE;

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
