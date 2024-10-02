#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <tobas_linux/realtime.hpp>

#define NUM_MANAGERS 4

struct ComponentManager
{
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
  std::shared_ptr<rclcpp_components::ComponentManager> node;
  std::thread thread;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::ExecutorOptions exec_options;
  const auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);

  std::array<ComponentManager, NUM_MANAGERS> managers;

  for (size_t i = 0; i < managers.size(); ++i)
  {
    managers[i].exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    const auto name = "component_manager_" + std::to_string(i + 1);
    managers[i].node = std::make_shared<rclcpp_components::ComponentManager>(managers[i].exec, name, node_options);

    // スレッドプール内のスレッド数を設定 (未指定ならばCPUのコア数と同じ)
    if (managers[i].node->has_parameter("thread_num"))
    {
      const auto thread_num = managers[i].node->get_parameter("thread_num").as_int();
      managers[i].exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_options, thread_num);
      managers[i].node->set_executor(managers[i].exec);
    }
    else
    {
      managers[i].node->set_executor(managers[i].exec);
    }

    managers[i].exec->add_node(managers[i].node);

    managers[i].thread = std::thread([&]() { managers[i].exec->spin(); });

    // スレッドのリアルタイム優先度を設定
    const auto priority = 90 - 10 * i;
    if (!linux::setThreadPriority(managers[i].thread.native_handle(), priority, SCHED_FIFO))
      RCLCPP_WARN(managers[i].node->get_logger(), "Failed to set realtime thread priority.");

    // TODO: Set CPU affinity?
  }

  for (auto& manager : managers)
    manager.thread.join();

  rclcpp::shutdown();
}
