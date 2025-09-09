#include "tobas_components_rt/multi_component_managers.hpp"

static void sigIntHandler(int)
{
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  // ROSノードを起動
  rclcpp::init(argc, argv);

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // 複数のComponentManagerをシングルプロセスで動作させる．
  // MultiThreadedExecutorはCPU不可が高く，パフォーマンス向上のため1つのCPUに1つのスレッドのみを割り当てるのが重要．
  ros2::MultiComponentManagers managers(3);

  managers.setPolicy(0, SCHED_FIFO);
  managers.setPriority(0, 98);
  managers.setCPUAffinity(0, 0b0010);
  managers.setNumThreads(0, 1);

  managers.setPolicy(1, SCHED_FIFO);
  managers.setPriority(1, 97);
  managers.setCPUAffinity(1, 0b0100);
  managers.setNumThreads(1, 1);

  managers.setPolicy(2, SCHED_FIFO);
  managers.setPriority(2, 96);
  managers.setCPUAffinity(2, 0b1000);
  managers.setNumThreads(2, 1);

  managers.spin();

  rclcpp::shutdown();
}
