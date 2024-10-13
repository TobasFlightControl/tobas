#include "tobas_components_rt/multi_component_managers.hpp"

static void sigIntHandler(int)
{
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  ros2::MultiComponentManagers managers(3);

  // MultiThreadedExecutorはCPU不可が高く，SingleThreadedExecutorよりもレイテンシが大きくなる．
  // しかし，シングルスレッドだとサービスコールやDDS再接続でブロッキングが発生する．
  // そのため，リアルタイム性が重視されるグループには最低でも2コア2スレッド与える必要がある．

  managers.setPolicy(0, SCHED_FIFO);
  managers.setPriority(0, 90);
  managers.setCPUAffinity(0, 0b0011);
  managers.setNumThreads(0, 2);

  managers.setPolicy(1, SCHED_FIFO);
  managers.setPriority(1, 50);
  managers.setCPUAffinity(1, 0b1100);
  managers.setNumThreads(1, 2);

  managers.setPolicy(2, SCHED_FIFO);
  managers.setPriority(2, 10);
  managers.setCPUAffinity(2, 0b1100);
  managers.setNumThreads(2, 2);

  managers.spin();

  rclcpp::shutdown();
}
