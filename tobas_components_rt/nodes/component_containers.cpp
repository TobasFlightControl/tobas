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
  // SCHED_FIFOよりもSCHED_RRの方がROSと相性が良いらしい．
  // なぜだか分からないが，最悪時間はSCHED_RRの方が小さいように思える．

  managers.setPolicy(0, SCHED_FIFO);
  managers.setPriority(0, 98);
  managers.setCPUAffinity(0, 0b0001);
  managers.setNumThreads(0, 1);

  managers.setPolicy(1, SCHED_FIFO);
  managers.setPriority(1, 97);
  managers.setCPUAffinity(1, 0b0010);
  managers.setNumThreads(1, 1);

  managers.setPolicy(2, SCHED_FIFO);
  managers.setPriority(2, 96);
  managers.setCPUAffinity(2, 0b0100);
  managers.setNumThreads(2, 1);

  managers.spin();

  rclcpp::shutdown();
}
