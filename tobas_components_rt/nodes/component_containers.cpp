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

  ros2::MultiComponentManagers managers(4);

  // FIXME: CPU Affinityを設定するとsystemdから起動したときに不具合が出る

  // MultiThreadedExecutorはCPU不可が高く，むしろ遅くなるため，最高優先度の処理もシングルスレッドで動作させる．
  // cf. https://github.com/ros2/rclpy/issues/1223
  managers.setPolicy(0, SCHED_FIFO);
  managers.setPriority(0, 90);
  // managers.setCPUAffinity(0, (1 << 0));
  managers.setNumThreads(0, 1);

  managers.setPolicy(1, SCHED_FIFO);
  managers.setPriority(1, 50);
  // managers.setCPUAffinity(1, (1 << 1));
  managers.setNumThreads(1, 1);

  managers.setPolicy(2, SCHED_FIFO);
  managers.setPriority(2, 10);
  // managers.setCPUAffinity(2, (1 << 2));
  managers.setNumThreads(2, 1);

  managers.setPolicy(3, SCHED_FIFO);
  managers.setPriority(3, 0);
  // managers.setCPUAffinity(3, (1 << 3));
  managers.setNumThreads(3, 1);

  managers.spin();

  rclcpp::shutdown();
}
