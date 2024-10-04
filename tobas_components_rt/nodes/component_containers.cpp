#include "tobas_components_rt/multi_component_managers.hpp"

#define SCHED_POLICY SCHED_FIFO
#define NUM_MANAGERS 4
#define NUM_THREADS 0

static void sigIntHandler(int)
{
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  ros2::MultiComponentManagers managers(NUM_MANAGERS);

  // FIXME: CPU Affinityを設定するとsystemdから起動したときにCPU0,1が使用されなくなる

  managers.setPolicy(0, SCHED_FIFO);
  managers.setPriority(0, 90);
  // managers.setCPUAffinity(0, (1 << 0) | (1 << 1));
  managers.setNumThreads(0, 0);

  managers.setPolicy(1, SCHED_FIFO);
  managers.setPriority(1, 50);
  // managers.setCPUAffinity(1, (1 << 2));
  managers.setNumThreads(1, 0);

  managers.setPolicy(2, SCHED_FIFO);
  managers.setPriority(2, 10);
  // managers.setCPUAffinity(2, (1 << 3));
  managers.setNumThreads(2, 0);

  managers.setPolicy(3, SCHED_FIFO);
  managers.setPriority(3, 0);
  // managers.setCPUAffinity(3, (1 << 3));
  managers.setNumThreads(3, 0);

  managers.spin();

  rclcpp::shutdown();
}
