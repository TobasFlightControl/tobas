#include "tobas_components_rt/multi_component_managers.hpp"

int main(int argc, char* argv[])
{
  constexpr size_t kNumManagers = 3;

  // ROSノードを起動．
  rclcpp::init(argc, argv);

  // Ctrl+Cで即終了．
  signal(SIGINT, [](int) { rclcpp::shutdown(); });

  // 複数のComponentManagerをシングルプロセスで動作させる．
  ros2::MultiComponentManagers managers(kNumManagers);

  for (size_t i = 0; i < kNumManagers; ++i) {
    // 厳密に優先度を守るポリシーに設定．
    managers.setPolicy(i, SCHED_FIFO);

    // IOのIRQのデフォルト値50以上のIRQ値にすると，CPUを固定した場合にデッドロックする恐れがあるため，それ未満に設定する．
    // https://docs.redhat.com/ja/documentation/red_hat_enterprise_linux/9/html/monitoring_and_managing_system_status_and_performance/priority-map_tuning-scheduling-policy
    managers.setPriority(i, 49 - i);

    // ComponentManagerごとにCPUを専有する．
    managers.setCpuAffinity(i, 1 << (i + 1));

    // MultiThreadedExecutorはCPU負荷が高く，パフォーマンス向上のため1つのCPUに1つのスレッドのみを割り当てる．
    managers.setNumThreads(i, 1);
  }

  managers.spin();

  rclcpp::shutdown();
}
