#include <QApplication>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

#include <tobas_setup_assistant/setup_assistant.hpp>

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // Rviz用のノード
  const auto client = std::make_unique<rviz_common::ros_integration::RosClientAbstraction>();
  const auto rviz_node_if = client->init(argc, argv, "tobas_setup_assistant_rviz", false);

  // 一般用途のノード
  // QtとROSのスレッドを分離することでサービスコール時のデッドロックを回避する
  const auto node = rclcpp::Node::make_shared("tobas_setup_assistant");
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([executor]() { executor->spin(); });

  // GUIを表示
  QApplication qt_app(argc, argv);
  gui::setup_assistant::SetupAssistantWidget saw(node, rviz_node_if);
  saw.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  rclcpp::shutdown();
  return result;
}
