#include <QApplication>

#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_qt_tools/rviz/node_manager.hpp>

#include <tobas_setup_assistant/setup_assistant.hpp>

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  qt::RvizNodeManager rviz_manager(argc, argv, "tobas_setup_assistant_rviz");
  ros2::AsyncNodeManager node_manager(argc, argv, "tobas_setup_assistant");

  // GUIを表示
  QApplication qt_app(argc, argv);
  gui::setup_assistant::SetupAssistantWidget saw(node_manager.node(), rviz_manager.node());
  saw.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  rclcpp::shutdown();
  return result;
}
