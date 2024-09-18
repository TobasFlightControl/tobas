#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QApplication>

#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_qt_tools/rviz/node_manager.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include <tobas_setup_assistant/setup_assistant.hpp>

namespace fs = std::filesystem;

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // ノードを起動
  qt::RvizNodeManager rviz_manager(argc, argv, "tobas_setup_assistant_rviz");
  ros2::AsyncNodeManager node_manager(argc, argv, "tobas_setup_assistant");

  // GUIを表示
  QApplication qt_app(argc, argv);
  const auto saw = new gui::setup_assistant::SetupAssistantWidget(node_manager.node(), rviz_manager.node());
  qt::MainWidget main("Tobas Setup Assistant", "", saw);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  rclcpp::shutdown();
  return result;
}
