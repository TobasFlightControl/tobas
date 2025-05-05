#include <QApplication>

#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_gui_common/util.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"
#include "tobas_setup_assistant/constants.hpp"

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // ノードを起動
  ros2::AsyncNodeManager node_manager(argc, argv, "sa");

  // GUIを表示
  QApplication qt_app(argc, argv);
  const auto widget = new gui::sa::SetupAssistantWidget(node_manager.node());
  qt::MainWidget main(gui::sa::kTitle, QString::fromStdString(gui::common::getIconPath()), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  rclcpp::shutdown();
  return result;
}
