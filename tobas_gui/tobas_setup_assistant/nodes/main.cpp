#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/util.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>

#include "tobas_setup_assistant/constants.hpp"
#include "tobas_setup_assistant/setup_assistant.hpp"

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // X11を強制
  gui::common::NonRosArgumentParser arg_parser(argc, argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform." << std::endl;
    return EXIT_FAILURE;
  }

  // ノードを起動
  ros2::AsyncNodeManager node_manager(argc, argv, "sa");

  // GUIを表示
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto widget = new gui::sa::SetupAssistantWidget(node_manager.node());
  qt::MainWidget main(gui::sa::kTitle, QString::fromStdString(gui::common::getIconPath()), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
