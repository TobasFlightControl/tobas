#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>

#include "tobas_gcs/gcs.hpp"
#include "tobas_gcs/util.hpp"

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // X11を強制
  gui::cmn::NonRosArgumentParser arg_parser(argc, argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform." << std::endl;
    return EXIT_FAILURE;
  }

  // ノードを起動
  ros2::AsyncNodeManager node_manager(argc, argv, "tobas_gcs");

  // GUIを表示
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto title = "Tobas (" + gui::cmn::currentVersion() + ")";
  const auto icon_path = gui::gcs::getPkgShareDir() / "resources/icon.png";
  const auto widget = new gui::gcs::GroundControlStationWidget(node_manager.node());
  qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
