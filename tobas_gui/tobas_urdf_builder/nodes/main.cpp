#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/argument.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include <tobas_urdf_builder/urdf_builder.hpp>
#include <tobas_urdf_builder/util.hpp>

namespace fs = std::filesystem;

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

  // GUIを表示
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto widget = new gui::ub::URDFBuilder();
  const auto icon_path = gui::ub::getPkgShareDir() / "resources/icon.png";
  qt::MainWidget main("Tobas URDF Builder", QString::fromStdString(icon_path), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
