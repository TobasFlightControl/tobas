#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include <tobas_urdf_builder/urdf_builder.hpp>
#include <tobas_urdf_builder/util.hpp>

int main(int argc, char** argv)
{
  // X11を強制
  tobas::gui::cmn::NonRosArgumentParser arg_parser(argc, argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform." << std::endl;
    return EXIT_FAILURE;
  }

  // コンソール出力に着色
  qInstallMessageHandler(tobas::qt::colorMessageHandler);

  // GUIを表示
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto title = "Tobas URDF Builder (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::ub::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::ub::URDFBuilder();
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, [](int) { QApplication::quit(); });

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
