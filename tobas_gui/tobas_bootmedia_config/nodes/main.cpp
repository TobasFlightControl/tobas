#include <csignal>
#include <filesystem>

#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_bootmedia_config/bootmedia_config.hpp"
#include "tobas_bootmedia_config/util.hpp"

namespace fs = std::filesystem;

// static void sigIntHandler(int)
// {
//   QApplication::quit();
// }

int main(int argc, char** argv)
{
  // GUIを表示
  QApplication qapp(argc, argv);
  const auto widget = new tobas::gui::bm::BootmediaConfigWidget();
  const auto pkg_path = tobas::gui::bm::getPkgShareDir();
  const auto icon_path = pkg_path / "resources/icon.png";
  qt::MainWidget main("Tobas Bootmedia Config", QString::fromStdString(icon_path), widget);
  main.show();

  // Ctrl+Cで即終了
  // signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  return result;
}
