#include <QApplication>

#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_bootmedia_config/bootmedia_config.hpp"
#include "tobas_bootmedia_config/util.hpp"

int main(int argc, char** argv)
{
  QApplication qapp(argc, argv);
  const auto widget = new tobas::gui::bm::BootmediaConfigWidget();
  const auto icon_path = tobas::gui::bm::getPkgShareDir() / "resources/icon.png";
  qt::MainWidget main("Tobas Bootmedia Config", QString::fromStdString(icon_path), widget);
  main.show();
  const auto result = qapp.exec();
  return result;
}
