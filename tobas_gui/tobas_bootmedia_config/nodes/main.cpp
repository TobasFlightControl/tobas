#include <QApplication>

#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_bootmedia_config/bootmedia_config.hpp"
#include "tobas_bootmedia_config/util.hpp"

int main(int argc, char** argv)
{
  QApplication qapp(argc, argv);
  const auto title = "Tobas Bootmedia Config (" + gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::bm::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::bm::BootmediaConfigWidget();
  qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();
  const auto result = qapp.exec();
  return result;
}
