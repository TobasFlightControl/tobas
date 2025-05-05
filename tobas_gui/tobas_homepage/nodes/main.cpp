#include <QApplication>

#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_gui_common/util.hpp>

#include "tobas_homepage/homepage.hpp"

int main(int argc, char** argv)
{
  // GUIを表示
  QApplication qt_app(argc, argv);
  const auto widget = new gui::homepage::HomepageWidget();
  qt::MainWidget main("Tobas Hardware Setup", QString::fromStdString(gui::common::getIconPath()), widget);
  main.show();

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  return result;
}
