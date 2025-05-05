#include <QApplication>

#include <tobas_gui_common/util.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include <tobas_urdf_builder/urdf_builder.hpp>

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  // GUIを表示
  QApplication qt_app(argc, argv);
  const auto widget = new gui::urdf_builder::URDFBuilder();
  qt::MainWidget main("URDF Builder", QString::fromStdString(gui::common::getIconPath()), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, sigIntHandler);

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qt_app.exec();
  rclcpp::shutdown();
  return result;
}
