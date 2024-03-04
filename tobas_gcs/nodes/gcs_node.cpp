#include <csignal>
#include <QApplication>

#include "../include/tobas_gcs/main_widget.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_gcs");

  QApplication app(argc, argv);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_gcs::MainWidget main_widget(nh, pnh);

  main_widget.show();

  // Ctrl+Cを検出したらプロセスを落とす
  signal(SIGINT, SIG_DFL);

  return app.exec();
}
