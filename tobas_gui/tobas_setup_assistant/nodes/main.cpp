#include <QApplication>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

#include <tobas_setup_assistant/setup_assistant.hpp>

static void sigIntHandler(int)
{
  QApplication::quit();
}

int main(int argc, char** argv)
{
  const auto client = std::make_unique<rviz_common::ros_integration::RosClientAbstraction>();
  const auto node = client->init(argc, argv, "tobas_setup_assistant", false);

  QApplication qt_app(argc, argv);
  setlocale(LC_NUMERIC, "C");

  gui::setup_assistant::SetupAssistantWidget saw(node);
  saw.show();

  signal(SIGINT, sigIntHandler);

  const auto result = qt_app.exec();
  rclcpp::shutdown();

  return result;
}
