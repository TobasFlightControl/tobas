#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/string.hpp>

#include "tobas_flight_log_gui/read_thread.hpp"

using namespace std;

namespace gui
{
namespace log
{
ReadThread::ReadThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void ReadThread::run()
{
  vector<string> list;
  if (ssh_client_.list(tobas::kROSBagDir, list) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage(), {});
    return;
  }

  Q_EMIT finished(true, "The name of flight logs are read successfully.", qt::stringListFromStdToQt(list));
}
}  // namespace log
}  // namespace gui
