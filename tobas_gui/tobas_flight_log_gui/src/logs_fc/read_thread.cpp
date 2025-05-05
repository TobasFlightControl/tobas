#include "tobas_flight_log_gui/logs_fc/read_thread.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/string.hpp>
#include <tobas_ros2_tools/util.hpp>

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
  if (ssh_client_.list(tobas::kRosbagDirRoot, list) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, ssh_client_.errorMessage(), {});
    return;
  }

  Q_EMIT finished(true, "Flight logs are read successfully.", qt::stringListFromStdToQt(list));
}
}  // namespace log
}  // namespace gui
