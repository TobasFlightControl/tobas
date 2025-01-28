#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/string.hpp>

#include "tobas_flight_log_gui/logs_fc/load_thread.hpp"

using namespace std;

namespace gui
{
namespace log
{
LoadThreadFC::LoadThreadFC(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void LoadThreadFC::run()
{
  vector<string> list;
  if (ssh_client_.list(tobas::kROSBagDirRoot, list) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage(), {});
    return;
  }

  Q_EMIT finished(true, "The name of flight logs are read successfully.", qt::stringListFromStdToQt(list));
}
}  // namespace log
}  // namespace gui
