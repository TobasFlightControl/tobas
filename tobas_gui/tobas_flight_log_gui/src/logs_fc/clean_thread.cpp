#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/logs_fc/clean_thread.hpp"

using namespace std;

namespace gui
{
namespace log
{
CleanThreadFC::CleanThreadFC(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void CleanThreadFC::run()
{
  if (ssh_client_.execute("rm -rf " + string(tobas::kROSBagDirRoot) + "/*", true) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "Flight logs are cleaned successfully.");
}
}  // namespace log
}  // namespace gui
