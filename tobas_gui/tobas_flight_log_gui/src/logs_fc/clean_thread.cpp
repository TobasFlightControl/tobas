#include "tobas_flight_log_gui/logs_fc/clean_thread.hpp"

#include <tobas_constants/constants.hpp>

namespace gui
{
namespace log
{
CleanThread::CleanThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void CleanThread::run()
{
  if (ssh_client_.execute("rm -rf " + std::string(tobas::kRosbagDirRoot) + "/*", true) != ssh::SshClient::kNoError) {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "Flight logs are cleaned successfully.");
}
}  // namespace log
}  // namespace gui
