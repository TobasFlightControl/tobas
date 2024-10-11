#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/clean_thread.hpp"

using namespace std;

namespace gui
{
namespace log
{
CleanThread::CleanThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void CleanThread::run()
{
  const auto rosbag_dir = ros2::expandUser(tobas::kROSBagDir);
  if (ssh_client_.execute("rm -rf " + rosbag_dir.string() + "/*") != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "Flight logs are cleaned successfully.");
}
}  // namespace log
}  // namespace gui
