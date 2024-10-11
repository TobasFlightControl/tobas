#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/download_thread.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace log
{
DownloadThread::DownloadThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void DownloadThread::run()
{
  const auto remote_rosbag_path = fs::path(tobas::kROSBagDir) / rosbag_name_.toStdString();
  const auto local_pardir = ros2::expandUser(tobas::kROSBagDir);
  if (ssh_client_.scpGet(remote_rosbag_path, local_pardir) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "The flight log has been downloaded successfully.");
}

void DownloadThread::setROSBagName(const QString& rosbag_name)
{
  rosbag_name_ = rosbag_name;
}
}  // namespace log
}  // namespace gui
