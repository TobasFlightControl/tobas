#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/logs_fc/download_thread.hpp"

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
  const auto remote_rosbag_path = fs::path(tobas::kROSBagDirRoot) / rosbag_name_.toStdString();
  const auto local_pardir = ros2::expandUser(tobas::kROSBagDirHome);
  if (ssh_client_.scpGet(remote_rosbag_path, local_pardir) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "The flight log has been downloaded successfully.");
}

const QString& DownloadThread::getROSBagName() const
{
  return rosbag_name_;
}

void DownloadThread::setROSBagName(const QString& rosbag_name)
{
  rosbag_name_ = rosbag_name;
}
}  // namespace log
}  // namespace gui
