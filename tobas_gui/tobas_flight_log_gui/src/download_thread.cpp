#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/download_thread.hpp"

using namespace std;

namespace gui
{
namespace log
{
DownloadThread::DownloadThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void DownloadThread::run()
{
  const auto pardir = ros2::expandUser(tobas::kROSBagDir);
  const auto rosbag_path = pardir / rosbag_name_.toStdString();
  if (ssh_client_.scpGet(rosbag_path, pardir) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, "The flight log is save as " + QString(rosbag_path.c_str()) + " successfully.");
}

void DownloadThread::setROSBagName(const QString& rosbag_name)
{
  rosbag_name_ = rosbag_name;
}
}  // namespace log
}  // namespace gui
