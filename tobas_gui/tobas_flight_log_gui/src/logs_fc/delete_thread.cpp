#include <filesystem>

#include <tobas_constants/constants.hpp>

#include "tobas_flight_log_gui/logs_fc/delete_thread.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace log
{
DeleteThreadFC::DeleteThreadFC(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void DeleteThreadFC::run()
{
  const string rosbag_path = fs::path(tobas::kROSBagDirRoot) / rosbag_name_.toStdString();
  if (ssh_client_.execute("rm -rf " + rosbag_path, true) != ssh::SSHClient::E_NO_ERROR)
  {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, QString::fromStdString(rosbag_path) + "is deleted successfully.");
}

void DeleteThreadFC::setROSBagName(const QString& rosbag_name)
{
  rosbag_name_ = rosbag_name;
}
}  // namespace log
}  // namespace gui
