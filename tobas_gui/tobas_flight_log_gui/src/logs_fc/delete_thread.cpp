#include "tobas_flight_log_gui/logs_fc/delete_thread.hpp"

#include <filesystem>

#include <tobas_constants/constants.hpp>

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace log
{
DeleteThread::DeleteThread(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
}

void DeleteThread::run()
{
  const string rosbag_path = fs::path(tobas::kRosbagDirRoot) / log_name_.toStdString();
  if (ssh_client_.execute("rm -rf " + rosbag_path, true) != ssh::SSHClient::E_NO_ERROR) {
    Q_EMIT finished(false, ssh_client_.errorMessage());
    return;
  }

  Q_EMIT finished(true, QString::fromStdString(rosbag_path) + "is deleted successfully.");
}

const QString& DeleteThread::getLogName() const
{
  return log_name_;
}

void DeleteThread::setLogName(const QString& log_name)
{
  log_name_ = log_name;
}
}  // namespace log
}  // namespace gui
