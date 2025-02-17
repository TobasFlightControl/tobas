#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>

#include "tobas_flight_log_gui/recorder/start_thread.hpp"

namespace gui
{
namespace log
{
RecordStartThread::RecordStartThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void RecordStartThread::run()
{
  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStart> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, tobas::kROSBagRecordStartSrv));

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStart::Request>();
  req->name = log_name_;

  if (!sc.call(req))
  {
    Q_EMIT finished(false, "Flight log recording service is unavailable.");
    return;
  }

  const auto res = sc.getResponse();
  if (!res->success)
  {
    Q_EMIT finished(false, "Failed to start recording flight log: " + QString(res->message.c_str()));
    return;
  }

  Q_EMIT finished(true, "");
}

void RecordStartThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

void RecordStartThread::setLogName(const std::string& log_name)
{
  log_name_ = log_name;
}
}  // namespace log
}  // namespace gui
