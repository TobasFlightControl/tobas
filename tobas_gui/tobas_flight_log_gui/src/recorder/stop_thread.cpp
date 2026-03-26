#include "tobas_flight_log_gui/recorder/stop_thread.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_msgs/srv/bag_record_stop.hpp>

#include "tobas_flight_log_gui/recorder/constants.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
RecordStopThread::RecordStopThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void RecordStopThread::run()
{
  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStop> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceNS, tobas::service::kRosbagRecordStop));

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStop::Request>();

  if (!sc.call(req, kRecordServiceTimeout)) {
    Q_EMIT finished(false, "Flight log recording service is unavailable.");
    return;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    Q_EMIT finished(false, "Failed to stop recording flight log: " + QString(res->message.c_str()));
    return;
  }

  Q_EMIT finished(true, "");
}

void RecordStopThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
