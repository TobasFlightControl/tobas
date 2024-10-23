#pragma once

#include <std_srvs/srv/empty.hpp>

#include <tobas_node/node.hpp>

namespace hal
{
class BaseSensorNode : public tobas::BaseNode
{
  using self = BaseSensorNode;
  using super = tobas::BaseNode;

  using Empty = std_srvs::srv::Empty;

public:
  explicit BaseSensorNode(const std::string& name, const rclcpp::NodeOptions& options);

protected:
  ros2::TimerPtr main_timer_;

private:
  ros2::ServiceServerPtr<Empty> start_ss_;
  ros2::ServiceServerPtr<Empty> stop_ss_;

  void startMainTimerSrvCb(const Empty::Request::ConstSharedPtr& req, const Empty::Response::SharedPtr& res);
  void stopMainTimerSrvCb(const Empty::Request::ConstSharedPtr& req, const Empty::Response::SharedPtr& res);
};
}  // namespace hal
