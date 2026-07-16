// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/hardware.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/static_for.hpp>

#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/set_rpm_control_gains.hpp>

namespace tobas
{
class RpmControlConfigServer : public BaseNode
{
  using self = RpmControlConfigServer;
  using super = BaseNode;

public:
  explicit RpmControlConfigServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::msg::RpmControlGain gain_;

  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::SetRpmControlGains> config_sc_;

  template <size_t Channel>
  bool gainCb(const long& p);

  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
};

RpmControlConfigServer::RpmControlConfigServer(const rclcpp::NodeOptions& options)
  : super(node::kRpmControlConfigServer, nodeOptions_DParam(options))
{
  rotor_states_sub_ = createSubscriber(topic::kRotorStates, &self::rotorStatesCb, this);
  config_sc_ = create_client<tobas_msgs::srv::SetRpmControlGains>(service::kSetRpmControlGains);
}

template <size_t Channel>
bool RpmControlConfigServer::gainCb(const long& p)
{
  if (!config_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kSetRpmControlGains, "\" is not ready.");
    return false;
  }

  gain_.channel = Channel;
  gain_.gain = p;

  const auto req = std::make_shared<tobas_msgs::srv::SetRpmControlGains::Request>();
  req->gains.push_back(gain_);

  config_sc_->async_send_request(req);

  return true;
}

void RpmControlConfigServer::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr&)
{
  // Register dynamic parameters after confirming that rotor states can be received,
  // meaning the node managing the DShot device is running.
  // This ensures that the initial filter settings are applied reliably.

  // Register dynamic parameters.
  st::staticFor<kMaxDshotChannels>(
    [&]<size_t I>()
    {
      const auto param_name = param::kRpmControlGainPrefix + std::to_string(I);
      addDynamicIntParam(param_name, &self::gainCb<I>, this, 1, 0, 0, UINT8_MAX);
    });

  // Cancel subscription.
  rotor_states_sub_.reset();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::RpmControlConfigServer)
