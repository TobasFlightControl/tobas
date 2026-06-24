// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/hardware.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_node/node.hpp>

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

  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  ros2::ServiceClientPtr<tobas_msgs::srv::SetRpmControlGains> config_sc_;

  bool gainCb(size_t ch, const rclcpp::Parameter& param);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
};

RpmControlConfigServer::RpmControlConfigServer(const rclcpp::NodeOptions& options)
  : super(node::kRpmControlConfigServer, nodeOptions_DParam(options))
{
  rotor_states_pub_ = createSubscriber(topic::kRotorStates, &self::rotorStatesCb, this);
  config_sc_ = create_client<tobas_msgs::srv::SetRpmControlGains>(service::kSetRpmControlGains);
}

bool RpmControlConfigServer::gainCb(size_t ch, const rclcpp::Parameter& param)
{
  if (!config_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kSetRpmControlGains, "\" is not ready.");
    return false;
  }

  gain_.channel = ch;
  gain_.gain = std::clamp<int64_t>(param.as_int(), 0, UINT8_MAX);

  const auto req = std::make_shared<tobas_msgs::srv::SetRpmControlGains::Request>();
  req->gains.push_back(gain_);

  config_sc_->async_send_request(req);

  return true;
}

void RpmControlConfigServer::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr&)
{
  // モータの状態が受信可能即ちDShotデバイスを管理しているノードが立ち上がっているのを確認してから動的パラメータを登録する．
  // そうすることでフィルタの初期設定が確実に反映される．

  // Register dynamic parameters
  for (size_t ch = 0; ch < kMaxDshotChannels; ++ch) {
    const auto name = param::kRpmControlGainPrefix + std::to_string(ch);
    declare_parameter(name, 0);
    const auto cb_handle =
      dparam_sub_.add_parameter_callback(name, std::bind(&self::gainCb, this, ch, std::placeholders::_1));
    dparam_handles_.push_back(cb_handle);
  }

  // Cancel subscription
  rotor_states_pub_.reset();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::RpmControlConfigServer)
