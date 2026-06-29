// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
class RotorAnomalyDetectorNode : public BaseNode
{
  using self = RotorAnomalyDetectorNode;
  using super = BaseNode;

public:
  explicit RotorAnomalyDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  double no_comm_timeout_;  // [s]

  struct RotorData
  {
    bool is_alive = false;
    builtin_interfaces::msg::Time last_alive_time;
    builtin_interfaces::msg::Time last_dead_time;
  };

  Drone::ConstSharedPtr drone_;
  std::map<std::string, RotorData> data_;  // Link Name -> RotorData

  ros2::PublisherPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;

  ros2::TimerPtr publish_rotor_liveliness_timer_;

  void publishRotorLiveliness();

  void droneCb(const Drone::ConstSharedPtr& drone);
  void statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states);
};

RotorAnomalyDetectorNode::RotorAnomalyDetectorNode(const rclcpp::NodeOptions& options)
  : super("rotor_anomaly_detector", nodeOptions_Default(options))
{
  no_comm_timeout_ = getDoubleParam("no_communication_timeout");

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
}

void RotorAnomalyDetectorNode::publishRotorLiveliness()
{
  auto msg = std::make_unique<tobas_msgs::msg::RotorLivelinessArray>();
  msg->header.stamp = now();

  for (const auto& [link_name, data] : data_) {
    msg->data.emplace_back();
    msg->data.back().link_name = link_name;
    msg->data.back().alive = data.is_alive;
  }

  rotor_liveliness_pub_->publish(std::move(msg));
}

void RotorAnomalyDetectorNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  if (!drone->prop) {
    return;
  }

  drone_ = drone;

  data_.clear();
  for (const auto& [link_name, _] : drone->prop->rotors) {
    data_[link_name] = RotorData();
  }

  rotor_liveliness_pub_ = createPublisher<tobas_msgs::msg::RotorLivelinessArray>(topic::kRotorLiv);
  rotor_states_sub_ = createSubscriber(addThrotNS(topic::kRotorStates), &self::statesCb, this);
}

void RotorAnomalyDetectorNode::statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states)
{
  // モータの状態を受け取ってから生存状況の発行を開始する
  if (!publish_rotor_liveliness_timer_) {
    publish_rotor_liveliness_timer_ = createTimer(1s, &self::publishRotorLiveliness, this);
  }

  bool state_changed = false;

  // TODO: 推進系の種類によって適切な判定を行う

  for (const auto& state : states->states) {
    const auto data_it = data_.find(state.link_name);
    if (data_it == data_.end()) {
      TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Invalid rotor: \"", state.link_name, "\"");
      continue;
    }

    auto& data = data_it->second;
    const auto& cur_time = states->header.stamp;

    if (data.is_alive) {
      if (state.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
        // 一定時間通信が途絶えている場合は死んでいるとみなす
        if ((cur_time - data.last_alive_time).seconds() > no_comm_timeout_) {
          state_changed = true;
          data.is_alive = false;
          data.last_dead_time = cur_time;
          TOBAS_WARN("No communication with rotor \"", state.link_name, "\".");
        }
      }
      else {
        // 通信が確認できた最新の時刻を更新
        data.last_alive_time = cur_time;
      }
    }
    else {
      if (state.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
        // 一定時間通信があれば回復したとみなす
        if ((cur_time - data.last_dead_time).seconds() > no_comm_timeout_ * 2) {
          state_changed = true;
          data.is_alive = true;
          data.last_alive_time = cur_time;
          TOBAS_INFO("Communication with rotor \"", state.link_name, "\" has been recovered.");
        }
      }
      else {
        // 通信が確認できない最新の時刻を更新
        data.last_dead_time = cur_time;
      }
    }
  }

  // 状態が切り替わっていれば発行
  if (state_changed) {
    publishRotorLiveliness();
    publish_rotor_liveliness_timer_->reset();
  }
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::RotorAnomalyDetectorNode)
