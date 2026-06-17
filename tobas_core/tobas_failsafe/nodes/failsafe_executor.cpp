// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_mission_items/mission.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas
{
class FailsafeExecutorNode : public BaseNode
{
  using self = FailsafeExecutorNode;
  using super = BaseNode;

  using Action = tobas_mission_msgs::action::ExecuteMission;
  using Client = rclcpp_action::Client<Action>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

public:
  explicit FailsafeExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum State
  {
    kNoFailSafe,
    kReturnToLaunch,
    kLand,
  } state_ = kNoFailSafe;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::RCInput::ConstSharedPtr rcin_;

  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;
  ros2::ActionClientPtr<Action> mission_ac_;

  void disarm();

  void startRTL();
  void startLand();

  void vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};

FailsafeExecutorNode::FailsafeExecutorNode(const rclcpp::NodeOptions& options)
  : super("failsafe_executor", nodeOptions_Default(options))
{
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::vehicleHealthCb, this);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  rcin_sub_ = createSubscriber(topic::kRcInput, &self::rcInputCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(service::kSetArm);
  mission_ac_ = rclcpp_action::create_client<Action>(this, action::kExecuteMission);
}

void FailsafeExecutorNode::disarm()
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  set_arm_sc_->async_send_request(req);
}

void FailsafeExecutorNode::startRTL()
{
  mission::ReturnToLaunch rtl;
  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = mission::kReturnToLaunch;
  mission_item.data = st::toBytes(rtl);

  Action::Goal goal;
  goal.mission.items.push_back(mission_item);
  goal.priority.data = tobas_mission_msgs::msg::Priority::DEFENSIVE;

  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh)
  {
    if (!gh) {
      TOBAS_ERROR("RTL mission was rejected by the executor.");
      startLand();  // FIXME: 立て続けにアクションゴールを送るとインターフェースノードが詰まる
    }
  };
  opts.result_callback = [this](const GoalHandle::WrappedResult& res)
  {
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::CANCELED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        switch (res.result->error_code.data) {
          case tobas_mission_msgs::msg::ErrorCode::NO_ERROR:
          case tobas_mission_msgs::msg::ErrorCode::MISSION_SUPERSEDED:
          case tobas_mission_msgs::msg::ErrorCode::MANUAL_OVERRIDE:
            break;
          default:
            TOBAS_ERROR("RTL mission was aborted: ", res.result->error_message);
            startLand();
            break;
        }
        break;
      case rclcpp_action::ResultCode::UNKNOWN:
      default:
        TOBAS_ERROR("Unknown result code: ", (int)res.code);
        startLand();
        break;
    }
  };

  mission_ac_->async_send_goal(goal, opts);
  state_ = kReturnToLaunch;
}

void FailsafeExecutorNode::startLand()
{
  mission::Land land;
  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = mission::kLand;
  mission_item.data = st::toBytes(land);

  Action::Goal goal;
  goal.mission.items.push_back(mission_item);
  goal.priority.data = tobas_mission_msgs::msg::Priority::DEFENSIVE;

  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh)
  {
    if (!gh) {
      TOBAS_ERROR("Land mission was rejected by the executor.");
      disarm();
    }
  };
  opts.result_callback = [this](const GoalHandle::WrappedResult& res)
  {
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::CANCELED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        switch (res.result->error_code.data) {
          case tobas_mission_msgs::msg::ErrorCode::NO_ERROR:
          case tobas_mission_msgs::msg::ErrorCode::MISSION_SUPERSEDED:
          case tobas_mission_msgs::msg::ErrorCode::MANUAL_OVERRIDE:
            break;
          default:
            TOBAS_ERROR("Land mission was aborted: ", res.result->error_message);
            disarm();
            break;
        }
        break;
      case rclcpp_action::ResultCode::UNKNOWN:
      default:
        TOBAS_ERROR("Unknown result code: ", (int)res.code);
        disarm();
        break;
    }
  };

  mission_ac_->async_send_goal(goal, opts);
  state_ = kLand;
}

void FailsafeExecutorNode::vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  using VH = tobas_msgs::msg::VehicleHealth;

  const auto voltage_too_low = (health->battery_voltage == VH::FAILED);
  const auto radio_link_lost = (health->radio_link == VH::FAILED);
  const auto posvel_accurate = (health->position_accuracy == VH::PASSED) && (health->velocity_accuracy == VH::PASSED);

  // アームしていなければフェイルセーフは発動しない
  if (!arming_ || !arming_->data) {
    return;
  }

  const auto is_manual_ctrl_enabled = (!radio_link_lost && rcin_ && rcin_->ok && rcin_->enable);

  switch (state_) {
    case kNoFailSafe: {
      // 手動操縦中はフェイルセーフを発動しない
      if (is_manual_ctrl_enabled) {
        break;
      }

      // 位置と速度の推定ができてないならフェイルセーフを発動しない
      if (!posvel_accurate) {
        break;
      }

      // フェイルセーフを更新
      if (voltage_too_low) {
        TOBAS_WARN("Battery fail-safe is activated.");
        startLand();
        break;
      }
      if (radio_link_lost) {
        TOBAS_WARN("Radio fail-safe is activated.");
        startRTL();
        break;
      }

      break;
    }
    case kReturnToLaunch: {
      // 手動操縦が有効になったらフェイルセーフをキャンセル
      if (is_manual_ctrl_enabled) {
        TOBAS_INFO("Fail-safe was canceled because the manual control was enabled.");
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      }

      // 位置と速度の推定ができてないならフェイルセーフを発動しない
      if (!posvel_accurate) {
        break;
      }

      // フェイルセーフを更新
      if (voltage_too_low) {
        TOBAS_WARN("Battery fail-safe is activated.");
        startLand();
        break;
      }

      break;
    }
    case kLand: {
      // 手動操縦が有効になったらフェイルセーフをキャンセル
      if (is_manual_ctrl_enabled) {
        TOBAS_INFO("Fail-safe was canceled because the manual control was enabled.");
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      }

      break;
    }
    default: {
      throw;
    }
  }
}

void FailsafeExecutorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (!arming_) {
    arming_ = arming;
    return;
  }

  if (arming_->data && !arming->data) {
    // フェイルセーフは全てディスアームに収束するため，ディスアームされたらフェイルセーフが終了したと判定できる．
    switch (state_) {
      case kNoFailSafe:
        break;
      case kReturnToLaunch:
        state_ = kNoFailSafe;
        break;
      case kLand:
        state_ = kNoFailSafe;
        break;
      default:
        throw;
    }
  }

  arming_ = arming;
}

void FailsafeExecutorNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  rcin_ = rcin;
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FailsafeExecutorNode)
