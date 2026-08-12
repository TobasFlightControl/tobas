// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_mission_items/mission.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
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
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;
  ros2::ActionClientPtr<Action> mission_ac_;

  void disarm();

  void startRTL();
  void startLand();

  void vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
};

FailsafeExecutorNode::FailsafeExecutorNode(const rclcpp::NodeOptions& options)
  : super("failsafe_executor", nodeOptions_Default(options))
{
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::vehicleHealthCb, this);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  rcin_sub_ = createSubscriber(topic::kRcInput, &self::rcInputCb, this);
  landed_sub_ = createSubscriber(topic::kLanded, &self::landedCb, this);

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
      startLand();  // FIXME: The interface node can get stuck if action goals are sent in quick succession.
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

  // Fail-safe does not activate when the vehicle is not armed.
  if (!arming_ || !arming_->data) {
    return;
  }

  const auto voltage_too_low = (health->battery_voltage == VH::FAILED);
  const auto radio_link_lost = (health->radio_link == VH::FAILED);
  const auto rotor_link_lost = (health->rotor_links == VH::FAILED);
  const auto gnss_fix_lost = (health->gnss_fix == VH::FAILED);
  const auto gnss_fixed = (health->gnss_fix == VH::PASSED);
  const auto horizontal_position_accurate = (health->horizontal_position_accuracy == VH::PASSED);
  const auto vertical_position_accurate = (health->vertical_position_accuracy == VH::PASSED);

  const auto landed = (landed_ && landed_->landed);
  const auto rcin_ok = (!radio_link_lost && rcin_ && rcin_->status == tobas_msgs::msg::RCInput::STATUS_OK);
  const auto manual_ctrl_enabled = (rcin_ok && rcin_->enable);
  const auto landing_ready = (!manual_ctrl_enabled && vertical_position_accurate);
  const auto rtl_ready = (landing_ready && gnss_fixed && horizontal_position_accurate);

  switch (state_) {
    case kNoFailSafe: {
      // Update fail-safe state.
      if (voltage_too_low) {
        if (landed) {
          TOBAS_WARN("Battery fail-safe activated: disarming because the vehicle is already landed.");
          disarm();
        }
        else if (landing_ready) {
          TOBAS_WARN("Battery fail-safe activated: starting landing.");
          startLand();
        }
      }
      else if (radio_link_lost) {
        if (landed) {
          TOBAS_WARN("Radio fail-safe activated: disarming because the vehicle is already landed.");
          disarm();
        }
        else if (rtl_ready) {
          TOBAS_WARN("Radio fail-safe activated: starting return to launch.");
          startRTL();
        }
        else if (landing_ready) {
          TOBAS_WARN("Radio fail-safe activated: landing in place because horizontal position is inaccurate.");
          startLand();
        }
      }
      else if (rotor_link_lost) {
        if (landed) {
          TOBAS_WARN("Rotor fail-safe activated: disarming because the vehicle is already landed.");
          disarm();
        }
        else if (rtl_ready) {
          TOBAS_WARN("Rotor fail-safe activated: starting return to launch.");
          startRTL();
        }
        else if (landing_ready) {
          TOBAS_WARN("Rotor fail-safe activated: landing in place because horizontal position is inaccurate.");
          startLand();
        }
      }
      else if (gnss_fix_lost) {
        if (landed) {
          TOBAS_WARN("GNSS fail-safe activated: disarming because the vehicle is already landed.");
          disarm();
        }
        else if (landing_ready) {
          TOBAS_WARN("GNSS fail-safe activated: landing in place because GNSS lost its 3D fix.");
          startLand();
        }
      }

      break;
    }
    case kReturnToLaunch: {
      // Cancel fail-safe when manual control is enabled.
      if (manual_ctrl_enabled) {
        TOBAS_INFO("Fail-safe was canceled because the manual control was enabled.");
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      }

      // Update fail-safe state.
      if (voltage_too_low && landing_ready) {
        TOBAS_WARN("Battery fail-safe activated during RTL: switching to landing.");
        startLand();
      }
      else if (gnss_fix_lost && landing_ready) {
        TOBAS_WARN("GNSS fail-safe activated during RTL: switching to landing.");
        startLand();
      }

      break;
    }
    case kLand: {
      // Cancel fail-safe when manual control is enabled.
      if (manual_ctrl_enabled) {
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
    // All fail-safe paths converge to disarming, so fail-safe can be considered finished once disarmed.
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

void FailsafeExecutorNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FailsafeExecutorNode)
