#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_mission_items/mission_items.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

class FailsafeExecutorNode : public tobas::BaseNode
{
  using self = FailsafeExecutorNode;
  using super = tobas::BaseNode;

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
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::RCInput::ConstSharedPtr rcin_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::Gnss::ConstSharedPtr gnss_;

  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;
  ros2::ActionClientPtr<Action> mission_ac_;

  void disarm();

  void startRTL();
  void startLand();

  void vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};

FailsafeExecutorNode::FailsafeExecutorNode(const rclcpp::NodeOptions& options) : super("failsafe_executor", options)
{
  health_sub_ = createSubscriber(tobas::kVehicleHealthTopic, &self::vehicleHealthCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  rcin_sub_ = createSubscriber(tobas::kRcInputTopic, &self::rcInputCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);
  gnss_sub_ = createSubscriber(tobas::kGnssTopic, &self::gnssCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
  mission_ac_ = rclcpp_action::create_client<Action>(this, tobas::kExecuteMissionAction);
}

void FailsafeExecutorNode::disarm()
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  set_arm_sc_->async_send_request(req);
}

void FailsafeExecutorNode::startRTL()
{
  tobas::mission::ReturnToLaunch rtl;
  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = tobas::mission::kReturnToLaunch;
  mission_item.data = tbs::toBytes(rtl);

  Action::Goal goal;
  goal.mission.header.stamp = now();
  goal.mission.items.push_back(mission_item);

  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh)
  {
    if (!gh) {
      TOBAS_ERROR("Mission goal was rejected by server.");
      startLand();
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
        TOBAS_ERROR("Mission was aborted: ", res.result->message);
        disarm();
        break;
      default:
        TOBAS_ERROR("Unknown result code: ", (int)res.code);
        disarm();
        break;
    }
  };

  mission_ac_->async_send_goal(goal, opts);

  state_ = kLand;
}

void FailsafeExecutorNode::startLand()
{
  tobas::mission::Land land;
  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = tobas::mission::kLand;
  mission_item.data = tbs::toBytes(land);

  Action::Goal goal;
  goal.mission.header.stamp = now();
  goal.mission.items.push_back(mission_item);

  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh)
  {
    if (!gh) {
      TOBAS_ERROR("Mission goal was rejected by server.");
      disarm();
    }
  };
  opts.result_callback = [this](const GoalHandle::WrappedResult& res)
  {
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;  // フェイルセーフは必ず着陸で終わる
      case rclcpp_action::ResultCode::CANCELED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        TOBAS_ERROR("Mission was aborted: ", res.result->message);
        disarm();
        break;
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
  const auto batt_voltage_too_low = health->battery_voltage == tobas_msgs::msg::VehicleHealth::FAILED;
  const auto radio_link_lost = health->radio_link == tobas_msgs::msg::VehicleHealth::FAILED;

  // RC入力が取れてなければメッセージをリセット
  if (radio_link_lost) {
    rcin_.reset();
  }

  // アームしていなければフェイルセーフは発動しない
  if (!arming_ || !arming_->data) {
    return;
  }

  switch (state_) {
    case kNoFailSafe: {
      // 手動操縦中はフェイルセーフは発動しない
      if (rcin_ && rcin_->enable) {
        break;
      }

      // フェイルセーフを更新
      if (batt_voltage_too_low) {
        TOBAS_WARN("Battery fail-safe is activated.");
        if (landed_ && landed_->data) {
          disarm();
        }
        else {
          startLand();
        }
        break;
      }
      if (radio_link_lost) {
        TOBAS_WARN("Radio fail-safe is activated.");
        if (landed_ && landed_->data) {
          disarm();
        }
        else if (gnss_->fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
          startRTL();
        }
        else {
          startLand();
        }
        break;
      }

      break;
    }
    case kReturnToLaunch: {
      // 手動操縦が有効になったらフェイルセーフをキャンセル
      if (rcin_ && rcin_->enable) {
        TOBAS_WARN("Canceling RTL because the manual mode is enabled.");
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      }

      // フェイルセーフを更新
      if (batt_voltage_too_low) {
        TOBAS_WARN("Battery fail-safe is activated.");
        mission_ac_->async_cancel_all_goals();
        startLand();
        break;
      }

      break;
    }
    case kLand: {
      // 手動操縦が有効になったらフェイルセーフをキャンセル
      if (rcin_ && rcin_->enable) {
        TOBAS_WARN("Canceling the land action because the manual mode is enabled.");
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
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      case kLand:
        mission_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      default:
        throw;
    }
  }

  arming_ = arming;
}

void FailsafeExecutorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void FailsafeExecutorNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  rcin_ = rcin;
}

void FailsafeExecutorNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

void FailsafeExecutorNode::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  gnss_ = gnss;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FailsafeExecutorNode)
