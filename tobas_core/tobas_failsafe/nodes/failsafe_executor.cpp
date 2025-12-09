#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_mission_msgs/action/land.hpp>
#include <tobas_mission_msgs/action/move.hpp>
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

  using MoveAction = tobas_mission_msgs::action::Move;
  using MoveClient = rclcpp_action::Client<MoveAction>;
  using MoveGoalHandle = rclcpp_action::ClientGoalHandle<MoveAction>;

  using LandAction = tobas_mission_msgs::action::Land;
  using LandClient = rclcpp_action::Client<LandAction>;
  using LandGoalHandle = rclcpp_action::ClientGoalHandle<LandAction>;

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
  tobas_msgs::Gnss::ConstSharedPtr gnss_arm_;  // アームした地点の座標

  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  MoveClient::SharedPtr move_ac_;
  LandClient::SharedPtr land_ac_;

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

  move_ac_ = rclcpp_action::create_client<MoveAction>(this, tobas::kMoveAction);
  land_ac_ = rclcpp_action::create_client<LandAction>(this, tobas::kLandAction);
}

void FailsafeExecutorNode::disarm()
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  set_arm_sc_->async_send_request(req);
}

void FailsafeExecutorNode::startRTL()
{
  assert(gnss_arm_);

  MoveAction::Goal goal;
  goal.level.data = tobas_command_msgs::msg::CommandLevel::DEFENSIVE;
  goal.target_latitude = gnss_arm_->latitude;
  goal.target_longitude = gnss_arm_->longitude;

  // TODO: パラメータをSAで指定可能にする
  goal.target_altitude = odom_ ? odom_->frame.p.z() : 15.;
  goal.max_horizontal_velocity = 5.;
  goal.max_vertical_velocity = 1.5;
  goal.max_horizontal_accel = 5.;
  goal.max_vertical_accel = 3.;
  goal.max_horizontal_jerk = 4.;
  goal.max_vertical_jerk = 4.;

  MoveClient::SendGoalOptions opts;
  opts.goal_response_callback = [this](const MoveGoalHandle::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      TOBAS_ERROR("Move action goal was rejected by server.");
      startLand();
    }
  };
  opts.result_callback = [this](const MoveGoalHandle::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        startLand();  // RTLの次は必ず着陸
        break;
      case rclcpp_action::ResultCode::CANCELED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        TOBAS_ERROR("Move action was aborted: ", result.result->message);
        startLand();
        break;
      default:
        TOBAS_ERROR("Unknown result code: ", (int)result.code);
        startLand();
        break;
    }
  };

  move_ac_->async_send_goal(goal, opts);

  state_ = kReturnToLaunch;
}

void FailsafeExecutorNode::startLand()
{
  // TODO: パラメータをSAで指定可能にする
  LandAction::Goal goal;
  goal.level.data = tobas_command_msgs::msg::CommandLevel::DEFENSIVE;
  goal.speed = 0.7;

  LandClient::SendGoalOptions opts;
  opts.goal_response_callback = [this](const LandGoalHandle::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      TOBAS_ERROR("Land action goal was rejected by server.");
      disarm();
    }
  };
  opts.result_callback = [this](const LandGoalHandle::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;  // フェイルセーフは必ず着陸で終わる
      case rclcpp_action::ResultCode::CANCELED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        TOBAS_ERROR("Land action was aborted: ", result.result->message);
        disarm();
        break;
      default:
        TOBAS_ERROR("Unknown result code: ", (int)result.code);
        disarm();
        break;
    }
  };

  land_ac_->async_send_goal(goal, opts);

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
        else if (gnss_arm_ && health->position_accuracy == tobas_msgs::msg::VehicleHealth::PASSED) {
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
        move_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      }

      // フェイルセーフを更新
      if (batt_voltage_too_low) {
        TOBAS_WARN("Battery fail-safe is activated.");
        move_ac_->async_cancel_all_goals();
        startLand();
        break;
      }

      break;
    }
    case kLand: {
      // 手動操縦が有効になったらフェイルセーフをキャンセル
      if (rcin_ && rcin_->enable) {
        TOBAS_WARN("Canceling the land action because the manual mode is enabled.");
        land_ac_->async_cancel_all_goals();
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
        move_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      case kLand:
        land_ac_->async_cancel_all_goals();
        state_ = kNoFailSafe;
        break;
      default:
        throw;
    }
  }
  else if (!arming_->data && arming->data) {
    // アームされた地点の座標を保存
    gnss_arm_ = gnss_;
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
  // 3次元Fixの場合のみ位置情報を更新
  if (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
    gnss_ = gnss;
  }
  else {
    gnss_.reset();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(FailsafeExecutorNode)
