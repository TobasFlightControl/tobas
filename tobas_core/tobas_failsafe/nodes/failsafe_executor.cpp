#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>

#include <tobas_mission_msgs/action/land.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

class FailsafeExecutorNode : public tobas::BaseNode
{
  using self = FailsafeExecutorNode;
  using super = tobas::BaseNode;

  using LandAction = tobas_mission_msgs::action::Land;
  using LandClient = rclcpp_action::Client<LandAction>;
  using LandGoalHandle = rclcpp_action::ClientGoalHandle<LandAction>;

public:
  explicit FailsafeExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum State
  {
    kNone,
    kLand,
  } state_ = kNone;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  LandClient::SharedPtr land_ac_;

  void disarm();

  void startLandAction();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
};

FailsafeExecutorNode::FailsafeExecutorNode(const rclcpp::NodeOptions& options) : super("failsafe_executor", options)
{
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  health_sub_ = createSubscriber(tobas::kVehicleHealthTopic, &self::vehicleHealthCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);

  land_ac_ = rclcpp_action::create_client<LandAction>(this, tobas::kLandAction);
}

void FailsafeExecutorNode::disarm()
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  set_arm_sc_->async_send_request(req);
}

void FailsafeExecutorNode::startLandAction()
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
        break;
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

void FailsafeExecutorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // フェイルセーフは全てディスアームに収束するため，ディスアームされたらフェイルセーフが終了したと判定できる．
  if (!arming->data) {
    state_ = kNone;
  }

  arming_ = arming;
}

void FailsafeExecutorNode::vehicleHealthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  if (!arming_ || !arming_->data) {
    return;
  }

  switch (state_) {
    case kNone:
      if (health->radio_link == tobas_msgs::msg::VehicleHealth::FAILED) {
        TOBAS_WARN("Radio fail-safe is activated.");
        startLandAction();
      }
      break;

    case kLand:
      if (health->radio_link == tobas_msgs::msg::VehicleHealth::PASSED) {
        land_ac_->async_cancel_all_goals();
        state_ = kNone;
      }
      break;

    default:
      throw;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(FailsafeExecutorNode)
