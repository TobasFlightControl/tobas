#include <rclcpp/wait_for_message.hpp>

#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/srv/get_arm.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/action/esc_calibration.hpp>

using namespace std;

class EscCalibrationNode : public tobas::BaseNode
{
  static constexpr double kHighDuration = 3.;      // [s]
  static constexpr double kLowDuration = 5.;       // [s]
  static constexpr double kTimeout = 30.;          // [s]
  static constexpr double kVoltageThreshold = 3.;  // [V]
  static constexpr auto kInterval = 10ms;
  static constexpr auto kWaitForBatteryTopic = 100ms;

  using self = EscCalibrationNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_calibration_msgs::action::ESCCalibration;

public:
  explicit EscCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  ros2::PublisherPtr<tobas_msgs::msg::ThrottleArray> throttles_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::ActionPtr<ActionType> as_;

  void sendMaximum();
  void sendMinimum();
  void setThrottle(const double& throttle);
  void setThrottleAndSleep(const double& throttle);
  bool checkDisarmed(string& message);
  bool enableRCOutputs(bool enable, string& message);
  bool checkBatteryDisconnected(string& message);
  bool waitForBatteryConnection(string& message);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

EscCalibrationNode::EscCalibrationNode(const rclcpp::NodeOptions& options) : super("esc_calibration", options)
{
  throttles_pub_ = createPublisher<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  drone_sub_ = createSubscriber<tobas::Drone>(tobas::kDroneTopic, &self::droneCb, this, true);
  as_ = createAction(tobas::kESCCalibAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

void EscCalibrationNode::sendMaximum()
{
  const auto start_time = get_clock()->now();
  while ((get_clock()->now() - start_time).seconds() < kHighDuration)
    setThrottleAndSleep(tobas::kMaxThrot);
}

void EscCalibrationNode::sendMinimum()
{
  const auto start_time = get_clock()->now();
  while ((get_clock()->now() - start_time).seconds() < kLowDuration)
    setThrottleAndSleep(tobas::kMinThrot);
}

void EscCalibrationNode::setThrottle(const double& throttle)
{
  auto throttles = std::make_unique<tobas_msgs::msg::ThrottleArray>();
  throttles->header.stamp = get_clock()->now();
  for (const auto& rotor : drone_->rotors)
  {
    throttles->throttles.emplace_back();
    throttles->throttles.back().channel = rotor.channel;
    throttles->throttles.back().throttle = throttle;
  }
  throttles_pub_->publish(move(throttles));
}

void EscCalibrationNode::setThrottleAndSleep(const double& throttle)
{
  setThrottle(throttle);
  rclcpp::sleep_for(kInterval);
}

bool EscCalibrationNode::checkDisarmed(string& message)
{
  ros2::SimpleServiceClient<tobas_msgs::srv::GetArm> sc(shared_from_this(), tobas::kGetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::GetArm::Request>();
  if (!sc.call(req))
  {
    message = "Failed to get arming status.";
    return false;
  }

  const auto& res = sc.getResponse();
  if (res->arming)
  {
    message = "Cannot execute ESC calibration because the motors are armed now.";
    return false;
  }

  return true;
}

bool EscCalibrationNode::enableRCOutputs(bool enable, string& message)
{
  ros2::SimpleServiceClient<tobas_msgs::srv::EnableRCOutput> sc(shared_from_this(), tobas::kEnableRcOutputSrv);

  for (const auto& rotor : drone_->rotors)
  {
    const auto req = std::make_shared<tobas_msgs::srv::EnableRCOutput::Request>();
    req->channel = rotor.channel;
    req->enable = enable;
    if (!sc.call(req))
    {
      message = "Failed to call \"" + string(tobas::kEnableRcOutputSrv) + "\" service.";
      return false;
    }

    const auto& res = sc.getResponse();
    if (!res->success)
    {
      message = res->message;
      return false;
    }
  }

  return true;
}

bool EscCalibrationNode::checkBatteryDisconnected(string& message)
{
  tobas_msgs::msg::Battery battery;

  // バッテリー状態を取得
  if (!rclcpp::wait_for_message(battery, shared_from_this(), tobas::kBatteryTopic, kWaitForBatteryTopic))
  {
    message = "Failed to get battery status.";
    return false;
  }

  // バッテリー電圧が閾値以下であることを確認
  if (battery.voltage > kVoltageThreshold)
  {
    message = "Please disconnect battery before starting ESC calibration.";
    return false;
  }

  return true;
}

bool EscCalibrationNode::waitForBatteryConnection(string& message)
{
  // バッテリーメッセージを初期化
  battery_ = nullptr;

  // 一時的にバッテリーの購読を開始
  const auto battery_sub = createSubscriber(tobas::kBatteryTopic, &EscCalibrationNode::batteryCb, this);

  // バッテリー電圧が閾値を超えるまで最大値を指令し続ける
  const auto start_time = get_clock()->now();
  while (battery_ == nullptr || battery_->voltage < kVoltageThreshold)
  {
    if ((get_clock()->now() - start_time).seconds() > kTimeout)
    {
      enableRCOutputs(false, message);
      message = "Battery connection is not detected before timeout.";
      return false;
    }
    setThrottleAndSleep(tobas::kMaxThrot);
  }

  return true;
}

void EscCalibrationNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void EscCalibrationNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

rclcpp_action::GoalResponse
EscCalibrationNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EscCalibrationNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EscCalibrationNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("ESC calibration is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  if (drone_ == nullptr)
  {
    result->message = "Drone configuration has not been received yet.";
    goal_handle->abort(result);
    return;
  }

  // アームされていないことを確認
  if (!checkDisarmed(result->message))
  {
    goal_handle->abort(result);
    return;
  }

  // バッテリーが接続されていないことを確認
  if (!checkBatteryDisconnected(result->message))
  {
    goal_handle->abort(result);
    return;
  }

  // RC出力を有効化
  if (!enableRCOutputs(true, result->message))
  {
    goal_handle->abort(result);
    return;
  }

  // バッテリーが接続されるのを待つ
  TOBAS_INFO("Waiting for battery connection.");
  if (!waitForBatteryConnection(result->message))
  {
    goal_handle->abort(result);
    return;
  }

  // 最大スロットルを指令
  TOBAS_INFO("Sending maximum throttle.");
  sendMaximum();

  // 最小スロットルを指令
  TOBAS_INFO("Sending minimum throttle.");
  sendMinimum();

  // RC出力を無効化
  if (!enableRCOutputs(false, result->message))
  {
    goal_handle->abort(result);
    return;
  }

  result->message.clear();
  goal_handle->succeed(result);
}

RCLCPP_COMPONENTS_REGISTER_NODE(EscCalibrationNode)
