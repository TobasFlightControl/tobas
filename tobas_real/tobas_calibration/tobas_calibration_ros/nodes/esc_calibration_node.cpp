#include <rclcpp/wait_for_message.hpp>

#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/srv/get_arm.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_drone_msgs/Drone.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/srv/esc_calibration.hpp>

using namespace std;

class EscCalibrationNode : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "esc_calibration";

  static constexpr double kHighDuration = 3.;      // [s]
  static constexpr double kLowDuration = 5.;       // [s]
  static constexpr double kTimeout = 30.;          // [s]
  static constexpr double kVoltageThreshold = 3.;  // [V]
  static constexpr auto kInterval = 10ms;
  static constexpr auto kWaitForBatteryTopic = 100ms;

  using self = EscCalibrationNode;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::srv::EscCalibration;

public:
  explicit EscCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  ros2::PublisherPtr<tobas_msgs::msg::ThrottleArray> throttles_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::ServicePtr<SrvType> ss_;

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
  void executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
};

EscCalibrationNode::EscCalibrationNode(const rclcpp::NodeOptions& options) : super("esc_calibration", options)
{
  throttles_pub_ = createPublisher<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  drone_sub_ = createSubscriber<tobas::Drone>(tobas::kDroneTopic, &self::droneCb, this, true);
  ss_ = createService<SrvType>(kServiceName, &self::executeCb, this);
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
    rclcpp::spin_some(shared_from_this());
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

void EscCalibrationNode::executeCb(const SrvType::Request::ConstSharedPtr&, const SrvType::Response::SharedPtr& res)
{
  if (drone_ == nullptr)
  {
    res->success = false;
    res->message = "Drone configuration has not been received yet.";
    return;
  }

  // アームされていないことを確認
  if (!checkDisarmed(res->message))
  {
    res->success = false;
    return;
  }

  // バッテリーが接続されていないことを確認
  if (!checkBatteryDisconnected(res->message))
  {
    res->success = false;
    return;
  }

  // RC出力を有効化
  if (!enableRCOutputs(true, res->message))
  {
    res->success = false;
    return;
  }

  // バッテリーが接続されるのを待つ
  TOBAS_INFO("Waiting for battery connection.");
  if (!waitForBatteryConnection(res->message))
  {
    res->success = false;
    return;
  }

  // 最大スロットルを指令
  TOBAS_INFO("Sending maximum throttle.");
  sendMaximum();

  // 最小スロットルを指令
  TOBAS_INFO("Sending minimum throttle.");
  sendMinimum();

  // RC出力を無効化
  if (!enableRCOutputs(false, res->message))
  {
    res->success = false;
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(EscCalibrationNode)
