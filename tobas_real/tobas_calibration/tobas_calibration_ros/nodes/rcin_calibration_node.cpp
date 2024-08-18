#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/srv/rc_input_calibration.hpp>

using namespace std;

class RCInputCalibrationNode : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "rcin_calibration";

  static constexpr size_t kMinSignalRange = 300;

  using self = RCInputCalibrationNode;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::srv::RCInputCalibration;

public:
  explicit RCInputCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ServicePtr<SrvType> ss_;

  void executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
};

RCInputCalibrationNode::RCInputCalibrationNode(const rclcpp::NodeOptions& options) : super("rcin_calibration", options)
{
  ss_ = createService<SrvType>(kServiceName, &self::executeCb, this);
}

void RCInputCalibrationNode::executeCb(
  const SrvType::Request::ConstSharedPtr& req,
  const SrvType::Response::SharedPtr& res)
{
  // 各チャンネルの値の範囲をチェック
  if (abs(req->roll_left - req->roll_right) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on Roll channel are too close.";
    return;
  }
  if (abs(req->pitch_up - req->pitch_down) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on Pitch channel are too close.";
    return;
  }
  if (abs(req->yaw_left - req->yaw_right) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on Yaw channel are too close.";
    return;
  }
  if (abs(req->throttle_up - req->throttle_down) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on Throttle channel are too close.";
    return;
  }
  if (abs(req->mode_program - req->mode_acrobat) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on Mode channel are too close.";
    return;
  }
  if (abs(req->estop_on - req->estop_off) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on E-Stop channel are too close.";
    return;
  }
  if (abs(req->gpsw_on - req->gpsw_off) < kMinSignalRange)
  {
    res->success = false;
    res->message = "The signals on GPSw channel are too close.";
    return;
  }

  // Configに保存
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_RcRollLeft, req->roll_left) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcRollRight, req->roll_right) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcPitchUp, req->pitch_up) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcPitchDown, req->pitch_down) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcYawLeft, req->yaw_left) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcYawRight, req->yaw_right) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcThrottleUp, req->throttle_up) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcThrottleDown, req->throttle_down) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcModeProgram, req->mode_program) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcModeStabilize, req->mode_stabilize) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcModeAcrobat, req->mode_acrobat) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcEStopOn, req->estop_on) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcEStopOff, req->estop_off) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcGPSwOn, req->gpsw_on) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.set(real::kConfigKey_RcGPSwOff, req->gpsw_off) < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }
  if (property_client.save() < 0)
  {
    res->success = false;
    res->message = property_client.errorMessage();
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(RCInputCalibrationNode)
