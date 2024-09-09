#include <tobas_algorithm/kahan.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/action/adc_calibration.hpp>

#include "./util.hpp"

using namespace std;

class AdcCalibrationNode : public tobas::BaseNode
{
  static constexpr size_t kDataCount = 100;
  static constexpr double kTimeout = 5.;  // [s]

  using self = AdcCalibrationNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_calibration_msgs::action::ADCCalibration;

public:
  explicit AdcCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_;
  algo::Kahan<double> voltage_sum_;

  ros2::ActionPtr<ActionType> as_;

  void adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

AdcCalibrationNode::AdcCalibrationNode(const rclcpp::NodeOptions& options) : super("adc_calibration", options)
{
  as_ = createAction<ActionType>(tobas::kADCCalibAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

void AdcCalibrationNode::adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc)
{
  ++cnt_;
  voltage_sum_.add(adc->voltage);
}

rclcpp_action::GoalResponse
AdcCalibrationNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr goal)
{
  if (goal->voltage <= 0.)
  {
    TOBAS_ERROR("Battery voltage must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AdcCalibrationNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AdcCalibrationNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("ADC calibration is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // 初期化
  cnt_ = 0;
  voltage_sum_.reset();

  // 一時的にADCの購読を開始
  auto adc_sub = createSubscriber(hal::kAdcTopic, &AdcCalibrationNode::adcCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(shared_from_this(), [this]() { return cnt_ >= kDataCount; }, kTimeout))
  {
    result->message = "Timeout before ADC data collection is completed.";
    goal_handle->abort(result);
    return;
  }

  // ADCの購読を終了
  adc_sub.reset();

  // 係数を計算
  const auto voltage_mean = voltage_sum_.get() / cnt_;
  const auto coefficient = goal_handle->get_goal()->voltage / voltage_mean;

  // 設定ファイルに係数を書き込む
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_AdcVoltageCoef, coefficient) < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }
  if (property_client.save() < 0)
  {
    result->message = property_client.errorMessage();
    goal_handle->abort(result);
    return;
  }

  result->coefficient = coefficient;
  result->message.clear();
  goal_handle->succeed(result);
}

RCLCPP_COMPONENTS_REGISTER_NODE(AdcCalibrationNode)
