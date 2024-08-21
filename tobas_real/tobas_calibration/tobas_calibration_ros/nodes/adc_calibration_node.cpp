#include <tobas_algorithm/kahan.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>

#include <tobas_real_common/constants.hpp>
#include <tobas_calibration_msgs/srv/adc_calibration.hpp>

using namespace std;

class AdcCalibrationNode : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "adc_calibration";

  static constexpr size_t kDataCount = 100;
  static constexpr double kTimeout = 5.;  // [s]

  using self = AdcCalibrationNode;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::srv::AdcCalibration;

public:
  explicit AdcCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_;
  algo::Kahan<double> voltage_sum_;

  ros2::ServicePtr<SrvType> ss_;

  void adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc);
  void executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res);
};

AdcCalibrationNode::AdcCalibrationNode(const rclcpp::NodeOptions& options) : super("adc_calibration", options)
{
  ss_ = createService<SrvType>(kServiceName, &self::executeCb, this);
}

void AdcCalibrationNode::adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc)
{
  ++cnt_;
  voltage_sum_.add(adc->voltage);
}

void AdcCalibrationNode::executeCb(const SrvType::Request::ConstSharedPtr& req, const SrvType::Response::SharedPtr& res)
{
  // 入力電圧のチェック
  if (req->voltage <= 0.)
  {
    res->success = false;
    res->message = "Battery voltage must be positive.";
    return;
  }

  // 初期化
  cnt_ = 0;
  voltage_sum_.reset();

  // 一時的にADCの購読を開始
  const auto adc_sub = createSubscriber(hal::kAdcTopic, &AdcCalibrationNode::adcCb, this);

  // データが溜まるまで待機
  if (!ros2::spinUntil(shared_from_this(), [this]() { return cnt_ == kDataCount; }, kTimeout))
  {
    res->success = false;
    res->message = "Timeout before ADC data collection is completed.";
    return;
  }

  // 係数を計算
  const auto voltage_mean = voltage_sum_.get() / kDataCount;
  res->coefficient = req->voltage / voltage_mean;

  // 設定ファイルに係数を書き込む
  ptree::PropertyClient property_client(shared_from_this(), real::kPropertyServerFC);
  if (property_client.set(real::kConfigKey_AdcVoltageCoef, res->coefficient) < 0)
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

RCLCPP_COMPONENTS_REGISTER_NODE(AdcCalibrationNode)
