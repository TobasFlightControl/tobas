#include <tobas_ros_tools/util.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_ros/common.hpp>

#include "../include/tobas_calibration_ros/adc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
AdcCalibrationRos::AdcCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, tobas_real_ros::kPropertyServerFC)
{
  adc_sub_ = nh_.subscribe(hal::kAdcTopic, 1, &AdcCalibrationRos::adcCb, this);
  ss_ = nh_.advertiseService(kServiceName, &AdcCalibrationRos::executeCb, this);
}

void AdcCalibrationRos::adcCb(const tobas_hal_msgs::AdcConstPtr& adc)
{
  ++cnt_;
  voltage_sum_.add(adc->voltage);
}

bool AdcCalibrationRos::executeCb(SrvType::Request& req, SrvType::Response& res)
{
  // 入力電圧のチェック
  if (req.voltage <= 0.)
  {
    res.success = false;
    res.message = "Battery voltage must be positive.";
    return true;
  }

  // 初期化
  cnt_ = 0;
  voltage_sum_.reset();

  // 一時的にADCの購読を開始
  const auto adc_sub = nh_.subscribe(hal::kAdcTopic, 1, &AdcCalibrationRos::adcCb, this);

  // データが溜まるまで待機
  if (!tobas_ros::spinUntil([this]() { return cnt_ == kDataCount; }, kTimeout))
  {
    res.success = false;
    res.message = "Timeout before ADC data collection is completed.";
    return true;
  }

  // 係数を計算
  const auto voltage_mean = voltage_sum_.get() / kDataCount;
  res.coefficient = req.voltage / voltage_mean;

  // 設定ファイルに係数を書き込む
  if (property_client_.set(tobas_real_ros::kConfigKey_AdcVoltageCoef, res.coefficient) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.save() < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }

  res.success = true;
  res.message = "";

  return true;
}
}  // namespace tobas_calibration
