#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_navio_ros/common.hpp>

#include "../include/tobas_calibration_ros/adc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
AdcCalibrationRos::AdcCalibrationRos(ros::NodeHandle& nh)
{
  if (adc_.initialize() < 0)
    ROS_EXIT(nh, "Failed to initialize ADC driver.");

  ss_ = nh.advertiseService(kServiceName, &AdcCalibrationRos::executeCb, this);
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

  // ADCの測定値を取得
  int a2_sum = 0;
  ros::Rate rate(kSamplingRate);
  for (size_t _ = 0; _ < kDataCount; ++_)
  {
    const int a2_value = adc_.read(tobas_navio_ros::kPowerModuleVoltageChannel);
    if (a2_value <= 0)
    {
      res.success = false;
      res.message = "Failed to read power module voltage.";
      return true;
    }
    a2_sum += a2_value;
    rate.sleep();
  }

  // 係数を計算
  const auto a2_mean = static_cast<double>(a2_sum) / kDataCount;
  res.coefficient = req.voltage / a2_mean * 1e+3;
  if (res.coefficient < kValidAdcCoefMin || kValidAdcCoefMax < res.coefficient)
  {
    res.success = false;
    res.message = "Strange ADC coefficient. Check the connection and voltage of the battery.";
    return true;
  }

  // 設定ファイルに係数を書き込む
  tobas_std::PropertyTree pt(tobas_navio_ros::kConfigPath);
  pt.put(tobas_navio_ros::kConfigKey_AdcCoef, res.coefficient);
  pt.save();

  res.success = true;
  return true;
}
}  // namespace tobas_calibration
