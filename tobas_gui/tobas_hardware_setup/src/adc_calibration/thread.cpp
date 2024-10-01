#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_common/constants.hpp>

#include "tobas_hardware_setup/adc_calibration/thread.hpp"
#include "tobas_hardware_setup/util.hpp"

namespace gui
{
namespace hardware_setup
{
ADCCalibrationThread::ADCCalibrationThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void ADCCalibrationThread::run()
{
  if (voltage_ <= 0.)
  {
    Q_EMIT finished(false, "Battery voltage must be positive.");
    return;
  }

  // 初期化
  cnt_ = 0;
  voltage_sum_.reset();

  // 一時的にADCの購読を開始
  auto adc_sub = ros2::createSubscriber(node_, ns_ + "/" + hal::kAdcTopic, &ADCCalibrationThread::adcCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(node_, [this]() { return cnt_ >= kDataCount; }, kTimeout))
  {
    if (cnt_ == 0)
      Q_EMIT finished(false, "ADC data is not received.");
    else
      Q_EMIT finished(false, "Timeout before ADC data collection is completed.");
    return;
  }

  // ADCの購読を終了
  adc_sub.reset();

  // 係数を計算
  const auto voltage_mean = voltage_sum_.get() / cnt_;
  const auto voltage_coef = voltage_ / voltage_mean;

  // パラメータを作成
  std::vector<double> params(real::handler::adc::kParamSize);
  params.at(real::handler::adc::kVoltageChannel) = voltage_coef;
  params.at(real::handler::adc::kCurrentChannel) = 1.;  // TODO

  // パラメータを更新
  ros2::SyncParamClient param_client(node_, ns_ + "/adc_handler");
  if (!param_client.setParam(real::handler::kParamName, params))
  {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  Q_EMIT finished(true, "ADC calibration finished successfully.");
}

void ADCCalibrationThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

void ADCCalibrationThread::setCurrentVoltage(double voltage)
{
  voltage_ = voltage;
}

void ADCCalibrationThread::adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc)
{
  ++cnt_;
  voltage_sum_.add(adc->voltage);
}
}  // namespace hardware_setup
}  // namespace gui
