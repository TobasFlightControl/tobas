#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_real_msgs/srv/set_battery_params.hpp>

#include "tobas_hardware_setup/adc_calibration/thread.hpp"
#include "tobas_hardware_setup/constants.hpp"
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
  auto adc_sub = ros2::createSubscriber(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, hal::kADCTopic), &ADCCalibrationThread::adcCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(node_, [this]() { return cnt_ >= kDataCount; }, kCollectDataTimeout))
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
  const auto req = std::make_shared<tobas_real_msgs::srv::SetBatteryParams::Request>();
  req->voltage_coef = voltage_coef;
  req->current_coef = 1.;  // TODO

  // パラメータを更新
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetBatteryParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::handler::adc::kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout))
  {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  // 結果を確認
  const auto res = sc.getResponse();
  if (!res->success)
  {
    Q_EMIT finished(false, "Calibration results are rejected: " + QString::fromStdString(res->message));
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
