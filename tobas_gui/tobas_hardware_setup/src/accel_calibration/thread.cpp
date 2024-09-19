#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_common/constants.hpp>

#include "tobas_hardware_setup/accel_calibration/thread.hpp"
#include "tobas_hardware_setup/util.hpp"

namespace gui
{
namespace hardware_setup
{
AccelCalibrationThread::AccelCalibrationThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void AccelCalibrationThread::run()
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  if (!getAccelMean(acc_top_))
  {
    Q_EMIT finished(false, "Timeout before accel data collection is completed.");
    return;
  }
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Eigen::Vector3d acc_offset = acc_top_ - Eigen::Vector3d(0, 0, tobas_std::kGravity);

  // パラメータを作成
  std::vector<double> params(real::handler::imu::kParamSize);
  params.at(real::handler::imu::kOffsetXChannel) = acc_offset.x();
  params.at(real::handler::imu::kOffsetYChannel) = acc_offset.y();
  params.at(real::handler::imu::kOffsetZChannel) = acc_offset.z();

  // パラメータを更新
  ros2::SyncParamClient param_client(node_, ns_ + "/imu_handler");
  if (!param_client.setParam(real::handler::kParamName, params))
  {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  Q_EMIT finished(true, "Accel calibration finished successfully.");
}

void AccelCalibrationThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

bool AccelCalibrationThread::getAccelMean(Eigen::Vector3d& des)
{
  // 初期化
  cnt_ = 0;
  for (auto& sum : acc_sum_)
    sum.reset();

  // 一時的にIMUの購読を開始
  auto imu_sub = ros2::createSubscriber(node_, ns_ + "/" + hal::kImuTopic, &self::imuCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(node_, [this]() { return cnt_ >= kDataCount; }, kTimeout))
    return false;

  // IMUの購読を終了
  imu_sub.reset();

  // 平均を計算
  for (size_t i = 0; i < 3; ++i)
    des(i) = acc_sum_.at(i).get() / cnt_;

  return true;
}

void AccelCalibrationThread::imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw)
{
  ++cnt_;
  for (size_t i = 0; i < 3; ++i)
    acc_sum_.at(i).add(imu_raw->accel(i));
}
}  // namespace hardware_setup
}  // namespace gui
