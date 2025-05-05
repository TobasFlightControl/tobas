#include "tobas_hardware_setup/accel_calibration/thread.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_real_msgs/srv/set_imu_params.hpp>

#include "tobas_hardware_setup/constants.hpp"
#include "tobas_hardware_setup/util.hpp"

namespace gui
{
namespace hw
{
AccelCalibrationThread::AccelCalibrationThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void AccelCalibrationThread::run()
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  if (!getAccelMean(acc_top_)) {
    return;
  }

  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Eigen::Vector3d acc_offset = acc_top_ - Eigen::Vector3d(0, 0, tobas_std::kGravity);

  // パラメータを作成
  const auto req = std::make_shared<tobas_real_msgs::srv::SetImuParams::Request>();
  req->offset_x = acc_offset.x();
  req->offset_y = acc_offset.y();
  req->offset_z = acc_offset.z();

  // パラメータを更新
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetImuParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::handler::imu::kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout)) {
    Q_EMIT finished(false, "Failed to send calibration results.");
    return;
  }

  // 結果を確認
  const auto res = sc.getResponse();
  if (!res->success) {
    Q_EMIT finished(false, "Calibration results are rejected: " + QString::fromStdString(res->message));
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
  for (auto& sum : acc_sum_) {
    sum.reset();
  }

  // 一時的にIMUの購読を開始
  auto imu_sub =
    ros2::createSubscriber(node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::kImuTopic), &self::imuCb, this);

  // データが溜まるまで待機
  if (!sleepUntil(node_, [this]() { return cnt_ >= kDataCount; }, kCollectDataTimeout)) {
    if (cnt_ == 0) {
      Q_EMIT finished(false, "IMU data is not received.");
    }
    else {
      Q_EMIT finished(false, "Timeout before IMU data collection is completed.");
    }
    return false;
  }

  // IMUの購読を終了
  imu_sub.reset();

  // 平均を計算
  for (size_t i = 0; i < 3; ++i) {
    des(i) = acc_sum_.at(i).get() / cnt_;
  }

  return true;
}

void AccelCalibrationThread::imuCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw)
{
  ++cnt_;
  for (size_t i = 0; i < 3; ++i) {
    acc_sum_.at(i).add(imu_raw->imu.accel(i));
  }
}
}  // namespace hw
}  // namespace gui
