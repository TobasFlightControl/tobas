#include "tobas_sensor_calibration/accel_calibration/thread.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_real_msgs/srv/set_imu_params.hpp>

#include "tobas_sensor_calibration/constants.hpp"

namespace gui
{
namespace sc
{
AccelCalibrationThread::AccelCalibrationThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge) : node_(node)
{
  connect(&bridge, &RosQtBridge::rawImuReceived, this, &self::imuCb, Qt::QueuedConnection);
}

void AccelCalibrationThread::run()
{
  // 必要なトピックが受け取れていることを確認
  if (!imu_raw_) {
    Q_EMIT finished(false, "IMU data is not received yet.");
    return;
  }

  // 初期化
  cnt_ = 0;
  for (auto& sum : acc_sum_) {
    sum.reset();
  }

  // 加速度データ加算開始
  get_data_ = true;

  // データが溜まるまで待機
  const auto clock = node_->get_clock();
  const auto start_time = clock->now();
  rclcpp::Rate rate(100., clock);
  while (rclcpp::ok()) {
    if (cnt_ >= kDataCount) {
      break;
    }
    if ((clock->now() - start_time).seconds() > kCollectDataTimeout) {
      Q_EMIT finished(false, "Timeout before IMU data collection is completed.");
      get_data_ = false;
      return;
    }
    rate.sleep();
  }

  // 加速度データ加算終了
  get_data_ = false;

  // 平均を計算
  kdl::Vector acc_mean;
  for (size_t i = 0; i < 3; ++i) {
    acc_mean(i) = acc_sum_.at(i).get() / cnt_;
  }

  // バイアスを計算
  const kdl::Vector acc_ref(0, 0, tbs::kGravity);
  const auto acc_bias = acc_mean - acc_ref;

  // バイアスが異常に大きい場合は失敗
  if (acc_bias.norm() > kAccelBiasNormThresh) {
    Q_EMIT finished(false, "Acceleration error is too high. Verify that the FMU is correctly oriented.");
    return;
  }

  // パラメータを作成
  const auto req = std::make_shared<tobas_real_msgs::srv::SetImuParams::Request>();
  req->offset_x = acc_bias.x();
  req->offset_y = acc_bias.y();
  req->offset_z = acc_bias.z();

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

  Q_EMIT finished(true, "Accelerometer calibration finished successfully.");
}

void AccelCalibrationThread::reset()
{
  imu_raw_.reset();

  get_data_ = false;
  cnt_ = 0;

  for (auto& sum : acc_sum_) {
    sum.reset();
  }
}

void AccelCalibrationThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

void AccelCalibrationThread::imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw)
{
  imu_raw_ = imu_raw;

  if (!get_data_) {
    return;
  }

  ++cnt_;
  for (size_t i = 0; i < 3; ++i) {
    acc_sum_.at(i).add(imu_raw->accel(i));
  }
}
}  // namespace sc
}  // namespace gui
