#include "tobas_sensor_calibration/mag_calibration/large_vehicle/thread.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_time_tools/util.hpp>

#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

#include "tobas_sensor_calibration/constants.hpp"

namespace gui
{
namespace sc
{
LargeVehicleMagCalibThread::LargeVehicleMagCalibThread(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node)
{
  connect(&bridge, &RosQtBridge::rawMagReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
}

void LargeVehicleMagCalibThread::run()
{
  if (!gnss_) {
    Q_EMIT finished(false, "GNSS is not received.");
    return;
  }
  if (gnss_->fix_type != tobas_msgs::msg::Gnss::FIX_3D) {
    Q_EMIT finished(false, "GNSS is not fixed.");
    return;
  }

  // 現在位置での地磁気の参照値を求める
  const auto mag =
    geomag::elementsFromGeodetic(gnss_->latitude, gnss_->longitude, gnss_->altitude, tim::yearFraction());
  const kdl::Vector mag_ref(mag.north, -mag.east, -mag.down);

  // 初期化
  cnt_ = 0;
  for (auto& sum : mag_sum_) {
    sum.reset();
  }

  // 地磁気データ加算開始
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
      if (cnt_ == 0) {
        Q_EMIT finished(false, "Magnetic field is not received.");
      }
      else {
        Q_EMIT finished(false, "Timeout before Magnetic field collection is completed.");
      }
      get_data_ = false;
      return;
    }
    rate.sleep();
  }

  // 地磁気データ加算終了
  get_data_ = false;

  // 平均を計算
  kdl::Vector mag_mean;
  for (size_t i = 0; i < 3; ++i) {
    mag_mean(i) = mag_sum_.at(i).get() / cnt_;
  }

  // バイアスを計算 (memo: 3-41)
  const auto hard_bias = mag_mean - mag_ref;
  const auto soft_bias = mag.total;

  // パラメータを作成
  const auto req = std::make_shared<tobas_real_msgs::srv::SetMagnetometerParams::Request>();
  req->hard_bias.at(0) = hard_bias.x();
  req->hard_bias.at(1) = hard_bias.y();
  req->hard_bias.at(2) = hard_bias.z();
  req->soft_bias.at(0) = soft_bias;
  req->soft_bias.at(1) = soft_bias;
  req->soft_bias.at(2) = soft_bias;
  req->soft_bias.at(3) = 0.;
  req->soft_bias.at(4) = 0.;
  req->soft_bias.at(5) = 0.;

  // パラメータを更新
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::handler::mag::kSetParamSrv));
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

  Q_EMIT finished(true, "Magnetometer calibration finished successfully.");
}

void LargeVehicleMagCalibThread::setNamespace(const std::string& ns)
{
  ns_ = ns;
}

void LargeVehicleMagCalibThread::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag_raw)
{
  if (!get_data_) {
    return;
  }

  ++cnt_;
  for (size_t i = 0; i < 3; ++i) {
    mag_sum_.at(i).add(mag_raw->mag(i));
  }
}

void LargeVehicleMagCalibThread::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  gnss_ = gnss;
}
}  // namespace sc
}  // namespace gui
