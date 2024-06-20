#include <tobas_std_tools/array.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_calibration_ros/accel_calibration.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_calibration
{
AccelCalibrationRos::AccelCalibrationRos(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), pt_(tobas_navio_ros::kConfigPath), rate_(kSamplingRate)
{
  imu_.initialize();

  ss_ = nh_.advertiseService(kServiceName, &AccelCalibrationRos::executeCb, this);
}

Vector3f AccelCalibrationRos::readAccel()
{
  // 加速度を取得
  rate_.start();
  for (size_t i = 0; i < kDataCount; ++i)
  {
    imu_.updateAccelerometer();
    imu_.readAccelerometer(&ax_[i], &ay_[i], &az_[i]);
    rate_.sleep();
  }

  // 平均を計算
  return Vector3f(tobas_std::fmean(ax_), tobas_std::fmean(ay_), tobas_std::fmean(az_));
}

bool AccelCalibrationRos::executeCb(SrvType::Request&, SrvType::Response& res)
{
  if (!imu_.probe())
  {
    res.success = false;
    res.message = "IMU not enabled.";
    return true;
  }

  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  const Vector3f acc_top = readAccel();
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3f acc_offset = acc_top - Vector3f(0., 0., tobas::kGravity);

  // Configに保存
  pt_.load();
  pt_.put(tobas_navio_ros::kConfigKey_AccOffsetX, acc_offset.x());
  pt_.put(tobas_navio_ros::kConfigKey_AccOffsetY, acc_offset.y());
  pt_.put(tobas_navio_ros::kConfigKey_AccOffsetZ, acc_offset.z());
  pt_.save();

  res.success = true;
  res.acc_raw.data = acc_top.cast<double>();
  return true;
}
}  // namespace tobas_calibration
