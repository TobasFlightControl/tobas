#include <tobas_std_tools/array.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_calibration_ros/accel_calibration.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_calibration
{
AccelCalibrationRos::AccelCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, tobas_navio_ros::kPropertyServerFC), rate_(kSamplingRate)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

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
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  const Vector3f acc_top = readAccel();
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3f acc_offset = acc_top - Vector3f(0., 0., tobas::kGravity);

  // Configに保存
  if (property_client_.set(tobas_navio_ros::kConfigKey_AccOffsetX, acc_offset.x()) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_navio_ros::kConfigKey_AccOffsetY, acc_offset.y()) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_navio_ros::kConfigKey_AccOffsetZ, acc_offset.z()) < 0)
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
  res.acc_raw.data = acc_top.cast<double>();

  return true;
}
}  // namespace tobas_calibration
