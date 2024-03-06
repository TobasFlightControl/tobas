#include <tobas_std_tools/property_tree.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_calibration/accel_calibration.hpp"

#define DATA_COUNT 1000
#define SLEEP_TIME 10000  // [us]

using namespace std;
using namespace Eigen;

namespace tobas_calibration
{
AccelCalibrationRos::AccelCalibrationRos(ros::NodeHandle& nh)
{
  imu_.initialize();
  if (!imu_.probe())
    throw runtime_error("IMU not enabled.");

  ss_ = nh.advertiseService(kServiceName, &AccelCalibrationRos::executeCb, this);
}

Vector3f AccelCalibrationRos::readAccel()
{
  // 加速度を取得
  Vector3f acc_sum = Vector3f::Zero();
  for (size_t _ = 0; _ < DATA_COUNT; ++_)
  {
    imu_.update();
    imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
    acc_sum += acc_;
    usleep(SLEEP_TIME);
  }

  // 平均を計算
  const Vector3f acc_mean = acc_sum / DATA_COUNT;
  return acc_mean;
}

bool AccelCalibrationRos::executeCb(
  tobas_calibration::AccelCalibrationRequest&,
  tobas_calibration::AccelCalibrationResponse& res)
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  const Vector3f acc_top = readAccel();
  // TODO: 明らかにおかしな値だった場合は失敗を返す

  // オフセットを計算
  const Vector3f acc_offset = acc_top - Vector3f(0., 0., tobas::kGravity);

  // Configに保存
  tobas_std::PropertyTree pt(tobas_real::kConfigPath);
  pt.put(tobas_real::kConfigKey_AccOffsetX, acc_offset.x());
  pt.put(tobas_real::kConfigKey_AccOffsetY, acc_offset.y());
  pt.put(tobas_real::kConfigKey_AccOffsetZ, acc_offset.z());
  pt.save();

  res.success = true;
  res.acc_raw.data = acc_top.cast<double>();
  return true;
}
}  // namespace tobas_calibration
