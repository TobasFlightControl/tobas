#include <tobas_std_tools/property_tree.hpp>
#include <tobas_tools/constants.hpp>

#include "../../include/tobas_real/calibration/accel_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
AccelCalibrator::AccelCalibrator()
{
  imu_.initialize();
  if (!imu_.probe())
    throw runtime_error("IMU not enabled.");
}

void AccelCalibrator::run()
{
  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  cout << "Press Enter with the flight controller's TOP surface facing up:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const Vector3f acc_top = readAccel();
  // TODO: 明らかにおかしな値だった場合は例外を出す

  // オフセットを計算
  const Vector3f acc_offset = acc_top - Vector3f(0., 0., tobas::kGravity);
  cout << "The estimated accelerometer offset is: " << acc_offset.transpose() << endl;

  // Configに保存
  tobas_std::PropertyTree pt(kConfigPath);
  pt.put(kConfigKey_AccOffsetX, acc_offset.x());
  pt.put(kConfigKey_AccOffsetY, acc_offset.y());
  pt.put(kConfigKey_AccOffsetZ, acc_offset.z());
  pt.save();
  cout << "Calibration finished. The result is saved to '" << kConfigPath << "'." << endl;
}

Vector3f AccelCalibrator::readAccel()
{
  // 加速度を取得
  Vector3f acc_sum = Vector3f::Zero();
  for (size_t _ = 0; _ < kDataCount; ++_)
  {
    imu_.update();
    imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
    acc_sum += acc_;
    usleep(kSleepTime);
  }

  // 平均を計算
  const Vector3f acc_mean = acc_sum / kDataCount;
  cout << "Finished reading. The average value of accelerometer readings is: "
       << acc_mean.transpose() << endl;
  return acc_mean;
}
}  // namespace tobas_real
