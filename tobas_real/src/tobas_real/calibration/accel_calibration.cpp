#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/fstream.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../../include/tobas_real/calibration/accel_calibration.hpp"
#include "../../../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
AccelCalibrator::AccelCalibrator()
{
  if (!imu_.probe())
  {
    rosthrow("Sensor not enabled.");
  }
}

void AccelCalibrator::run()
{
  imu_.initialize();

  // TODO: 6面分取得して最小二乗法で同時変換行列を推定
  // https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/accelerometer_calibration.cpp

  // Top
  cout << "Press Enter with the flight controller's TOP surface facing up:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const Vector3f acc_top = readAccel();

  // オフセットを計算
  const Vector3f acc_offset = acc_top - Vector3f(0., 0., tobas::kGravity);
  rosInfo("The estimated accelerometer offset is: " << acc_offset);

  // Configに保存
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_AccOffsetX, acc_offset.x());
  pt.put(kConfigKey_AccOffsetY, acc_offset.y());
  pt.put(kConfigKey_AccOffsetZ, acc_offset.z());
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("Calibration finished. The result is saved to '" << kConfigPath << "'.");
}

Vector3f AccelCalibrator::readAccel()
{
  // 加速度を取得
  Vector3f acc_sum = Vector3f::Zero();
  for (uint32_t _ = 0; _ < kDataCount; ++_)
  {
    imu_.read_accelerometer(&acc_.x(), &acc_.y(), &acc_.z());
    rosInfoThrottle(kShowSensorReadingPeriod, "Accelerometer reading:" << acc_);
    acc_sum += acc_;
    usleep(kSleepTime);
  }

  // 平均を計算
  const Vector3f acc_mean = acc_sum / kDataCount;
  rosInfo("Finished reading. The average value of accelerometer readings is: " << acc_mean);
  return acc_mean;
}
}  // namespace tobas_real
