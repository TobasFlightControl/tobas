#include <boost/property_tree/ini_parser.hpp>
#include <Eigen/LU>

#include <dh_std_tools/fstream.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../../include/tobas_real/calibration/mag_calibration.hpp"
#include "../../../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
MagnetometerCalibrator::MagnetometerCalibrator()
{
  if (!imu_.probe())
  {
    rosthrow("Sensor not enabled.");
  }
}

void MagnetometerCalibrator::run()
{
  imu_.initialize();
  mag_.setZero();

  // 6面分のデータを取得
  getMagData();

  // 最小二乗法で楕円の方程式を推定
  // https://rikei-tawamure.com/entry/2021/10/07/211725
  const Matrix<float, kDataCount * kDirections, 1> x = mag_.col(0);
  const Matrix<float, kDataCount * kDirections, 1> y = mag_.col(1);
  const Matrix<float, kDataCount * kDirections, 1> z = mag_.col(2);
  const Matrix<float, kDataCount * kDirections, 1> xx = x.cwiseProduct(x);
  const Matrix<float, kDataCount * kDirections, 1> yy = y.cwiseProduct(y);
  const Matrix<float, kDataCount * kDirections, 1> zz = z.cwiseProduct(z);
  const Matrix<float, kDataCount * kDirections, 1> xy = x.cwiseProduct(y);
  const Matrix<float, kDataCount * kDirections, 1> yz = y.cwiseProduct(z);
  const Matrix<float, kDataCount * kDirections, 1> zx = z.cwiseProduct(x);

  Matrix<float, kDataCount * kDirections, 9> A;
  A << xx, yy, zz, 2 * xy, 2 * yz, 2 * zx, x, y, z;
  Matrix<float, kDataCount * kDirections, 1> b;
  b.fill(-1);
  const Matrix<float, 9, 1> coefs = A.fullPivLu().solve(b);

  // Configに保存
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_MagEllipseAxx, coefs(0));
  pt.put(kConfigKey_MagEllipseAyy, coefs(1));
  pt.put(kConfigKey_MagEllipseAzz, coefs(2));
  pt.put(kConfigKey_MagEllipseAxy, coefs(3));
  pt.put(kConfigKey_MagEllipseAyz, coefs(4));
  pt.put(kConfigKey_MagEllipseAzx, coefs(5));
  pt.put(kConfigKey_MagEllipseBx, coefs(6));
  pt.put(kConfigKey_MagEllipseBy, coefs(7));
  pt.put(kConfigKey_MagEllipseBz, coefs(8));
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("Calibration finished. The result is saved to '" << kConfigPath << "'.");
}

void MagnetometerCalibrator::getMagData()
{
  // TODO: データがキレイな楕円体を描いているかどうかを評価し，進捗バーを表示する (cf. ArduPilot)

  constexpr uint32_t get_data_time = kDataCount * kSleepTime / 1000000;

  // Top
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the TOP surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(0);

  // Down
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the DOWN surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(1);

  // Front
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the FRONT surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(2);

  // Back
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the BACK surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(3);

  // Left
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the LEFT surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(4);

  // Right
  cout << "Rotate the flight controller around the gravity direction TWICE in " << get_data_time
       << " seconds with the RIGHT surface facing up. Press Enter to start:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  readMag(5);
}

void MagnetometerCalibrator::readMag(uint32_t idx)
{
  for (uint32_t i = 0; i < kDataCount; ++i)
  {
    const auto row = kDataCount * idx + i;
    imu_.update();
    imu_.read_accelerometer(&mag_(row, 0), &mag_(row, 1), &mag_(row, 2));
    usleep(kSleepTime);
  }
}
}  // namespace tobas_real
