#include <boost/property_tree/ini_parser.hpp>
#include <Eigen/LU>

#include <dh_std_tools/fstream.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_ros_tools/rosparam.hpp>
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
  const Matrix<float, kDataCount * kDirections, 1> x = mag_.col(0);
  const Matrix<float, kDataCount * kDirections, 1> y = mag_.col(1);
  const Matrix<float, kDataCount * kDirections, 1> z = mag_.col(2);
  const Matrix<float, kDataCount * kDirections, 1> xx = x.cwiseProduct(x);
  const Matrix<float, kDataCount * kDirections, 1> yy = y.cwiseProduct(y);
  const Matrix<float, kDataCount * kDirections, 1> zz = z.cwiseProduct(z);
  const Matrix<float, kDataCount * kDirections, 1> xy = x.cwiseProduct(y);
  const Matrix<float, kDataCount * kDirections, 1> yz = y.cwiseProduct(z);
  const Matrix<float, kDataCount * kDirections, 1> zx = z.cwiseProduct(x);

  // 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
  mag_trans_.c = 1.;  // TODO: x,y,zのスケールに依って決める
  Matrix<float, kDataCount * kDirections, 1> ce0;
  ce0.fill(-mag_trans_.c);

  if (method_ == "sphere")
  {
    // 球体でフィッティング．
    // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
    Matrix<float, kDataCount * kDirections, 4> CE;
    CE << xx + yy + zz, x, y, z;
    const Matrix<float, 4, 1> coefs = CE.fullPivLu().solve(ce0);

    mag_trans_.a_xx = coefs(0);
    mag_trans_.a_yy = coefs(0);
    mag_trans_.a_zz = coefs(0);
    mag_trans_.a_xy = 0.;
    mag_trans_.a_yz = 0.;
    mag_trans_.a_zx = 0.;
    mag_trans_.b_x = coefs(1);
    mag_trans_.b_y = coefs(2);
    mag_trans_.b_z = coefs(3);
  }
  else if (method_ == "ellipse")
  {
    // 楕円体でフィッティング．球より精密だが過学習のリスクがある．
    // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
    Matrix<float, kDataCount * kDirections, 9> CE;
    CE << xx, yy, zz, 2 * xy, 2 * yz, 2 * zx, x, y, z;
    const Matrix<float, 9, 1> coefs = CE.fullPivLu().solve(ce0);

    mag_trans_.a_xx = coefs(0);
    mag_trans_.a_yy = coefs(1);
    mag_trans_.a_zz = coefs(2);
    mag_trans_.a_xy = coefs(3);
    mag_trans_.a_yz = coefs(4);
    mag_trans_.a_zx = coefs(5);
    mag_trans_.b_x = coefs(6);
    mag_trans_.b_y = coefs(7);
    mag_trans_.b_z = coefs(8);
  }
  else
  {
    rosError("Invalid method: " << method_);
    return;
  }

  // 楕円体の射影クラスの初期化に成功したら有効な係数だと言える
  try
  {
    mag_trans_.initialize();
  }
  catch (const exception& e)
  {
    rosError("Magnetometer calibration failed.");
    return;
  }

  // Configに保存
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_MagEllipseAxx, mag_trans_.a_xx);
  pt.put(kConfigKey_MagEllipseAyy, mag_trans_.a_yy);
  pt.put(kConfigKey_MagEllipseAzz, mag_trans_.a_zz);
  pt.put(kConfigKey_MagEllipseAxy, mag_trans_.a_xy);
  pt.put(kConfigKey_MagEllipseAyz, mag_trans_.a_yz);
  pt.put(kConfigKey_MagEllipseAzx, mag_trans_.a_zx);
  pt.put(kConfigKey_MagEllipseBx, mag_trans_.b_x);
  pt.put(kConfigKey_MagEllipseBy, mag_trans_.b_y);
  pt.put(kConfigKey_MagEllipseBz, mag_trans_.b_z);
  pt.put(kConfigKey_MagEllipseC, mag_trans_.c);
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("Calibration finished. The result is saved to '" << kConfigPath << "'.");
}

void MagnetometerCalibrator::getRosParams()
{
  dh_ros::getParam("~method", method_, kDefaultMethod);
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
    imu_.read_magnetometer(&mag_(row, 0), &mag_(row, 1), &mag_(row, 2));
    usleep(kSleepTime);
  }
}
}  // namespace tobas_real
