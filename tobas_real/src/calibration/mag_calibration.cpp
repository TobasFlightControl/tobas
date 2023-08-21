#include <iostream>
#include <boost/property_tree/ini_parser.hpp>
#include <Eigen/SVD>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/fstream.hpp>
#include <dh_eigen_tools/linalg.hpp>

#include "../../include/tobas_real/calibration/mag_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_real
{
MagnetometerCalibrator::MagnetometerCalibrator() : mag_data_(kDataCount * kDirections, 3)
{
  imu_.initialize();
  if (!imu_.probe())
  {
    throw runtime_error("IMU not enabled.");
  }
}

void MagnetometerCalibrator::run(const std::string& method)
{
  mag_data_.setZero();

  // 6面分のデータを取得
  getMagData();
  const Matrix<double, kDataCount * kDirections, 1> x = mag_data_.col(0);
  const Matrix<double, kDataCount * kDirections, 1> y = mag_data_.col(1);
  const Matrix<double, kDataCount * kDirections, 1> z = mag_data_.col(2);
  const Matrix<double, kDataCount * kDirections, 1> xx = x.cwiseProduct(x);
  const Matrix<double, kDataCount * kDirections, 1> yy = y.cwiseProduct(y);
  const Matrix<double, kDataCount * kDirections, 1> zz = z.cwiseProduct(z);
  const Matrix<double, kDataCount * kDirections, 1> xy = x.cwiseProduct(y);
  const Matrix<double, kDataCount * kDirections, 1> yz = y.cwiseProduct(z);
  const Matrix<double, kDataCount * kDirections, 1> zx = z.cwiseProduct(x);

  if (method == "bounding")
  {
    // https://okasho-engineer.com/magnetic-sensor-calibration/
    const double x_min = x.minCoeff();
    const double x_max = x.maxCoeff();
    const double y_min = y.minCoeff();
    const double y_max = y.maxCoeff();
    const double z_min = z.minCoeff();
    const double z_max = z.maxCoeff();

    const double x0 = (x_min + x_max) / 2;
    const double y0 = (y_min + y_max) / 2;
    const double z0 = (z_min + z_max) / 2;
    const double rx = (x_max - x_min) / 2;
    const double ry = (y_max - y_min) / 2;
    const double rz = (z_max - z_min) / 2;
    const double rx2 = dh_std::sqr(rx);
    const double ry2 = dh_std::sqr(ry);
    const double rz2 = dh_std::sqr(rz);

    mag_trans_.a_xx = 1 / rx2;
    mag_trans_.a_yy = 1 / ry2;
    mag_trans_.a_zz = 1 / rz2;
    mag_trans_.a_xy = 0;
    mag_trans_.a_yz = 0;
    mag_trans_.a_zx = 0;
    mag_trans_.b_x = -2 * x0 / rx2;
    mag_trans_.b_y = -2 * y0 / ry2;
    mag_trans_.b_z = -2 * z0 / rz2;
    mag_trans_.c = dh_std::sqr(x0) / rx2 + dh_std::sqr(y0) / ry2 + dh_std::sqr(z0) / rz2 - 1;
  }
  else
  {
    // 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
    // SVDは遅いが最も精度が高い: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    mag_trans_.c = -(xx + yy + zz).mean();
    VectorXd ce0(kDataCount * kDirections);  // メモリ制限回避のため可変サイズで定義
    ce0.fill(-mag_trans_.c);

    if (method == "sphere_fitting")
    {
      // 球体でフィッティング．
      // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
      MatrixXd CE(kDataCount * kDirections, 4);
      CE << xx + yy + zz, x, y, z;
      const Matrix<double, 4, 1> coefs = CE.bdcSvd(ComputeThinU | ComputeThinV).solve(ce0);

      mag_trans_.a_xx = coefs(0);
      mag_trans_.a_yy = coefs(0);
      mag_trans_.a_zz = coefs(0);
      mag_trans_.a_xy = 0;
      mag_trans_.a_yz = 0;
      mag_trans_.a_zx = 0;
      mag_trans_.b_x = coefs(1);
      mag_trans_.b_y = coefs(2);
      mag_trans_.b_z = coefs(3);
    }
    else if (method == "ellipse_fitting")
    {
      // 楕円体でフィッティング．球より精密だが過学習のリスクがある．
      // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
      MatrixXd CE(kDataCount * kDirections, 9);
      CE << xx, yy, zz, 2 * xy, 2 * yz, 2 * zx, x, y, z;
      const Matrix<double, 9, 1> coefs = CE.bdcSvd(ComputeThinU | ComputeThinV).solve(ce0);

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
      throw runtime_error("Invalid method: " + method);
      return;
    }
  }

  // 推定された係数を表示
  cout << "Estimated coefficients:\n" << mag_trans_ << endl;

  // 楕円体の射影クラスの初期化に成功したら有効な係数だと言える
  try
  {
    mag_trans_.initialize();
  }
  catch (const exception& e)
  {
    throw runtime_error("Magnetometer calibration failed: " + string(e.what()));
  }

  // 中心と半径を表示
  cout << "Center:" << mag_trans_.getCenter().transpose() << endl;
  cout << "Radius:" << mag_trans_.getRadius().transpose() << endl;

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
  cout << "Calibration finished. The result is saved to '" << kConfigPath << "'." << endl;
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
  RowVector3f tmp;
  for (uint32_t i = 0; i < kDataCount; ++i)
  {
    imu_.update();
    imu_.read_magnetometer(&tmp(0), &tmp(1), &tmp(2));

    const auto row = kDataCount * idx + i;
    mag_data_.block(row, 0, 1, 3) = tmp.cast<double>();
    cout << "Magnetic field: " << mag_data_.block(row, 0, 1, 3) << endl;

    usleep(kSleepTime);
  }
}
}  // namespace tobas_real
