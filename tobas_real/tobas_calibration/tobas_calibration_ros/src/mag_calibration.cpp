#include <Eigen/SVD>
#include <geometry_msgs/PointStamped.h>

#include <tobas_math/core.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_calibration_ros/mag_calibration.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_calibration
{
MagCalibrationRos::MagCalibrationRos(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), pt_(tobas_navio_ros::kConfigPath)
{
  imu_.initialize();

  collect_data_timer_ = nh_.createTimer(kSamplingRate, &self::collectDataTimerCb, this, false, false);

  mag_pub_ = nh_.advertise<geometry_msgs::PointStamped>(kMagTopicName, 1);

  start_ss_ = nh_.advertiseService(kStartServiceName, &self::startServiceCb, this);
  finish_ss_ = nh_.advertiseService(kFinishServiceName, &self::finishServiceCb, this);
  cancel_ss_ = nh_.advertiseService(kCancelServiceName, &self::cancelServiceCb, this);
}

void MagCalibrationRos::collectDataTimerCb(const ros::TimerEvent& event)
{
  // データサイズが最大値以上ならば強制終了
  if (mag_data_.size() >= kMaxDataCount)
  {
    TOBAS_ERROR("The size of magnetic field vectors is over maximum. Data collection is stopped.");
    mag_data_.clear();
    collect_data_timer_.stop();
    return;
  }

  // 地磁気センサの値を取得
  imu_.update();
  imu_.readMagnetometer(&mx_, &my_, &mz_);

  // データを追加
  mag_data_.emplace_back(mx_, my_, mz_);

  // センサ値を発行
  const auto mag_msg = boost::make_shared<geometry_msgs::PointStamped>();
  mag_msg->header.stamp = event.current_real;
  mag_msg->header.frame_id = tobas::kNavioFrame;
  mag_msg->point.x = mx_;
  mag_msg->point.y = my_;
  mag_msg->point.z = mz_;
  mag_pub_.publish(mag_msg);
}

bool MagCalibrationRos::startServiceCb(StartSrvType::Request&, StartSrvType::Response& res)
{
  if (!imu_.probe())
  {
    res.success = false;
    res.message = "IMU not enabled.";
    return true;
  }

  // 既にデータ収集が始まっている場合も再スタートする
  mag_data_.clear();
  collect_data_timer_.start();

  res.success = true;
  return true;
}

bool MagCalibrationRos::finishServiceCb(FinishSrvType::Request& req, FinishSrvType::Response& res)
{
  if (!collect_data_timer_.hasStarted())
  {
    res.success = false;
    res.message = "Data collecting timer is not running.";
    return true;
  }

  // TODO: 外れ値の除去
  // TODO: データが均一になるように間引く
  // TODO: データがきれいな楕円体を描いているかどうかをチェック

  // データを整理
  const auto size = mag_data_.size();
  VectorXf x(size), y(size), z(size);
  for (size_t i = 0; i < size; ++i)
  {
    x(i) = mag_data_[i].x();
    y(i) = mag_data_[i].y();
    z(i) = mag_data_[i].z();
  }
  const VectorXf xx = x.cwiseProduct(x);
  const VectorXf yy = y.cwiseProduct(y);
  const VectorXf zz = z.cwiseProduct(z);
  const VectorXf xy = x.cwiseProduct(y);
  const VectorXf yz = y.cwiseProduct(z);
  const VectorXf zx = z.cwiseProduct(x);

  // 楕円体の係数を求める
  if (req.method == FinishSrvType::Request::BOUNDING)
  {
    // https://okasho-engineer.com/magnetic-sensor-calibration/
    const auto x_min = x.minCoeff();
    const auto x_max = x.maxCoeff();
    const auto y_min = y.minCoeff();
    const auto y_max = y.maxCoeff();
    const auto z_min = z.minCoeff();
    const auto z_max = z.maxCoeff();

    const auto x0 = (x_min + x_max) / 2;
    const auto y0 = (y_min + y_max) / 2;
    const auto z0 = (z_min + z_max) / 2;
    const auto rx = (x_max - x_min) / 2;
    const auto ry = (y_max - y_min) / 2;
    const auto rz = (z_max - z_min) / 2;
    const auto rx2 = math::sqr(rx);
    const auto ry2 = math::sqr(ry);
    const auto rz2 = math::sqr(rz);

    mag_trans_.a_xx = 1 / rx2;
    mag_trans_.a_yy = 1 / ry2;
    mag_trans_.a_zz = 1 / rz2;
    mag_trans_.a_xy = 0;
    mag_trans_.a_yz = 0;
    mag_trans_.a_zx = 0;
    mag_trans_.b_x = -2 * x0 / rx2;
    mag_trans_.b_y = -2 * y0 / ry2;
    mag_trans_.b_z = -2 * z0 / rz2;
    mag_trans_.c = math::sqr(x0) / rx2 + math::sqr(y0) / ry2 + math::sqr(z0) / rz2 - 1;
  }
  else
  {
    // 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
    // SVDは遅いが最も精度が高い: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    mag_trans_.c = -(xx + yy + zz).mean();
    VectorXf ce0(size);
    ce0.fill(-mag_trans_.c);

    if (req.method == FinishSrvType::Request::SPHERE_FITTING)
    {
      // 球体でフィッティング．
      // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
      MatrixXf CE(size, 4);
      CE << xx + yy + zz, x, y, z;
      const Vector4f coefs = CE.bdcSvd(ComputeThinU | ComputeThinV).solve(ce0);

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
    else if (req.method == FinishSrvType::Request::ELLIPSE_FITTING)
    {
      // 楕円体でフィッティング．球より精密だが過学習のリスクがある．
      // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
      MatrixXf CE(size, 9);
      CE << xx, yy, zz, 2 * xy, 2 * yz, 2 * zx, x, y, z;
      const Matrix<float, 9, 1> coefs = CE.bdcSvd(ComputeThinU | ComputeThinV).solve(ce0);

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
      res.success = false;
      res.message = "Unknown fitting method.";
      return true;
    }
  }

  // 楕円体であることを確認
  if (!mag_trans_.initialize())
  {
    res.success = false;
    res.message = "The estimated coefficients do not satisfy the conditions necessary for forming an ellipsoid.";
    return true;
  }

  // Configに保存
  pt_.load();
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAxx, mag_trans_.a_xx);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAyy, mag_trans_.a_yy);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAzz, mag_trans_.a_zz);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAxy, mag_trans_.a_xy);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAyz, mag_trans_.a_yz);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseAzx, mag_trans_.a_zx);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseBx, mag_trans_.b_x);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseBy, mag_trans_.b_y);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseBz, mag_trans_.b_z);
  pt_.put(tobas_navio_ros::kConfigKey_MagEllipseC, mag_trans_.c);
  pt_.save();

  // データ収集タイマーを停止
  collect_data_timer_.stop();

  // データを消去
  mag_data_.clear();

  res.success = true;
  return true;
}

bool MagCalibrationRos::cancelServiceCb(CancelSrvType::Request&, CancelSrvType::Response& res)
{
  // データ収集が始まっていない場合もエラーを吐かずに初期化する
  mag_data_.clear();
  collect_data_timer_.stop();

  res.success = true;
  return true;
}
}  // namespace tobas_calibration
