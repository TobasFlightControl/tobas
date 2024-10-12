#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_group.hpp>

#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/mag_calibration/widget.hpp"
#include "tobas_hardware_setup/mag_calibration/method.hpp"
#include "tobas_hardware_setup/constants.hpp"

using namespace std;
using namespace Eigen;
namespace fs = filesystem;

namespace gui
{
namespace hardware_setup
{
MagCalibrationWidget::MagCalibrationWidget(rclcpp::Node::SharedPtr node)
  : node_(node), rviz_manager_("rviz_mag_calibration")
{
}

const char* MagCalibrationWidget::name() const
{
  return "Magnetometer\nCalibration";
}

const char* MagCalibrationWidget::title() const
{
  return "Calibrate Magnetometer";
}

void MagCalibrationWidget::onInit()
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Start\" button.\n\n"
    "2. For each of the 6 faces of the FC, "
    "slowly rotate the FC twice around the direction of gravity with the face pointing upwards.\n\n"
    "3. Confirm that the point cloud forms a neat ellipsoid on the screen below.\n\n"
    "4. Press \"Finish\" button.\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  cols->addWidget(start_button_);

  finish_button_ = new QPushButton("Finish");
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setEnabled(false);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  cols->addWidget(finish_button_);

  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  cols->addWidget(cancel_button_);

  cols->addStretch();

  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto rviz_config_path = pkg_path / "config/mag_calibration.rviz";
  rviz_manager_.initialize(QString::fromStdString(rviz_config_path));
  rows_->addWidget(rviz_manager_.frame());

  rows_->addStretch();

  const auto manager = rviz_manager_.frame()->getManager();

  // 固定フレームを設定
  // TFが出ているフレームでなければならない
  manager->setFixedFrame(tobas::kWorldFrame);

  // Point表示用ディスプレイを取得
  const auto display = manager->getRootDisplayGroup()->getDisplayAt(0);
  TOBAS_CHECK(display->getName() == "PointStamped");

  // Rvizのトピックを指定
  display->subProp("Topic")->setValue(kRvizPointTopic);
  point_pub_ = ros2::createPublisher<geometry_msgs::msg::PointStamped>(node_, kRvizPointTopic);

  // データバッファ関連
  history_length_ = display->subProp("History Length");

  setEnabled(false);
}

void MagCalibrationWidget::setNamespace(const string& ns)
{
  ns_ = ns;

  reset();

  arming_ = nullptr;
  arming_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kArmingTopic), &self::armingCb, this);

  setEnabled(true);
}

void MagCalibrationWidget::reset()
{
  mag_raw_sub_ = nullptr;

  history_length_->setValue(0);

  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
}

void MagCalibrationWidget::magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw)
{
  // 最初のデータからスケールを決定
  if (cnt_ == 0)
  {
    mag_norm_ = mag_raw->magnetic_field.norm();
    if (mag_norm_ == 0.)
    {
      RCLCPP_WARN(node_->get_logger(), "The first magnetic field is zero.");
      return;
    }
  }

  // データを追加
  mag_data_.at(cnt_++ % kMaxDataSize) = mag_raw->magnetic_field.data;

  // 表示用メッセージを発行
  auto point_msg = make_unique<geometry_msgs::msg::PointStamped>();
  point_msg->header = mag_raw->header;
  point_msg->header.frame_id = tobas::kWorldFrame;  // Rvizの設定の"Global Options/Fixed Frame"と一致させる
  kdl::pointKDLToMsg(mag_raw->magnetic_field / mag_norm_ * kRvizPointScale, point_msg->point);
  point_pub_->publish(std::move(point_msg));
}

void MagCalibrationWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void MagCalibrationWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  // カウンターをリセット
  cnt_ = 0;

  // 一時的にトピック通信を開始
  mag_raw_sub_ = ros2::createSubscriber(node_, ns_ + "/" + hal::kMagTopic, &self::magCb, this);

  history_length_->setValue(kMaxDataSize);

  start_button_->setEnabled(false);
  finish_button_->setEnabled(true);
  cancel_button_->setEnabled(true);

  qt::qInfoBox(this, "Magnetometer calibration is started.");
}

void MagCalibrationWidget::onCancelButtonClicked()
{
  qt::qInfoBox(this, "Magnetometer calibration is cancelled.");
  reset();
}

void MagCalibrationWidget::onFinishButtonClicked()
{
  const auto size = min(cnt_, kMaxDataSize);
  if (size < kMinDataSize)
  {
    qt::qErrorBox(this, "The number of collected samples is too small.");
    reset();
    return;
  }

  // TODO: 外れ値の除去
  // TODO: データが均一になるように間引く
  // TODO: データがきれいな楕円体を描いているかどうかをチェック

  // データを整理
  VectorXd x(size), y(size), z(size);
  for (int i = 0; i < size; ++i)
  {
    x(i) = mag_data_[i].x();
    y(i) = mag_data_[i].y();
    z(i) = mag_data_[i].z();
  }
  const VectorXd xx = x.cwiseProduct(x);
  const VectorXd yy = y.cwiseProduct(y);
  const VectorXd zz = z.cwiseProduct(z);
  const VectorXd xy = x.cwiseProduct(y);
  const VectorXd yz = y.cwiseProduct(z);
  const VectorXd zx = z.cwiseProduct(x);

  constexpr auto kCalibMethod = BOUNDING;  // TODO: 手法を選べるようにする

  // 楕円体の係数を求める
  if (kCalibMethod == BOUNDING)
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
    VectorXd ce0(size);
    ce0.fill(-mag_trans_.c);

    if (kCalibMethod == SPHERE_FITTING)
    {
      // 球体でフィッティング．
      // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
      MatrixXd CE(size, 4);
      CE << xx + yy + zz, x, y, z;
      const Vector4d coefs = CE.bdcSvd(ComputeThinU | ComputeThinV).solve(ce0);

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
    else if (kCalibMethod == ELLIPSE_FITTING)
    {
      // 楕円体でフィッティング．球より精密だが過学習のリスクがある．
      // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
      MatrixXd CE(size, 9);
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
      throw;
    }
  }

  // 楕円体であることを確認
  if (!mag_trans_.initialize())
  {
    qt::qErrorBox(this, "The estimated coefficients do not satisfy the conditions necessary for forming an ellipsoid.");
    reset();
    return;
  }

  // パラメータを作成
  vector<double> params(real::handler::mag::kParamSize);
  params.at(real::handler::mag::kAxxChannel) = mag_trans_.a_xx;
  params.at(real::handler::mag::kAyyChannel) = mag_trans_.a_yy;
  params.at(real::handler::mag::kAzzChannel) = mag_trans_.a_zz;
  params.at(real::handler::mag::kAxyChannel) = mag_trans_.a_xy;
  params.at(real::handler::mag::kAyzChannel) = mag_trans_.a_yz;
  params.at(real::handler::mag::kAzxChannel) = mag_trans_.a_zx;
  params.at(real::handler::mag::kBxChannel) = mag_trans_.b_x;
  params.at(real::handler::mag::kByChannel) = mag_trans_.b_y;
  params.at(real::handler::mag::kBzChannel) = mag_trans_.b_z;
  params.at(real::handler::mag::kCChannel) = mag_trans_.c;

  // パラメータを更新
  ros2::SyncParamClient param_client(node_, ns_ + "/magnetometer_handler");
  if (!param_client.setParam(real::handler::kParamName, params, kSetParamTimeout))
  {
    qt::qErrorBox(this, "Failed to send calibration results.");
    reset();
    return;
  }

  qt::qInfoBox(this, "Magnetometer calibration finished successfully.");
  reset();
}
}  // namespace hardware_setup
}  // namespace gui
