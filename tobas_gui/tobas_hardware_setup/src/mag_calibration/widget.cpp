#include "tobas_hardware_setup/mag_calibration/widget.hpp"

#include <filesystem>
#include <ranges>

#include <QDebug>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_eigen_conversions/eigen_msg.hpp>
#include <tobas_eigen_tools/hash.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/core.hpp>
#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>

#include "tobas_hardware_setup/constants.hpp"
#include "tobas_hardware_setup/mag_calibration/method.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace hw
{
MagCalibrationWidget::MagCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), rviz_manager_("rviz_mag_calibration")
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Start\" button.\n\n"
    "2. For each of the 6 faces of the FC, "
    "slowly rotate the FC twice around the direction of gravity with the face pointing upwards.\n\n"
    "3. Confirm that the point cloud forms a neat ellipsoid on the screen below.\n\n"
    "4. Press \"Finish\" button.\n\n",
    kBodyPSize);

  start_button_ = new QPushButton("Start");
  finish_button_ = new QPushButton("Finish");
  cancel_button_ = new QPushButton("Cancel");

  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);

  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  progress_bar_ = new QProgressBar();

  face_circles_.at(kTopIdx) = new FaceCircleWidget("Top");
  face_circles_.at(kBottomIdx) = new FaceCircleWidget("Bottom");
  face_circles_.at(kFrontIdx) = new FaceCircleWidget("Front");
  face_circles_.at(kBackIdx) = new FaceCircleWidget("Back");
  face_circles_.at(kLeftIdx) = new FaceCircleWidget("Left");
  face_circles_.at(kRightIdx) = new FaceCircleWidget("Right");

  const fs::path pkg_path(ament_index_cpp::get_package_share_directory(kPackageName));
  const auto rviz_config_path = pkg_path / "config/mag_calibration.rviz";
  rviz_manager_.initialize(QString::fromStdString(rviz_config_path));

  // 固定フレームを設定
  // TFが出ているフレームでなければならない
  rviz_manager_.setFixedFrame(tobas::kWorldFrame);

  const auto ps_display = rviz_manager_.getDisplay("PointStamped");
  ps_display->subProp("Topic")->setValue(kRvizPointStampedTopic);
  ps_history_length_ = ps_display->subProp("History Length");

  const auto pc_display = rviz_manager_.getDisplay("PointCloud");
  pc_display->subProp("Topic")->setValue(kRvizPointCloudTopic);

  ps_pub_ = ros2::createPublisher<geometry_msgs::msg::PointStamped>(node_, kRvizPointStampedTopic);
  pc_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kRvizPointCloudTopic, false, true);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(start_button_);
  button_cols->addWidget(finish_button_);
  button_cols->addWidget(cancel_button_);
  button_cols->addStretch();

  const auto face_cols = new QHBoxLayout();
  for (const auto& face_circle : face_circles_) {
    face_cols->addWidget(face_circle);
  }

  rows_->addWidget(instruction);
  rows_->addLayout(button_cols);
  rows_->addWidget(progress_bar_);
  rows_->addLayout(face_cols, 1);
  rows_->addWidget(rviz_manager_.widget(), 4);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  connect(&bridge, &RosQtBridge::rawMagReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);

  reset();
}

const char* MagCalibrationWidget::name() const
{
  return "Magnetometer\nCalibration";
}

const char* MagCalibrationWidget::title() const
{
  return "Calibrate Magnetometer";
}

void MagCalibrationWidget::reset()
{
  resetToPreStart();

  rviz_manager_.resetTime();

  arming_.reset();
  odom_.reset();
}

void MagCalibrationWidget::setNamespace(const std::string& ns)
{
  reset();

  ns_ = ns;
}

void MagCalibrationWidget::paintEvent(QPaintEvent*)
{
  // 最小のポイントサイズを決める
  auto psize = INT32_MAX;
  for (const auto& face_circle : face_circles_) {
    psize = std::min(psize, face_circle->calcMaxTextPointSize());
  }

  // 最小のポイントサイズに揃える
  for (const auto& face_circle : face_circles_) {
    face_circle->setTextPointSize(psize);
  }
}

void MagCalibrationWidget::resetToPreStart()
{
  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  progress_bar_->setValue(0);

  for (const auto& face_circle : face_circles_) {
    face_circle->setProgress(0.);
    face_circle->setSelected(false);
  }

  running_ = false;
  cnt_ = 0;
  rot_angles_.fill(0.);
  completed_.fill(false);
}

int MagCalibrationWidget::numActiveSamples() const
{
  return std::count(active_.begin(), active_.begin() + cnt_, true);
}

size_t MagCalibrationWidget::computeFaceIndex() const
{
  const auto& R_W_B = odom_->frame.M;

  // 世界座標系から見た各軸のZ成分を取得
  const auto axz = R_W_B.axisX().z();
  const auto ayz = R_W_B.axisY().z();
  const auto azz = R_W_B.axisZ().z();
  const auto abs_axz = fabs(axz);
  const auto abs_ayz = fabs(ayz);
  const auto abs_azz = fabs(azz);

  // 要素の大小関係から現在上を向いている面を決定
  if (abs_axz >= std::max(abs_ayz, abs_azz)) {
    if (axz > 0.) {
      return kFrontIdx;
    }
    else {
      return kBackIdx;
    }
  }
  else if (abs_ayz >= std::max(abs_azz, abs_axz)) {
    if (ayz > 0.) {
      return kLeftIdx;
    }
    else {
      return kRightIdx;
    }
  }
  else if (abs_azz >= std::max(abs_axz, abs_ayz)) {
    if (azz > 0.) {
      return kTopIdx;
    }
    else {
      return kBottomIdx;
    }
  }
  else {
    throw std::runtime_error("Impossible orientation.");
  }
}

void MagCalibrationWidget::subsample()
{
  static constexpr int N = 100;
  std::unordered_map<Eigen::Vector3i, std::set<int>> groups;

  // バウンディングボックスを求める
  auto lb = kdl::Vector::Constant(std::numeric_limits<double>::max());
  auto ub = kdl::Vector::Constant(std::numeric_limits<double>::lowest());
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    lb = lb.min(p);
    ub = ub.max(p);
  }
  const auto d = (ub - lb) / N;  // 1つの領域の幅

  // 各点をグリッドに割り当てる
  Eigen::Vector3i gi;  // Grid index
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    for (int i = 0; i < 3; ++i) {
      gi(i) = std::clamp(static_cast<int>((p(i) - lb(i)) / d(i)), 0, N - 1);
    }
    groups[gi].insert(pi);
  }

  // 同じ領域に複数の点が存在する場合は平均でまとめる
  for (const auto& [_, group] : groups) {
    const auto group_size = group.size();
    if (group_size < 2) {
      continue;
    }

    // 同じ領域に属する点の平均を計算
    auto sum = kdl::Vector::Zero();
    for (const auto& pi : group) {
      sum += buf_.at(pi);
    }
    const auto mean = sum / group_size;

    // 最初の要素に平均値を入れ，それ以外を無効化する
    for (const auto& [i, pi] : std::views::enumerate(group)) {
      if (i == 0) {
        buf_.at(pi) = mean;
      }
      else {
        TOBAS_CHECK(active_.at(pi));
        active_.at(pi) = false;
      }
    }
  }
}

void MagCalibrationWidget::removeOutliers()
{
  const auto size = numActiveSamples();

  // 平均点を求める
  auto sum = kdl::Vector::Zero();
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    sum += buf_.at(pi);
  }
  const auto mean = sum / size;

  // 平均点からの距離の標準偏差を求める
  double dist2_sum = 0.;
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    const auto dist2 = (p - mean).squaredNorm();
    dist2_sum += dist2;
  }
  const auto dist_var = dist2_sum / size;
  const auto dist_stddev = sqrt(dist_var);

  // 平均点からの距離が大きく外れているものを弾く
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    const auto dist = (p - mean).norm();
    if (dist > dist_stddev * 3) {  // Zスコア法の典型的な閾値
      qWarning() << "Point (" << p.x() << ", " << p.y() << ", " << p.z() << ") was identified as an outlier.";
      active_.at(pi) = false;
    }
  }
}

void MagCalibrationWidget::onStartButtonClicked()
{
  // 必要なトピックが受け取れていることを確認
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (!odom_) {
    qt::qWarnBox(
      this,
      "This operation cannot be performed because the odometry is not received yet. "
      "Please check whether the accelerometer has been calibrated.");
    return;
  }

  // アームされていないことを確認
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  // 一度クリアしてから描画する点の個数を設定
  // FIXME: History Lengthの最小値は1であり，この方法でクリアしようとしても前のデータが1つ残ってしまう．
  ps_history_length_->setValue(1);
  ps_history_length_->setValue(kMaxDataSize);

  // 空の点群を発行することでキャリブレーション後の描画をリセット
  auto pc_empty = std::make_unique<sensor_msgs::msg::PointCloud>();
  pc_empty->header.stamp = node_->get_clock()->now();
  pc_empty->header.frame_id = tobas::kWorldFrame;
  pc_pub_->publish(std::move(pc_empty));

  start_button_->setEnabled(false);
  cancel_button_->setEnabled(true);

  running_ = true;
  qt::qInfoBox(this, "Magnetometer calibration is started.");
}

void MagCalibrationWidget::onCancelButtonClicked()
{
  resetToPreStart();
  qt::qInfoBox(this, "Magnetometer calibration is cancelled.");
}

void MagCalibrationWidget::onFinishButtonClicked()
{
  TOBAS_CHECK(cnt_ <= kMaxDataSize);

  if (cnt_ < kMinDataSize) {
    qt::qWarnBox(this, "The number of collected samples is too small.");
    return;
  }

  // サンプル分だけ有効化
  std::fill(active_.begin(), active_.begin() + cnt_, true);

  // 前処理
  subsample();
  removeOutliers();

  // 有効なサンプル数を計算
  const auto size = numActiveSamples();

  // データを整理
  Eigen::VectorXd x(size), y(size), z(size);
  int idx = 0;
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& mag = buf_.at(pi);
    x(idx) = mag.x();
    y(idx) = mag.y();
    z(idx) = mag.z();
    ++idx;
  }
  TOBAS_CHECK(idx == size);
  const auto xx = x.cwiseProduct(x).eval();
  const auto yy = y.cwiseProduct(y).eval();
  const auto zz = z.cwiseProduct(z).eval();
  const auto xy = x.cwiseProduct(y).eval();
  const auto yz = y.cwiseProduct(z).eval();
  const auto zx = z.cwiseProduct(x).eval();

  constexpr auto kCalibMethod = kEllipseFitting;  // TODO: 手法を選べるようにする

  // 楕円体の係数を求める
  math::EllipseTransformer mag_trans;
  if (kCalibMethod == kBounding) {
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

    mag_trans.a_xx = 1 / rx2;
    mag_trans.a_yy = 1 / ry2;
    mag_trans.a_zz = 1 / rz2;
    mag_trans.a_xy = 0;
    mag_trans.a_yz = 0;
    mag_trans.a_zx = 0;
    mag_trans.b_x = -2 * x0 / rx2;
    mag_trans.b_y = -2 * y0 / ry2;
    mag_trans.b_z = -2 * z0 / rz2;
    mag_trans.c = math::sqr(x0) / rx2 + math::sqr(y0) / ry2 + math::sqr(z0) / rz2 - 1;
  }
  else {
    // 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
    // SVDは遅いが最も精度が高い: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    mag_trans.c = -(xx + yy + zz).mean();
    Eigen::VectorXd ce0(size);
    ce0.fill(-mag_trans.c);

    if (kCalibMethod == kSphereFitting) {
      // 球体でフィッティング．
      // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
      Eigen::MatrixX4d CE(size, 4);
      CE.col(0) = xx + yy + zz;
      CE.col(1) = x;
      CE.col(2) = y;
      CE.col(3) = z;
      const auto coefs = CE.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ce0).eval();

      mag_trans.a_xx = coefs(0);
      mag_trans.a_yy = coefs(0);
      mag_trans.a_zz = coefs(0);
      mag_trans.a_xy = 0;
      mag_trans.a_yz = 0;
      mag_trans.a_zx = 0;
      mag_trans.b_x = coefs(1);
      mag_trans.b_y = coefs(2);
      mag_trans.b_z = coefs(3);
    }
    else if (kCalibMethod == kEllipseFitting) {
      // 楕円体でフィッティング．球より精密だが過学習のリスクがある．
      // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + bx x + by y + bz z + c = 0
      Eigen::Matrix<double, Eigen::Dynamic, 9> CE(size, 9);
      CE.col(0) = xx;
      CE.col(1) = yy;
      CE.col(2) = zz;
      CE.col(3) = 2 * xy;
      CE.col(4) = 2 * yz;
      CE.col(5) = 2 * zx;
      CE.col(6) = x;
      CE.col(7) = y;
      CE.col(8) = z;
      const auto coefs = CE.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ce0).eval();

      mag_trans.a_xx = coefs(0);
      mag_trans.a_yy = coefs(1);
      mag_trans.a_zz = coefs(2);
      mag_trans.a_xy = coefs(3);
      mag_trans.a_yz = coefs(4);
      mag_trans.a_zx = coefs(5);
      mag_trans.b_x = coefs(6);
      mag_trans.b_y = coefs(7);
      mag_trans.b_z = coefs(8);
    }
    else {
      throw;
    }
  }

  // 楕円体であることを確認
  if (!mag_trans.initialize()) {
    qt::qErrorBox(this, "The estimated coefficients do not satisfy the conditions necessary for forming an ellipsoid.");
    return;
  }

  // TODO: データがきれいな楕円体を描いているかどうかをチェック

  // パラメータを作成
  const auto req = std::make_shared<tobas_real_msgs::srv::SetMagnetometerParams::Request>();
  req->a_xx = mag_trans.a_xx;
  req->a_yy = mag_trans.a_yy;
  req->a_zz = mag_trans.a_zz;
  req->a_xy = mag_trans.a_xy;
  req->a_yz = mag_trans.a_yz;
  req->a_zx = mag_trans.a_zx;
  req->b_x = mag_trans.b_x;
  req->b_y = mag_trans.b_y;
  req->b_z = mag_trans.b_z;
  req->c = mag_trans.c;

  // パラメータを更新
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::handler::mag::kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout)) {
    qt::qErrorBox(this, "Failed to send calibration results.");
    return;
  }

  // 結果を確認
  const auto res = sc.getResponse();
  if (!res->success) {
    qt::qErrorBox(this, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return;
  }

  // キャリブレーション後の点群を表示
  auto pc_calib = std::make_unique<sensor_msgs::msg::PointCloud>();
  pc_calib->header.stamp = node_->get_clock()->now();
  pc_calib->header.frame_id = tobas::kWorldFrame;
  pc_calib->points.resize(size);
  for (int i = 0; i < size; ++i) {
    const auto p_calib = mag_trans.transform(buf_.at(i).data);
    const auto p_disp = p_calib.cast<float>() * kRvizPointScale;
    tf::point32EigenToMsg(p_disp, pc_calib->points.at(i));
  }
  pc_pub_->publish(std::move(pc_calib));

  resetToPreStart();
  qt::qInfoBox(this, "Magnetometer calibration finished successfully.");
}

void MagCalibrationWidget::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg)
{
  if (!running_) {
    return;
  }

  const auto& mag = msg->mag;

  // 最初のデータ
  if (cnt_ == 0) {
    mag_norm_ = mag.norm();
    if (mag_norm_ == 0.) {
      qWarning() << "Zero magnetic field is received.";
      return;
    }

    last_time_ = msg->header.stamp;
    last_face_idx_ = computeFaceIndex();
    face_circles_.at(last_face_idx_)->setSelected(true);
  }

  // 最大点数に達したら強制終了
  if (cnt_ >= kMaxDataSize) {
    resetToPreStart();
    qt::qErrorBox(this, "Magnetometer sample limit reached. Calibration canceled.");
    return;
  }

  // データを追加
  buf_.at(cnt_++) = mag;

  // 表示用メッセージを発行
  auto point_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  point_msg->header = msg->header;
  point_msg->header.frame_id = tobas::kWorldFrame;  // Rvizの設定の"Global Options/Fixed Frame"と一致させる
  kdl::pointKDLToMsg(mag * (kRvizPointScale / mag_norm_), point_msg->point);
  ps_pub_->publish(std::move(point_msg));

  // 時刻を更新
  const auto& cur_time = msg->header.stamp;
  const auto dt = (cur_time - last_time_).seconds();
  last_time_ = cur_time;

  // 現在の向きを円の外枠の色で表示
  const auto face_idx = computeFaceIndex();
  if (face_idx != last_face_idx_) {
    face_circles_.at(last_face_idx_)->setSelected(false);
    face_circles_.at(face_idx)->setSelected(true);
    last_face_idx_ = face_idx;
  }

  if (!finish_button_->isEnabled()) {
    // 現在の向きの回転量を更新
    if (!completed_.at(face_idx)) {
      // グローバルZ軸回りの回転速さを計算
      const auto W_gyro = odom_->frame.M * odom_->twist.rot;
      const auto yawrate = fabs(W_gyro.z());

      // 回転を検知したら回転量を積分
      // 回転が速すぎると十分にサンプリングできないため上限を定める
      if (yawrate > kMinYawRate) {
        rot_angles_.at(face_idx) += std::min(yawrate, kMaxYawRate) * dt;
      }

      // 個別の進捗を更新
      const auto progress = rot_angles_.at(face_idx) / kYawAngleThresh;  // [-]
      face_circles_.at(face_idx)->setProgress(progress);

      // 十分回転したら完了
      if (progress > 1.) {
        completed_.at(face_idx) = true;
      }
    }

    // 全体の進捗バーを更新
    double total_progress = 0.;  // [-]
    for (const auto& [rot_angle, completed] : std::views::zip(rot_angles_, completed_)) {
      const auto progress = completed ? 1. : rot_angle / kYawAngleThresh;
      total_progress += progress / kFaceSize;
    }
    progress_bar_->setValue(static_cast<int>(total_progress * 100.));

    // 全ての面のデータが十分に溜まったらFinishボタンを有効化
    if (tobas_std::allEqual(completed_, true)) {
      finish_button_->setEnabled(true);
      progress_bar_->setValue(100);
    }
  }
}

void MagCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg)
{
  arming_ = msg;
}

void MagCalibrationWidget::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& msg)
{
  odom_ = msg;
}
}  // namespace hw
}  // namespace gui
