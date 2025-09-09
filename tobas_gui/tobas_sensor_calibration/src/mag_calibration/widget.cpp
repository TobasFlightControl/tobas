#include "tobas_sensor_calibration/mag_calibration/widget.hpp"

#include <iostream>
#include <ranges>

#include <QDebug>

#include <tobas_constants/constants.hpp>
#include <tobas_eigen_conversions/eigen_msg.hpp>
#include <tobas_eigen_tools/hash.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/core.hpp>
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

#include "tobas_sensor_calibration/constants.hpp"
#include "tobas_sensor_calibration/util.hpp"

namespace gui
{
namespace sc
{
MagCalibrationWidget::MagCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), rviz_manager_("rviz_mag_calibration")
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Click \"Start,\" and the magnetic field points (white) will begin appearing in the view.\n\n"
    "2. With each face up, rotate the FC around the gravity vector until each gauge is full and green.\n\n"
    "3. When all six faces are green and the progress bar reaches 100%, click \"Finish.\"\n\n"
    "4. Confirm that the calibrated point cloud (green) draws a neat sphere around the origin.\n\n",
    cmn::kBodyPSize);

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

  const auto rviz_config_path = getPkgShareDir() / "config/mag_calibration.rviz";
  rviz_manager_.initialize(QString::fromStdString(rviz_config_path));

  // 固定フレームを設定
  // TFが出ているフレームでなければならない
  rviz_manager_.setFixedFrame(tobas::kWorldFrame);

  const auto point_stamped_displays = rviz_manager_.getDisplays("PointStamped");
  TOBAS_CHECK(point_stamped_displays.size() == 1);
  const auto& samples_display = point_stamped_displays.at(0);

  const auto point_cloud_displays = rviz_manager_.getDisplays("PointCloud");
  TOBAS_CHECK(point_cloud_displays.size() == 3);
  const auto& used_display = point_cloud_displays.at(0);
  const auto& removed_display = point_cloud_displays.at(1);
  const auto& calibrated_display = point_cloud_displays.at(2);

  const auto marker_array_displays = rviz_manager_.getDisplays("MarkerArray");
  TOBAS_CHECK(marker_array_displays.size() == 1);
  const auto& ellipsoid_display = marker_array_displays.at(0);

  samples_display->subProp("Topic")->setValue(kSampledPointsTopic);
  used_display->subProp("Topic")->setValue(kUsedPointsTopic);
  removed_display->subProp("Topic")->setValue(kRemovedPointsTopic);
  calibrated_display->subProp("Topic")->setValue(kCalibratedPointsTopic);
  ellipsoid_display->subProp("Topic")->setValue(kEllipsoidTopic);

  samples_pub_ = ros2::createPublisher<geometry_msgs::msg::PointStamped>(node_, kSampledPointsTopic);
  used_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kUsedPointsTopic);
  removed_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kRemovedPointsTopic);
  calibrated_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kCalibratedPointsTopic);
  ellipsoid_pub_ = ros2::createPublisher<visualization_msgs::msg::MarkerArray>(node_, kEllipsoidTopic);

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

void MagCalibrationWidget::clearDisplayPoints()
{
  // FIXME: クリア後に僅かに遅れて受け取られたPointStampedが表示されてしまうことがある
  return rviz_manager_.resetTime();
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
  static constexpr int N = 30;

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
  Eigen::Vector3i gi;                                         // Grid index
  std::unordered_map<Eigen::Vector3i, std::set<int>> groups;  // 疎なのでハッシュテーブルで管理
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
    if (dist > dist_stddev * kZScoreThresh) {
      qWarning() << "Point (" << p.x() << ", " << p.y() << ", " << p.z() << ") was identified as an outlier.";
      active_.at(pi) = false;
    }
  }
}

bool MagCalibrationWidget::computeHardBias(
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& z,
  Eigen::Vector3d& dst)
{
  const auto size = numActiveSamples();

  const auto xx = x.cwiseProduct(x).eval();
  const auto yy = y.cwiseProduct(y).eval();
  const auto zz = z.cwiseProduct(z).eval();

  // 適当にスケールを決める
  const auto scale = (xx + yy + zz).mean();
  Eigen::VectorXd ce0(size);
  ce0.fill(scale);

  // 球体でフィッティング
  // axx x^2 + axx y^2 + axx z^2 + bx x + by y + bz z + c = 0
  Eigen::MatrixX4d CE(size, 4);
  CE.col(0) = xx + yy + zz;
  CE.col(1) = x;
  CE.col(2) = y;
  CE.col(3) = z;
  const auto sol = CE.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ce0).eval();

  eigen::EllipsoidCoefficients coefs;
  coefs.a_xx = sol(0);
  coefs.a_yy = sol(0);
  coefs.a_zz = sol(0);
  coefs.a_xy = 0;
  coefs.a_yz = 0;
  coefs.a_zx = 0;
  coefs.b_x = sol(1);
  coefs.b_y = sol(2);
  coefs.b_z = sol(3);
  coefs.c = -scale;

  eigen::Ellipsoid ellipsoid;
  if (!ellipsoid.initialize(coefs)) {
    qt::qErrorBox(this, "Sphere fitting failed.");
    return false;
  }

  // オフセットのみ使用
  dst = ellipsoid.getHardBias();
  return true;
}

bool MagCalibrationWidget::computeSoftBias(
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& z,
  Eigen::Vector6d& dst)
{
  const auto size = numActiveSamples();

  const auto xx = x.cwiseProduct(x).eval();
  const auto yy = y.cwiseProduct(y).eval();
  const auto zz = z.cwiseProduct(z).eval();
  const auto xy = x.cwiseProduct(y).eval();
  const auto yz = y.cwiseProduct(z).eval();
  const auto zx = z.cwiseProduct(x).eval();

  // 適当にスケールを決める
  const auto scale = (xx + yy + zz).mean();
  Eigen::VectorXd ce0(size);
  ce0.fill(scale);

  // 原点中心の楕円体でフィッティング
  // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + c = 0
  // cf. 最小二乗法で方程式を推定: https://rikei-tawamure.com/entry/2021/10/07/211725
  Eigen::MatrixX6d CE(size, 6);
  CE.col(0) = xx;
  CE.col(1) = yy;
  CE.col(2) = zz;
  CE.col(3) = 2 * xy;
  CE.col(4) = 2 * yz;
  CE.col(5) = 2 * zx;
  const auto sol = CE.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ce0).eval();

  eigen::EllipsoidCoefficients coefs;
  coefs.a_xx = sol(0);
  coefs.a_yy = sol(1);
  coefs.a_zz = sol(2);
  coefs.a_xy = sol(3);
  coefs.a_yz = sol(4);
  coefs.a_zx = sol(5);
  coefs.b_x = 0;
  coefs.b_y = 0;
  coefs.b_z = 0;
  coefs.c = -scale;

  eigen::Ellipsoid ellipsoid;
  if (!ellipsoid.initialize(coefs)) {
    qt::qErrorBox(this, "Ellipsoid fitting failed.");
    return false;
  }

  dst = ellipsoid.getSoftBias();
  return true;
}

bool MagCalibrationWidget::updateRemoteParameters(const Eigen::Vector3d& hard_bias, const Eigen::Vector6d& soft_bias)
{
  // パラメータを作成
  const auto req = std::make_shared<tobas_real_msgs::srv::SetMagnetometerParams::Request>();
  req->hard_bias = eigen::toStdArray(hard_bias);
  req->soft_bias = eigen::toStdArray(soft_bias);

  // パラメータを更新
  ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, real::handler::mag::kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout)) {
    qt::qErrorBox(this, "Failed to send calibration results.");
    return false;
  }

  // 結果を確認
  const auto res = sc.getResponse();
  if (!res->success) {
    qt::qErrorBox(this, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void MagCalibrationWidget::displayPointClouds(const eigen::Ellipsoid& ellipsoid)
{
  auto used_points = std::make_unique<sensor_msgs::msg::PointCloud>();
  auto removed_points = std::make_unique<sensor_msgs::msg::PointCloud>();
  auto calibrated_points = std::make_unique<sensor_msgs::msg::PointCloud>();

  const auto cur_time = node_->get_clock()->now();
  used_points->header.stamp = cur_time;
  removed_points->header.stamp = cur_time;
  calibrated_points->header.stamp = cur_time;

  used_points->header.frame_id = tobas::kWorldFrame;
  removed_points->header.frame_id = tobas::kWorldFrame;
  calibrated_points->header.frame_id = tobas::kWorldFrame;

  for (int pi = 0; pi < cnt_; ++pi) {
    const auto& p_raw = buf_.at(pi).data;

    if (active_.at(pi)) {
      used_points->points.emplace_back();
      tf::point32EigenToMsg(p_raw.cast<float>() * kRvizPointScale, used_points->points.back());

      const auto p_calib = ellipsoid.toUnitSphere(p_raw);
      calibrated_points->points.emplace_back();
      tf::point32EigenToMsg(p_calib.cast<float>() * kRvizPointScale, calibrated_points->points.back());
    }
    else {
      removed_points->points.emplace_back();
      tf::point32EigenToMsg(p_raw.cast<float>() * kRvizPointScale, removed_points->points.back());
    }
  }

  used_pub_->publish(std::move(used_points));
  removed_pub_->publish(std::move(removed_points));
  calibrated_pub_->publish(std::move(calibrated_points));
}

void MagCalibrationWidget::displayEllipsoidWireFrame(const eigen::Ellipsoid& ellipsoid)
{
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->get_clock()->now();
  marker.header.frame_id = tobas::kWorldFrame;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;  // LINE_STRIPの場合はy,zは無視される
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  // theta固定のラインを追加
  for (int theta_deg = -90; theta_deg < 90; theta_deg += kEllipsoidLineStep) {
    marker.points.clear();

    const auto theta = tobas_std::deg2rad(theta_deg);
    for (int phi_deg = 0; phi_deg <= 360; ++phi_deg) {
      const auto phi = tobas_std::deg2rad(phi_deg);
      addEllipsoidPoint(theta, phi, ellipsoid, marker.points);
    }

    markers->markers.push_back(marker);
    ++marker.id;
  }

  // phi固定のラインを追加
  for (int phi_deg = 0; phi_deg < 360; phi_deg += kEllipsoidLineStep) {
    marker.points.clear();

    const auto phi = tobas_std::deg2rad(phi_deg);
    for (int theta_deg = -90; theta_deg <= 90; ++theta_deg) {
      const auto theta = tobas_std::deg2rad(theta_deg);
      addEllipsoidPoint(theta, phi, ellipsoid, marker.points);
    }

    markers->markers.push_back(marker);
    ++marker.id;
  }

  // マーカを発行
  ellipsoid_pub_->publish(std::move(markers));
}

void MagCalibrationWidget::addEllipsoidPoint(
  double theta,
  double phi,
  const eigen::Ellipsoid& ellipsoid,
  std::vector<geometry_msgs::msg::Point>& points)
{
  const Eigen::Vector3d p(cos(theta) * cos(phi), cos(theta) * sin(phi), sin(theta));  // Unit sphere
  const Eigen::Vector3d q = ellipsoid.fromUnitSphere(p);                              // Ellipsoid

  points.emplace_back();
  tf::pointEigenToMsg(kRvizPointScale * q, points.back());
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

  clearDisplayPoints();

  start_button_->setEnabled(false);
  cancel_button_->setEnabled(true);

  running_ = true;
  qt::qInfoBox(this, "Magnetometer calibration is started.");
}

void MagCalibrationWidget::onCancelButtonClicked()
{
  resetToPreStart();
  clearDisplayPoints();

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

  // 球体近似でハードアイアンバイアスを計算
  Eigen::Vector3d hard_bias;
  if (!computeHardBias(x, y, z, hard_bias)) {
    return;
  }

  // データを原点中心に移動する
  x = x.array() - hard_bias.x();
  y = y.array() - hard_bias.y();
  z = z.array() - hard_bias.z();

  // 楕円体近似でソフトアイアンバイアスを計算
  // 展開式を用いた線形回帰はユークリッド距離の最小化ではないが，データが原点中心ならばそれに近くなる
  // TODO: 写像後の点の原点からの距離の2乗と1の誤差の総和を目的関数にできればより実用に堪えると思われる
  Eigen::Vector6d soft_bias;
  if (!computeSoftBias(x, y, z, soft_bias)) {
    return;
  }

  // FCのパラメータを更新
  if (!updateRemoteParameters(hard_bias, soft_bias)) {
    return;
  }

  // 結果を表示
  rviz_manager_.resetTime();
  const eigen::Ellipsoid ellipsoid(hard_bias, soft_bias);
  displayPointClouds(ellipsoid);
  displayEllipsoidWireFrame(ellipsoid);

  // デバッグ情報を表示
  std::cout << "The number of samples before preprocessing: " << cnt_ << std::endl;
  std::cout << "The number of samples after preprocessing: " << size << std::endl;
  std::cout << "The number of removed samples: " << cnt_ - size << std::endl;
  std::cout << "Hard-iron bias: " << hard_bias.transpose() << std::endl;
  std::cout << "Soft-iron bias: " << soft_bias.transpose() << std::endl;

  resetToPreStart();
  qt::qInfoBox(this, "Magnetometer calibration finished successfully.");
}

void MagCalibrationWidget::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg)
{
  if (!running_) {
    return;
  }

  // 最初のデータ
  if (cnt_ == 0) {
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
  buf_.at(cnt_++) = msg->mag;

  // 表示用メッセージを発行
  auto point_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  point_msg->header = msg->header;
  point_msg->header.frame_id = tobas::kWorldFrame;  // Rvizの設定の"Global Options/Fixed Frame"と一致させる
  kdl::pointKDLToMsg(msg->mag * kRvizPointScale, point_msg->point);
  samples_pub_->publish(std::move(point_msg));

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
}  // namespace sc
}  // namespace gui
