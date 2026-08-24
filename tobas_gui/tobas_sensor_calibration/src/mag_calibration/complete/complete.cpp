// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/mag_calibration/complete/complete.hpp"

#include <iostream>
#include <ranges>

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/frame.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_eigen_conversions/eigen_msg.hpp>
#include <tobas_eigen_tools/hash.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_real_common/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_sensor_calibration/util.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
CompleteMagCalibWidget::CompleteMagCalibWidget(const RosQtBridge& bridge)
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

  rviz_rows_ = new QVBoxLayout();
  rviz_rows_->setContentsMargins(0, 0, 0, 0);

  const auto rows = new QVBoxLayout();
  rows->addWidget(instruction);
  rows->addLayout(button_cols);
  rows->addWidget(progress_bar_);
  rows->addLayout(face_cols, 1);
  rows->addLayout(rviz_rows_, 4);

  setLayout(rows);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  connect(&bridge, &RosQtBridge::rawMagReceived, this, &self::magCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);

  reset();
}

void CompleteMagCalibWidget::reset()
{
  resetToPreStart();

  if (rviz_manager_) {
    rviz_manager_->resetTime();
  }

  mag_raw_.reset();
  arming_.reset();
  odom_.reset();
}

void CompleteMagCalibWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  if (!rviz_manager_) {
    initializeRviz();
  }

  node_ = std::move(node);

  samples_pub_ = ros2::createPublisher<geometry_msgs::msg::PointStamped>(node_, kSampledPointsTopic);
  used_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kUsedPointsTopic);
  removed_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kRemovedPointsTopic);
  calibrated_pub_ = ros2::createPublisher<sensor_msgs::msg::PointCloud>(node_, kCalibratedPointsTopic);
  ellipsoid_pub_ = ros2::createPublisher<visualization_msgs::msg::MarkerArray>(node_, kEllipsoidTopic);

  set_params_sc_ = std::make_shared<ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams>>(
    node_, path::join(ns, kRemoteIfaceNS, real::handler::mag::kSetParamSrv));
}

void CompleteMagCalibWidget::initializeRviz()
{
  // RvizFrameManager creates a ROS node, so wait until the GCS ROS context has been configured.
  rviz_manager_ = rviz::RvizFrameManager("rviz_mag_calibration");
  const auto rviz_config_path = getPkgShareDir() / "config/mag_calibration.rviz";
  rviz_manager_->initialize(QString::fromStdString(rviz_config_path), this);

  // Set the fixed frame.
  // This must be a frame with TF published.
  rviz_manager_->setFixedFrame(frame::kWorld);

  const auto point_stamped_displays = rviz_manager_->getDisplays("PointStamped");
  TOBAS_CHECK(point_stamped_displays.size() == 1);
  const auto& samples_display = point_stamped_displays.at(0);

  const auto point_cloud_displays = rviz_manager_->getDisplays("PointCloud");
  TOBAS_CHECK(point_cloud_displays.size() == 3);
  const auto& used_display = point_cloud_displays.at(0);
  const auto& removed_display = point_cloud_displays.at(1);
  const auto& calibrated_display = point_cloud_displays.at(2);

  const auto marker_array_displays = rviz_manager_->getDisplays("MarkerArray");
  TOBAS_CHECK(marker_array_displays.size() == 1);
  const auto& ellipsoid_display = marker_array_displays.at(0);

  constexpr char kTopicProperty[] = "Topic";
  samples_display->subProp(kTopicProperty)->setValue(kSampledPointsTopic);
  used_display->subProp(kTopicProperty)->setValue(kUsedPointsTopic);
  removed_display->subProp(kTopicProperty)->setValue(kRemovedPointsTopic);
  calibrated_display->subProp(kTopicProperty)->setValue(kCalibratedPointsTopic);
  ellipsoid_display->subProp(kTopicProperty)->setValue(kEllipsoidTopic);

  rviz_rows_->addWidget(rviz_manager_->widget());
}

void CompleteMagCalibWidget::paintEvent(QPaintEvent*)
{
  // Determine the minimum point size.
  auto psize = INT32_MAX;
  for (const auto& face_circle : face_circles_) {
    psize = std::min(psize, face_circle->calcMaxTextPointSize());
  }

  // Align to the minimum point size.
  for (const auto& face_circle : face_circles_) {
    face_circle->setTextPointSize(psize);
  }
}

void CompleteMagCalibWidget::resetToPreStart()
{
  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  progress_bar_->setValue(0);

  for (const auto& face_circle : face_circles_) {
    face_circle->setProgress(0.0);
    face_circle->setSelected(false);
  }

  running_ = false;
  cnt_ = 0;
  rot_angles_.fill(0.0);
  completed_.fill(false);
}

void CompleteMagCalibWidget::clearDisplayPoints()
{
  // FIXME: A `PointStamped` received slightly after clearing may still be displayed.
  if (rviz_manager_) {
    rviz_manager_->resetTime();
  }
}

int CompleteMagCalibWidget::numActiveSamples() const
{
  return std::count(active_.begin(), active_.begin() + cnt_, true);
}

size_t CompleteMagCalibWidget::computeFaceIndex() const
{
  const auto& R_W_B = odom_->odom.odom.frame.M;

  // Get each axis Z component in the world coordinate system.
  const auto axz = R_W_B.axisX().z();
  const auto ayz = R_W_B.axisY().z();
  const auto azz = R_W_B.axisZ().z();
  const auto abs_axz = std::abs(axz);
  const auto abs_ayz = std::abs(ayz);
  const auto abs_azz = std::abs(azz);

  // Determine the face currently pointing upward from the order of components.
  if (abs_axz >= std::max(abs_ayz, abs_azz)) {
    if (axz > 0.0) {
      return kFrontIdx;
    }
    else {
      return kBackIdx;
    }
  }
  else if (abs_ayz >= std::max(abs_azz, abs_axz)) {
    if (ayz > 0.0) {
      return kLeftIdx;
    }
    else {
      return kRightIdx;
    }
  }
  else if (abs_azz >= std::max(abs_axz, abs_ayz)) {
    if (azz > 0.0) {
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

void CompleteMagCalibWidget::subsample()
{
  static constexpr int N = 30;

  // Find the bounding box.
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
  const auto d = (ub - lb) / N;  // Width of one region.

  // Assign each point to the grid.
  Eigen::Vector3i gi;                                         // Grid index
  std::unordered_map<Eigen::Vector3i, std::set<int>> groups;  // Use a hash table because the grid is sparse.
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

  // If multiple points exist in the same region, merge them by averaging.
  for (const auto& [_, group] : groups) {
    const auto group_size = group.size();
    if (group_size < 2) {
      continue;
    }

    // Calculate the average of points in the same region.
    auto sum = kdl::Vector::Zero();
    for (const auto& pi : group) {
      sum += buf_.at(pi);
    }
    const auto mean = sum / group_size;

    // Put the average in the first element and invalidate the others.
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

void CompleteMagCalibWidget::removeOutliers()
{
  const auto size = numActiveSamples();

  // Find the average point.
  auto sum = kdl::Vector::Zero();
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    sum += buf_.at(pi);
  }
  const auto mean = sum / size;

  // Find the standard deviation of distances from the average point.
  double dist2_sum = 0.0;
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    const auto dist2 = (p - mean).squaredNorm();
    dist2_sum += dist2;
  }
  const auto dist_var = dist2_sum / size;
  const auto dist_stddev = std::sqrt(dist_var);

  // Reject points whose distance from the average point is a large outlier.
  for (int pi = 0; pi < cnt_; ++pi) {
    if (!active_.at(pi)) {
      continue;
    }
    const auto& p = buf_.at(pi);
    const auto dist = (p - mean).norm();
    if (dist > dist_stddev * kZScoreThresh) {
      qWarning().nospace() << "Point (" << p.x() << ", " << p.y() << ", " << p.z() << ") was identified as an outlier.";
      active_.at(pi) = false;
    }
  }
}

bool CompleteMagCalibWidget::computeHardBias(
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& z,
  Eigen::Vector3d& dst)
{
  const auto size = numActiveSamples();

  const auto xx = x.cwiseProduct(x).eval();
  const auto yy = y.cwiseProduct(y).eval();
  const auto zz = z.cwiseProduct(z).eval();

  // Choose a rough scale.
  const auto scale = (xx + yy + zz).mean();
  Eigen::VectorXd ce0(size);
  ce0.fill(scale);

  // Fit with a sphere.
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

  // Use only the offset.
  dst = ellipsoid.getHardBias();
  return true;
}

bool CompleteMagCalibWidget::computeSoftBias(
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

  // Choose a rough scale.
  const auto scale = (xx + yy + zz).mean();
  Eigen::VectorXd ce0(size);
  ce0.fill(scale);

  // Fit with an ellipsoid centered at the origin.
  // axx x^2 + ayy y^2 + azz z^2 + 2 axy xy + 2 ayz yz + 2 azx zx + c = 0
  // cf. Estimate the equation with least squares: https://rikei-tawamure.com/entry/2021/10/07/211725
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

bool CompleteMagCalibWidget::updateRemoteParameters(const Eigen::Vector3d& hard_bias, const Eigen::Vector6d& soft_bias)
{
  // Create parameters.
  const auto req = std::make_shared<tobas_real_msgs::srv::SetMagnetometerParams::Request>();
  req->hard_bias = eigen::toStdArray(hard_bias);
  req->soft_bias = eigen::toStdArray(soft_bias);

  // Update parameters.
  const auto res = set_params_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to send calibration results.");
    return false;
  }

  // Check the result.
  if (!res->success) {
    qt::qErrorBox(this, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void CompleteMagCalibWidget::displayPointClouds(const eigen::Ellipsoid& ellipsoid)
{
  auto used_points = std::make_unique<sensor_msgs::msg::PointCloud>();
  auto removed_points = std::make_unique<sensor_msgs::msg::PointCloud>();
  auto calibrated_points = std::make_unique<sensor_msgs::msg::PointCloud>();

  const auto cur_time = node_->now();
  used_points->header.stamp = cur_time;
  removed_points->header.stamp = cur_time;
  calibrated_points->header.stamp = cur_time;

  used_points->header.frame_id = frame::kWorld;
  removed_points->header.frame_id = frame::kWorld;
  calibrated_points->header.frame_id = frame::kWorld;

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

void CompleteMagCalibWidget::displayEllipsoidWireFrame(const eigen::Ellipsoid& ellipsoid)
{
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->now();
  marker.header.frame_id = frame::kWorld;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;  // For `LINE_STRIP`, y and z are ignored.
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  // Add lines with fixed theta.
  for (int theta_deg = -90; theta_deg < 90; theta_deg += kEllipsoidLineStep) {
    marker.points.clear();

    const auto theta = st::deg2rad(theta_deg);
    for (int phi_deg = 0; phi_deg <= 360; ++phi_deg) {
      const auto phi = st::deg2rad(phi_deg);
      addEllipsoidPoint(theta, phi, ellipsoid, marker.points);
    }

    markers->markers.push_back(marker);
    ++marker.id;
  }

  // Add lines with fixed phi.
  for (int phi_deg = 0; phi_deg < 360; phi_deg += kEllipsoidLineStep) {
    marker.points.clear();

    const auto phi = st::deg2rad(phi_deg);
    for (int theta_deg = -90; theta_deg <= 90; ++theta_deg) {
      const auto theta = st::deg2rad(theta_deg);
      addEllipsoidPoint(theta, phi, ellipsoid, marker.points);
    }

    markers->markers.push_back(marker);
    ++marker.id;
  }

  // Publish the marker.
  ellipsoid_pub_->publish(std::move(markers));
}

void CompleteMagCalibWidget::addEllipsoidPoint(
  double theta,
  double phi,
  const eigen::Ellipsoid& ellipsoid,
  std::vector<geometry_msgs::msg::Point>& points)
{
  const Eigen::Vector3d p(
    std::cos(theta) * std::cos(phi), std::cos(theta) * std::sin(phi), std::sin(theta));  // Unit sphere
  const Eigen::Vector3d q = ellipsoid.fromUnitSphere(p);                                 // Ellipsoid

  points.emplace_back();
  tf::pointEigenToMsg(kRvizPointScale * q, points.back());
}

void CompleteMagCalibWidget::onStartButtonClicked()
{
  // Confirm that the vehicle is not armed.
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    return;
  }

  // Confirm that required topics have been received.
  if (!mag_raw_) {
    qt::qWarnBox(this, "Magnetic field has not been received yet.");
    return;
  }
  if (!odom_) {
    qt::qWarnBox(
      this,
      "This operation cannot be performed because the odometry has not been received yet. "
      "Please check whether the accelerometer has been calibrated.");
    return;
  }

  clearDisplayPoints();

  start_button_->setEnabled(false);
  cancel_button_->setEnabled(true);

  running_ = true;
  qt::qInfoBox(this, "Magnetometer calibration started.");
}

void CompleteMagCalibWidget::onCancelButtonClicked()
{
  resetToPreStart();
  clearDisplayPoints();

  qt::qInfoBox(this, "Magnetometer calibration was canceled.");
}

void CompleteMagCalibWidget::onFinishButtonClicked()
{
  TOBAS_CHECK(cnt_ <= kMaxDataSize);

  if (cnt_ < kMinDataSize) {
    qt::qWarnBox(this, "The number of collected samples is too small.");
    return;
  }

  // Enable only the sample count.
  std::fill(active_.begin(), active_.begin() + cnt_, true);

  // Preprocess.
  subsample();
  removeOutliers();

  // Calculate the number of valid samples.
  const auto size = numActiveSamples();

  // Organize data.
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

  // Calculate hard-iron bias using sphere approximation.
  Eigen::Vector3d hard_bias;
  if (!computeHardBias(x, y, z, hard_bias)) {
    return;
  }

  // Move data to the origin.
  x = x.array() - hard_bias.x();
  y = y.array() - hard_bias.y();
  z = z.array() - hard_bias.z();

  // Calculate soft-iron bias using ellipsoid approximation.
  // Linear regression with the expanded equation does not minimize Euclidean distance,
  // but centering the data at the origin makes the result a close approximation.
  Eigen::Vector6d soft_bias;
  if (!computeSoftBias(x, y, z, soft_bias)) {
    return;
  }

  // Update FC parameters.
  if (!updateRemoteParameters(hard_bias, soft_bias)) {
    return;
  }

  // Show the result.
  rviz_manager_->resetTime();
  const eigen::Ellipsoid ellipsoid(hard_bias, soft_bias);
  displayPointClouds(ellipsoid);
  displayEllipsoidWireFrame(ellipsoid);

  // Show debug information.
  std::cout << "The number of samples before preprocessing: " << cnt_ << std::endl;
  std::cout << "The number of samples after preprocessing: " << size << std::endl;
  std::cout << "The number of removed samples: " << cnt_ - size << std::endl;
  std::cout << "Hard-iron bias: " << hard_bias.transpose() << std::endl;
  std::cout << "Soft-iron bias: " << soft_bias.transpose() << std::endl;

  resetToPreStart();
  qt::qInfoBox(this, "Magnetometer calibration finished successfully. Please restart the flight controller.");
}

void CompleteMagCalibWidget::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg)
{
  mag_raw_ = msg;

  if (!running_) {
    return;
  }

  // First data point.
  if (cnt_ == 0) {
    last_time_ = msg->header.stamp;
    last_face_idx_ = computeFaceIndex();
    face_circles_.at(last_face_idx_)->setSelected(true);
  }

  // Stop collecting data when the maximum number of points is reached.
  if (cnt_ >= kMaxDataSize) {
    resetToPreStart();
    qt::qErrorBox(this, "Magnetometer sample limit reached. Calibration canceled.");
    return;
  }

  // Add data.
  buf_.at(cnt_++) = msg->mag;

  // Publish a display message.
  auto point_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  point_msg->header = msg->header;
  point_msg->header.frame_id = frame::kWorld;  // Match the RViz `Global Options/Fixed Frame` setting.
  kdl::pointKDLToMsg(msg->mag * kRvizPointScale, point_msg->point);
  samples_pub_->publish(std::move(point_msg));

  // Update the time.
  const auto& cur_time = msg->header.stamp;
  const auto dt = (cur_time - last_time_).seconds();
  last_time_ = cur_time;

  // Show the current orientation with the circle outline color.
  const auto face_idx = computeFaceIndex();
  if (face_idx != last_face_idx_) {
    face_circles_.at(last_face_idx_)->setSelected(false);
    face_circles_.at(face_idx)->setSelected(true);
    last_face_idx_ = face_idx;
  }

  if (!finish_button_->isEnabled()) {
    // Update the rotation amount for the current orientation.
    if (!completed_.at(face_idx)) {
      // Calculate the rotation speed around the global Z axis.
      const auto W_gyro = odom_->odom.odom.frame.M * odom_->odom.odom.twist.rot;
      const auto yawrate = std::abs(W_gyro.z());

      // Integrate the rotation amount when rotation is detected.
      // Set an upper limit because sampling is insufficient if rotation is too fast.
      if (yawrate > kMinYawRate) {
        rot_angles_.at(face_idx) += std::min(yawrate, kMaxYawRate) * dt;
      }

      // Update individual progress.
      const auto progress = rot_angles_.at(face_idx) / kYawAngleThresh;  // [-]
      face_circles_.at(face_idx)->setProgress(progress);

      // Complete when enough rotation is reached.
      if (progress > 1.0) {
        completed_.at(face_idx) = true;
      }
    }

    // Update the overall progress bar.
    double total_progress = 0.0;  // [-]
    for (const auto& [rot_angle, completed] : std::views::zip(rot_angles_, completed_)) {
      const auto progress = completed ? 1.0 : rot_angle / kYawAngleThresh;
      total_progress += progress / kFaceSize;
    }
    progress_bar_->setValue(static_cast<int>(total_progress * 100.0));

    // Enable the Finish button when enough data has been collected for all faces.
    if (st::allEqual(completed_, true)) {
      finish_button_->setEnabled(true);
      progress_bar_->setValue(100);
    }
  }
}

void CompleteMagCalibWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg)
{
  if (running_ && msg->data) {
    resetToPreStart();
    clearDisplayPoints();
    qt::qWarnBox(this, "Magnetometer calibration was canceled because an arming command was issued.");
  }

  arming_ = msg;
}

void CompleteMagCalibWidget::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& msg)
{
  odom_ = msg;
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
