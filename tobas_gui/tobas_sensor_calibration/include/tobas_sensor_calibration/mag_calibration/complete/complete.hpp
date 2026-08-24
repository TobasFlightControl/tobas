// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/properties/property.hpp>

#include <tobas_eigen_tools/ellipsoid.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_rqt_bridge/bridge.hpp>
#include <tobas_rviz_wrapper/rviz.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "../base.hpp"
#include "./face_circle.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class CompleteMagCalibWidget : public BaseMagCalibWidget
{
  Q_OBJECT

  using self = CompleteMagCalibWidget;
  using super = BaseMagCalibWidget;

  static constexpr char kSampledPointsTopic[] = "rviz/mag_calibration/sampled";
  static constexpr char kUsedPointsTopic[] = "rviz/mag_calibration/used";
  static constexpr char kRemovedPointsTopic[] = "rviz/mag_calibration/removed";
  static constexpr char kCalibratedPointsTopic[] = "rviz/mag_calibration/calibrated";
  static constexpr char kEllipsoidTopic[] = "rviz/mag_calibration/ellipsoid";
  static constexpr int kMinDataSize = 500;
  static constexpr int kMaxDataSize = 10000;  // RViz limits the maximum to 100000.
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr double kRvizPointScale = 10.0;
  static constexpr double kMinYawRate = M_PI / 30;     // [rad/s]
  static constexpr double kMaxYawRate = M_PI_2;        // [rad/s]
  static constexpr double kYawAngleThresh = 8 * M_PI;  // [rad]
  static constexpr double kZScoreThresh = 2.0;
  static constexpr int kEllipsoidLineStep = 20;  // [deg]

  static constexpr size_t kTopIdx = 0;
  static constexpr size_t kBottomIdx = kTopIdx + 1;
  static constexpr size_t kFrontIdx = kBottomIdx + 1;
  static constexpr size_t kBackIdx = kFrontIdx + 1;
  static constexpr size_t kLeftIdx = kBackIdx + 1;
  static constexpr size_t kRightIdx = kLeftIdx + 1;
  static constexpr size_t kFaceSize = kRightIdx + 1;

public:
  explicit CompleteMagCalibWidget(const RosQtBridge& bridge);

  void reset() override;
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns) override;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  rclcpp::Node::SharedPtr node_;

  ros2::SyncServiceClient<tobas_real_msgs::srv::SetMagnetometerParams>::SharedPtr set_params_sc_;

  rviz::RvizFrameManager rviz_manager_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  QProgressBar* progress_bar_;
  std::array<FaceCircleWidget*, kFaceSize> face_circles_;

  // Measurement variables.
  bool running_;
  int cnt_;
  builtin_interfaces::msg::Time last_time_;
  size_t last_face_idx_;
  std::array<double, kFaceSize> rot_angles_;
  std::array<bool, kFaceSize> completed_;
  std::array<kdl::Vector, kMaxDataSize> buf_;
  std::array<bool, kMaxDataSize> active_;

  // ROS messages
  tobas_msgs::MagneticField::ConstSharedPtr mag_raw_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;

  // ROS Pub/Sub
  ros2::PublisherPtr<geometry_msgs::msg::PointStamped> samples_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> used_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> removed_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> calibrated_pub_;
  ros2::PublisherPtr<visualization_msgs::msg::MarkerArray> ellipsoid_pub_;

  /* Reset to the state before calibration starts. */
  void resetToPreStart();

  /* Clear the point cloud on the screen. */
  void clearDisplayPoints();

  int numActiveSamples() const;
  size_t computeFaceIndex() const;

  /* Equalize density: https://www.jstage.jst.go.jp/article/pscjspe/2011A/0/2011A_0_277/_pdf/-char/ja */
  void subsample();

  /* Remove outliers: https://www.codexa.net/python-outlier/ */
  void removeOutliers();

  /* Find the offset using sphere approximation. */
  bool
  computeHardBias(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z, Eigen::Vector3d& dst);

  /* Find distortion using ellipsoid approximation. */
  bool
  computeSoftBias(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z, Eigen::Vector6d& dst);

  /* Update FC parameters. */
  bool updateRemoteParameters(const Eigen::Vector3d& hard_bias, const Eigen::Vector6d& soft_bias);

  /* Display the result point cloud. */
  void displayPointClouds(const eigen::Ellipsoid& ellipsoid);

  /* Display the estimated ellipsoid. */
  void displayEllipsoidWireFrame(const eigen::Ellipsoid& ellipsoid);
  void addEllipsoidPoint(
    double theta,
    double phi,
    const eigen::Ellipsoid& ellipsoid,
    std::vector<geometry_msgs::msg::Point>& points);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();

  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& msg);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
