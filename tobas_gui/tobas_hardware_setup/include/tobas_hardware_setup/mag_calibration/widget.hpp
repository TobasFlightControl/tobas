#pragma once

#include <QProgressBar>
#include <QPushButton>
#include <rviz_common/properties/property.hpp>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_rqt_bridge/bridge.hpp>
#include <tobas_rviz_wrapper/rviz.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "../base.hpp"
#include "./face_circle.hpp"

namespace gui
{
namespace hw
{
class MagCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = MagCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr char kRvizPointStampedTopic[] = "rviz/magnetic_field_raw";
  static constexpr char kRvizPointCloudTopic[] = "rviz/magnetic_field_calib";
  static constexpr int kMinDataSize = 500;
  static constexpr int kMaxDataSize = 10000;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr double kRvizPointScale = 10.;
  static constexpr double kMinYawRate = M_PI / 30;     // [rad/s]
  static constexpr double kMaxYawRate = M_PI_2;        // [rad/s]
  static constexpr double kYawAngleThresh = 4 * M_PI;  // [rad]

  static constexpr size_t kTopIdx = 0;
  static constexpr size_t kBottomIdx = kTopIdx + 1;
  static constexpr size_t kFrontIdx = kBottomIdx + 1;
  static constexpr size_t kBackIdx = kFrontIdx + 1;
  static constexpr size_t kLeftIdx = kBackIdx + 1;
  static constexpr size_t kRightIdx = kLeftIdx + 1;
  static constexpr size_t kFaceSize = kRightIdx + 1;

public:
  explicit MagCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void setNamespace(const std::string& ns);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  const rclcpp::Node::SharedPtr node_;
  rviz::RvizFrameManager rviz_manager_;

  std::string ns_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  QProgressBar* progress_bar_;
  std::array<FaceCircleWidget*, kFaceSize> face_circles_;

  // 計測用の変数
  bool running_;
  int cnt_;
  double mag_norm_;
  builtin_interfaces::msg::Time last_time_;
  size_t last_face_idx_;
  std::array<double, kFaceSize> rot_angles_;
  std::array<bool, kFaceSize> completed_;
  std::array<kdl::Vector, kMaxDataSize> buf_;
  std::array<bool, kMaxDataSize> active_;

  // ROS messages
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  // Rviz
  rviz_common::properties::Property* ps_history_length_;

  // ROS Pub/Sub
  ros2::PublisherPtr<geometry_msgs::msg::PointStamped> ps_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> pc_pub_;

  /* キャリブレーション開始前の状態にリセットする． */
  void resetToPreStart();

  int numActiveSamples() const;
  size_t computeFaceIndex() const;

  /* 密度を均一化: https://www.jstage.jst.go.jp/article/pscjspe/2011A/0/2011A_0_277/_pdf/-char/ja */
  void subsample();

  /* 外れ値を除去: https://www.codexa.net/python-outlier/ */
  void removeOutliers();

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();

  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& msg);
};
}  // namespace hw
}  // namespace gui
