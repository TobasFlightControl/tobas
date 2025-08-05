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
  static constexpr int kMaxDataSize = 50000;  // 8[B] * 3 * 50000 = 1200000[B] = 1.2[MB]
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
  const RosQtBridge& bridge_;
  rviz::RvizFrameManager rviz_manager_;

  std::string ns_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  QProgressBar* progress_bar_;
  std::array<FaceCircleWidget*, kFaceSize> face_circles_;

  // 計測用の変数とバッファ
  int cnt_;
  double mag_norm_;
  builtin_interfaces::msg::Time last_time_;
  size_t last_face_idx_;
  std::array<double, kFaceSize> rot_angles_;
  std::array<bool, kFaceSize> completed_;
  std::array<kdl::Vector, kMaxDataSize> mag_data_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  rviz_common::properties::Property* ps_history_length_;

  ros2::PublisherPtr<geometry_msgs::msg::PointStamped> ps_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> pc_pub_;

  QMetaObject::Connection mag_conn_;

  /* キャリブレーション開始前の状態にリセットする． */
  void resetToPreStart();

  size_t computeFaceIndex() const;

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
