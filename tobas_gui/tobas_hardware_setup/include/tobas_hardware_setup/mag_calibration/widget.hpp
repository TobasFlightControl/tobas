#pragma once

#include <QPushButton>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>
#include <tobas_qt_tools/rviz.hpp>

#include "../base.hpp"

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

public:
  explicit MagCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  qt::RvizFrameManager rviz_manager_;

  std::string ns_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  int cnt_;
  double mag_norm_;
  std::array<Eigen::Vector3d, kMaxDataSize> mag_data_;
  math::EllipseTransformer mag_trans_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  rviz_common::properties::Property* ps_history_length_;

  ros2::PublisherPtr<geometry_msgs::msg::PointStamped> ps_pub_;
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud> pc_pub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticFieldStamped> mag_raw_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;

  void magCb(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& mag_raw);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();
};
}  // namespace hw
}  // namespace gui
