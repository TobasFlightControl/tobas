#pragma once

#include <QPushButton>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_common/properties/int_property.hpp>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_hal_msgs_adapter/MagneticField.hpp>
#include <tobas_qt_tools/rviz.hpp>

#include "../base.hpp"

namespace gui
{
namespace hardware_setup
{
class MagCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = MagCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr char kRvizPointTopic[] = "rviz/magnetic_field";
  static constexpr int kMinDataSize = 1000;
  static constexpr int kMaxDataSize = 100000;  // 8[B] * 3 * 100000 = 2400000[B] = 2.4[MB]
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr double kRvizPointScale = 10.;

public:
  explicit MagCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

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

  rviz_common::properties::Property* history_length_;

  ros2::PublisherPtr<geometry_msgs::msg::PointStamped> point_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::MagneticField> mag_raw_sub_;

  void reset();

  void magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();
};
}  // namespace hardware_setup
}  // namespace gui
