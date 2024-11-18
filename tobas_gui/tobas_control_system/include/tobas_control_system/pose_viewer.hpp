#pragma once

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_kdl_msgs_adapter/euler_stamped.hpp>
#include <tobas_qt_tools/widgets/widget.hpp>

namespace gui
{
namespace control_system
{
class PoseViewerWidget : public qt::Widget
{
  Q_OBJECT

  using self = PoseViewerWidget;
  using super = qt::Widget;

  static constexpr int kOriginalSize = 640;
  static constexpr int kLineWidth = 3;       // ゲージ線の幅
  static constexpr int kScaleInterval = 10;  // [deg]
  static constexpr int kRollRadius = 200;    // ロール円の半径
  static constexpr int kRollTickLength = 10;
  static constexpr double kPitchHeightRange = tobas_std::deg2rad(120);  // [rad]
  static constexpr int kPitchVisualRange = 25;                          // [deg] 描画するピッチ角の範囲
  static constexpr int kPitchLineLength = 100;
  static constexpr double kYawWidthRange = tobas_std::deg2rad(120);  // [rad]
  static constexpr int kYawLineY = 60;
  static constexpr int kYawTickLength = 10;

public:
  explicit PoseViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  // 現在のオイラー角
  kdl::Euler euler_;

  // 機体から見た地平線の方程式
  double slope_, y_intercept_;

  ros2::SubscriberPtr<tobas_kdl_msgs::EulerStamped> euler_sub_;

  void paintEvent(QPaintEvent* event) override;

  void reset();

  void scale(QPainter& painter, bool keep_aspect);

  void drawGround(QPainter& painter);
  void drawSky(QPainter& painter);
  void drawRoll(QPainter& painter);
  void drawPitch(QPainter& painter);
  void drawYaw(QPainter& painter);

  void addGradation(QPainter& painter);

  /* カメラの枠内の点が空に含まれるかどうかを判定する． */
  bool isSky(const QPoint& p) const;

  /* ピッチ角 [rad] をウィンドウ高さに変換する． */
  double pitchToHeight(double pitch) const;

  /* ヨー角 [rad] をウィンドウ幅に変換する． */
  double yawToWidth(double yaw) const;

  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler);
};
}  // namespace control_system
}  // namespace gui
