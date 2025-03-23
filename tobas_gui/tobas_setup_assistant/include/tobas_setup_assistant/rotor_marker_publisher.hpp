#pragma once

#include <QTimer>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tobas_ros2_tools/register.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace gui
{
namespace sa
{
class RotorMarkerPublisher : public QObject
{
  Q_OBJECT

  using self = RotorMarkerPublisher;
  using super = QObject;

  static constexpr double kArrowLength = 0.2;  // TODO: 想定される推力の最大値を矢印の長さに反映

public:
  explicit RotorMarkerPublisher(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

  visualization_msgs::msg::MarkerArray markers_;
  ros2::PublisherPtr<visualization_msgs::msg::MarkerArray> markers_pub_;
  QTimer publish_markers_timer_;

  /* 指定されたリンクのマーカのアクションを設定する． */
  void setAction(const QString& link_name, int action);

  void publishTimerCb();

private Q_SLOTS:
  void onRotorLinkAdded(const QString& link_name);
  void onRotorLinkRemoved(const QString& link_name);
};
}  // namespace sa
}  // namespace gui
