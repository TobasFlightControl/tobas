#pragma once

#include <QTimer>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tobas_ros2_tools/definitions.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "./selected_link.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
class SelectedLinksWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = SelectedLinksWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;
  static constexpr double kArrowLength = 0.2;  // TODO: 想定される推力の最大値を矢印の長さに反映

Q_SIGNALS:
  void linkRemoved(const QString& link_name);
  void channelChanged(const QString& link_name, int channel);
  void isTiltStateChanged(const QString& link_name, bool is_tilt);
  void tiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);

public:
  explicit SelectedLinksWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  void updateInternalDataStructures();

  void clear();
  bool isValid();

  void addLink(const QString& link_name);
  void removeLink(const QString& link_name);

  QString linkName(int index) const;
  QStringList linkNames() const;
  QStringList tiltJointNames() const;

  /* タブのインデックスを返す．存在しなければ-1を返す． */
  int index(const QString& link_name) const;

  SelectedLinkWidget* widget(int index);
  const SelectedLinkWidget* widget(int index) const;
  SelectedLinkWidget* widget(const QString& link_name);
  const SelectedLinkWidget* widget(const QString& link_name) const;

  /* 両方の回転方向のプロペラが設定されていることを確かめる． */
  bool hasBothRotationalDirections() const;

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
  void onTabCloseRequested(int index);
  void onCopyFromLeftButtonClicked(const QString& link_name);
  void onCopyToAllButtonClicked(const QString& link_name);
  void onChannelChanged(const QString& link_name, int channel);
  void onIsTiltStateChanged(const QString& link_name, bool is_tilt);
  void onTiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
