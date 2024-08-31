#include <tobas_std_tools/console.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/stream.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/selected_links.hpp"

namespace gui
{
namespace setup_assistant
{
SelectedLinksWidget::SelectedLinksWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
  : node_(node), robot_(robot)
{
  ignoreWheelEvent();
  setSize(kTabWidth, kTabHeight);
  setMovable(true);
  setTabsClosable(true);

  markers_pub_ = ros2::createPublisher<visualization_msgs::msg::MarkerArray>(node, "visualization_marker_array");

  publish_markers_timer_ = new QTimer(this);
  connect(publish_markers_timer_, &QTimer::timeout, this, &self::publishTimerCb);

  connect(this, &qt::TabWidget::tabCloseRequested, this, &self::onTabCloseRequested);
}

void SelectedLinksWidget::updateInternalDataStructures()
{
  clear();

  // 全ての可動リンクのマーカを保持しておく
  size_t id = 0;
  for (const auto& [seg_name, seg_ele] : robot_.tree().getSegments())
  {
    // ジョイントを取得
    const auto& joint = seg_ele.segment.joint();
    if (joint.type != kdl::Joint::RotAxis)
      continue;

    // 推力の作用線
    const auto arrow_start = kdl::Vector::Zero();
    const auto arrow_end = joint.axis();
    const auto arrow_scale = kdl::Vector(0.1, 0.2, 0.3) * kArrowLength;

    // マーカを作成
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = seg_name;
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::DELETE;  // デフォルトでは非表示

    marker.points.resize(2);
    kdl::pointKDLToMsg(arrow_start, marker.points.at(0));
    kdl::pointKDLToMsg(arrow_end, marker.points.at(1));
    kdl::vectorKDLToMsg(arrow_scale, marker.scale);

    // TODO: 回転方向によって色分け
    marker.color.r = 1.0;
    marker.color.g = 0.4;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // 無限の生存期間
    marker.frame_locked = true;                               // TFが変化してもフレームに固定

    // マーカを追加
    markers_.markers.push_back(marker);
  }
}

void SelectedLinksWidget::clear()
{
  super::clear();
  markers_.markers.clear();
  publish_markers_timer_->stop();
}

bool SelectedLinksWidget::isValid()
{
  const auto num_rotors = count();

  // 最低1つは登録されていなければならない
  if (num_rotors == 0)
  {
    qt::qErrorBox(this, "Please register at least one link as a propulsion system.");
    return false;
  }

  // それぞれのタブの設定が有効であることを確認
  for (int i = 0; i < num_rotors; ++i)
    if (!widget(i)->isValid())
      return false;

  return true;
}

void SelectedLinksWidget::add(const QString& link_name)
{
  // タブを追加
  const auto link_widget = new SelectedLinkWidget(node_);
  addTab(link_widget, link_name);

  // 指定リンクのマーカを表示
  setAction(link_name, visualization_msgs::msg::Marker::ADD);
}

void SelectedLinksWidget::remove(const QString& link_name)
{
  // タブを削除
  removeTab(index(link_name));

  // 指定リンクのマーカを非表示
  setAction(link_name, visualization_msgs::msg::Marker::DELETE);
}

int SelectedLinksWidget::index(const QString& link_name) const
{
  for (int i = 0; i < count(); ++i)
    if (tabText(i) == link_name)
      return i;

  PRINT_WARN("Link \"" << link_name << "\" is not selected as a propulsion system.");
  return -1;
}

SelectedLinkWidget* SelectedLinksWidget::widget(int index)
{
  return qobject_cast<SelectedLinkWidget*>(super::widget(index));
}

const SelectedLinkWidget* SelectedLinksWidget::widget(int index) const
{
  return qobject_cast<SelectedLinkWidget*>(super::widget(index));
}

SelectedLinkWidget* SelectedLinksWidget::widget(const QString& link_name)
{
  return widget(index(link_name));
}

const SelectedLinkWidget* SelectedLinksWidget::widget(const QString& link_name) const
{
  return widget(index(link_name));
}

void SelectedLinksWidget::publishTimerCb()
{
  // Fill timestamps
  const auto now = node_->get_clock()->now();
  for (auto& marker : markers_.markers)
    marker.header.stamp = now;

  // Publish markers
  auto markers_ptr = std::make_unique<visualization_msgs::msg::MarkerArray>(markers_);
  markers_pub_->publish(std::move(markers_ptr));
}

void SelectedLinksWidget::onTabCloseRequested(int index)
{
  const auto link_name = tabText(index);
  remove(link_name);
  Q_EMIT linkRemoved(link_name);
}

void SelectedLinksWidget::setAction(const QString& link_name, int action)
{
  for (auto& marker : markers_.markers)
  {
    if (marker.header.frame_id == link_name.toStdString())
    {
      marker.action = action;
      return;
    }
  }

  PRINT_WARN("Link \"" << link_name << "\" not found.");
}
}  // namespace setup_assistant
}  // namespace gui
