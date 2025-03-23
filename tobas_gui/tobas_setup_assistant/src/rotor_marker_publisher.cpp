#include <QDebug>

#include <tobas_kdl_conversions/kdl_urdf.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>

#include "tobas_setup_assistant/rotor_marker_publisher.hpp"

namespace gui
{
namespace sa
{
RotorMarkerPublisher::RotorMarkerPublisher(rclcpp::Node::SharedPtr node, const RobotInfo& robot, Signals& _signals)
  : node_(node), robot_(robot)
{
  markers_pub_ =
    ros2::createPublisher<visualization_msgs::msg::MarkerArray>(node, "visualization_marker_array", false, true);

  connect(&_signals, &Signals::rotorLinkAdded, this, &self::onRotorLinkAdded);
  connect(&_signals, &Signals::rotorLinkRemoved, this, &self::onRotorLinkRemoved);
  connect(&publish_markers_timer_, &QTimer::timeout, this, &self::publishTimerCb);
}

void RotorMarkerPublisher::updateInternalDataStructures()
{
  markers_.markers.clear();
  publish_markers_timer_.stop();

  // 全ての可動リンクのマーカを保持しておく
  size_t id = 0;
  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    // ジョイントを取得
    const auto& kdl_joint = elem.segment.joint();
    if (kdl_joint.type != kdl::Joint::ROTATION)
      continue;
    const auto urdf_joint = robot_.urdf()->getJoint(kdl_joint.name);

    // 推力の作用線
    const auto arrow_start = kdl::Vector::Zero();
    const auto arrow_end = kdl::vectorUrdfToKdl(urdf_joint->axis) * kArrowLength;
    const auto arrow_scale = kdl::Vector(0.1, 0.2, 0.3) * kArrowLength;

    // マーカを作成
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = link_name;
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

  // マーカを発行開始
  publish_markers_timer_.start(100);
}

void RotorMarkerPublisher::setAction(const QString& link_name, int action)
{
  for (auto& marker : markers_.markers)
  {
    if (marker.header.frame_id == link_name.toStdString())
    {
      marker.action = action;
      return;
    }
  }

  qWarning() << link_name << " not found.";
}

void RotorMarkerPublisher::publishTimerCb()
{
  // Fill timestamps
  const auto now = node_->get_clock()->now();
  for (auto& marker : markers_.markers)
    marker.header.stamp = now;

  // Publish markers
  auto markers_ptr = make_unique<visualization_msgs::msg::MarkerArray>(markers_);
  markers_pub_->publish(std::move(markers_ptr));
}

void RotorMarkerPublisher::onRotorLinkAdded(const QString& link_name)
{
  // 指定リンクのマーカを表示
  setAction(link_name, visualization_msgs::msg::Marker::ADD);
}

void RotorMarkerPublisher::onRotorLinkRemoved(const QString& link_name)
{
  // 指定リンクのマーカを非表示
  setAction(link_name, visualization_msgs::msg::Marker::DELETE);
}
}  // namespace sa
}  // namespace gui
