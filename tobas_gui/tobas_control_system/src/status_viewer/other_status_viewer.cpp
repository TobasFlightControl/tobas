#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_control_system/status_viewer/other_status_viewer.hpp"

namespace gui
{
namespace control_system
{
OtherStatusViewerWidget::OtherStatusViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  arming_status_ = new StatusWidget("Rotors Armed");
  gps_status_ = new StatusWidget("GPS 3D Fix");

  const auto rows = new QVBoxLayout();
  rows->addWidget(arming_status_);
  rows->addWidget(gps_status_);
  rows->addStretch();

  setLayout(rows);
}

void OtherStatusViewerWidget::updateNamespace(const std::string& ns)
{
  gps_status_->reset();
  arming_status_->reset();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);
  gps_sub_ =
    ros2::createSubscriber(node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kGNSSTopic), &self::gpsCb, this);
}

void OtherStatusViewerWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_status_->setStatus(arming->data);
}

void OtherStatusViewerWidget::gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps)
{
  gps_status_->setStatus(gps->fix_type == tobas_msgs::msg::Gps::FIX_3D);
}
}  // namespace control_system
}  // namespace gui
