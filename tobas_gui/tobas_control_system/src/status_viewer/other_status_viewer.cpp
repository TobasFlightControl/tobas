#include "tobas_control_system/status_viewer/other_status_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

namespace gui
{
namespace gcs
{
OtherStatusViewerWidget::OtherStatusViewerWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  arming_status_ = new StatusWidget("Rotors Armed");
  gnss_status_ = new StatusWidget("GNSS 3D Fix");

  const auto rows = new QVBoxLayout();
  rows->addWidget(arming_status_);
  rows->addWidget(gnss_status_);
  rows->addStretch();

  setLayout(rows);
}

void OtherStatusViewerWidget::reset()
{
  arming_status_->reset();
  gnss_status_->reset();
}

void OtherStatusViewerWidget::updateNamespace(const std::string& ns)
{
  reset();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);
  gnss_sub_ =
    ros2::createSubscriber(node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kGnssTopic), &self::gnssCb, this);
}

void OtherStatusViewerWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_status_->setStatus(arming->data);
}

void OtherStatusViewerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  gnss_status_->setStatus(gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D);
}
}  // namespace gcs
}  // namespace gui
