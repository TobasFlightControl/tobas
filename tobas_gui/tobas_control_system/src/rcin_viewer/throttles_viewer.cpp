#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rcin_viewer/throttles_viewer.hpp"

namespace gui
{
namespace gcs
{
namespace rcin
{
ThrottlesViewer::ThrottlesViewer(rclcpp::Node::SharedPtr node) : node_(node)
{
  roll_range_ = new qt::HPositionBarWidget(tobas::kRCInputMin, tobas::kRCInputMax);
  pitch_range_ = new qt::VPositionBarWidget(tobas::kRCInputMax, tobas::kRCInputMin);
  yaw_range_ = new qt::HPositionBarWidget(tobas::kRCInputMax, tobas::kRCInputMin);
  throt_range_ = new qt::VPositionBarWidget(tobas::kRCInputMax, tobas::kRCInputMin);

  roll_range_->setFixedHeight(kRangeSideShort);
  pitch_range_->setFixedWidth(kRangeSideShort);
  yaw_range_->setFixedHeight(kRangeSideShort);
  throt_range_->setFixedWidth(kRangeSideShort);

  roll_range_->setFillRange(false);
  pitch_range_->setFillRange(false);
  yaw_range_->setFillRange(false);
  throt_range_->setFillRange(false);

  // Layout
  const auto cols2 = new QHBoxLayout();
  cols2->addWidget(new qt::Label("Pitch", kLabelPSize), 0, Qt::AlignLeft);
  cols2->addWidget(new qt::Label("Throttle", kLabelPSize), 0, Qt::AlignRight);

  const auto rows1 = new QVBoxLayout();
  rows1->addWidget(roll_range_);
  qt::addWidgetCenter(new qt::Label("Roll", kLabelPSize), rows1);
  rows1->addStretch();
  rows1->addLayout(cols2);
  rows1->addStretch();
  qt::addWidgetCenter(new qt::Label("Yaw", kLabelPSize), rows1);
  rows1->addWidget(yaw_range_);

  const auto cols1 = new QHBoxLayout();
  cols1->addWidget(pitch_range_);
  cols1->addLayout(rows1);
  cols1->addWidget(throt_range_);

  setLayout(cols1);

  reset();
}

void ThrottlesViewer::reset()
{
  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();

  rcin_sub_ = nullptr;
}

void ThrottlesViewer::updateNamespace(const std::string& ns)
{
  reset();
  rcin_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kRcInputTopic), &self::rcInputCb, this);
}

void ThrottlesViewer::rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin)
{
  roll_range_->setValue(rcin->roll);
  pitch_range_->setValue(rcin->pitch);
  yaw_range_->setValue(rcin->yaw);
  throt_range_->setValue(rcin->throttle);
}
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
