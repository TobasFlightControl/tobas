#include "tobas_control_system/rcin_viewer/throttles_viewer.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace gcs
{
namespace rcin
{
ThrottlesViewer::ThrottlesViewer(rclcpp::Node::SharedPtr node) : node_(node)
{
  roll_range_ = new qt::HPositionBarWidget(tobas::kRcInputMin, tobas::kRcInputMax);
  pitch_range_ = new qt::VPositionBarWidget(tobas::kRcInputMax, tobas::kRcInputMin);
  yaw_range_ = new qt::HPositionBarWidget(tobas::kRcInputMax, tobas::kRcInputMin);
  throt_range_ = new qt::VPositionBarWidget(tobas::kRcInputMax, tobas::kRcInputMin);

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

  // Connection
  connect(this, &self::rcInputReceived, this, &self::rcInputCbQt, Qt::QueuedConnection);
}

void ThrottlesViewer::reset()
{
  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();
}

void ThrottlesViewer::updateNamespace(const std::string& ns)
{
  reset();

  rcin_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kRcInputTopic), &self::rcInputCbRos, this);
}

void ThrottlesViewer::rcInputCbRos(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin)
{
  Q_EMIT rcInputReceived(rcin->roll, rcin->pitch, rcin->yaw, rcin->throttle, rcin->enable);
}

void ThrottlesViewer::rcInputCbQt(double roll, double pitch, double yaw, double throttle, bool enable)
{
  roll_range_->setValue(roll);
  pitch_range_->setValue(pitch);
  yaw_range_->setValue(yaw);
  throt_range_->setValue(throttle);

  if (enable) {
    roll_range_->setValueLineColor(kLineColorEnable);
    pitch_range_->setValueLineColor(kLineColorEnable);
    yaw_range_->setValueLineColor(kLineColorEnable);
    throt_range_->setValueLineColor(kLineColorEnable);
  }
  else {
    roll_range_->setValueLineColor(kLineColorDisable);
    pitch_range_->setValueLineColor(kLineColorDisable);
    yaw_range_->setValueLineColor(kLineColorDisable);
    throt_range_->setValueLineColor(kLineColorDisable);
  }
}
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
