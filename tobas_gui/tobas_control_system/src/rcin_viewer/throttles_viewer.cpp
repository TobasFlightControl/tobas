#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_control_system/rcin_viewer/throttles_viewer.hpp"

namespace gui
{
namespace control_system
{
namespace rcin
{
ThrottlesViewer::ThrottlesViewer(rclcpp::Node::SharedPtr node) : node_(node)
{
  roll_range_ = new qt::HPositionBarWidget(tobas::kRCInputMin, tobas::kRCInputMax);
  pitch_range_ = new qt::VPositionBarWidget(tobas::kRCInputMin, tobas::kRCInputMax);
  yaw_range_ = new qt::HPositionBarWidget(tobas::kRCInputMin, tobas::kRCInputMax);
  throt_range_ = new qt::VPositionBarWidget(tobas::kRCInputMin, tobas::kRCInputMax);

  roll_range_->setFixedHeight(kRangeSideShort);
  pitch_range_->setFixedWidth(kRangeSideShort);
  yaw_range_->setFixedHeight(kRangeSideShort);
  throt_range_->setFixedWidth(kRangeSideShort);

  // Layout
  const auto cols2 = new QHBoxLayout();
  cols2->addWidget(new QLabel("Pitch"), 0, Qt::AlignLeft);
  cols2->addWidget(new QLabel("Throttle"), 0, Qt::AlignRight);

  const auto rows1 = new QVBoxLayout();
  rows1->addWidget(roll_range_);
  qt::addWidgetCenter(new QLabel("Roll"), rows1);
  rows1->addStretch();
  rows1->addLayout(cols2);
  rows1->addStretch();
  qt::addWidgetCenter(new QLabel("Yaw"), rows1);
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
  rcin_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kRcInputTopic), &self::rcInputCb, this);
}

void ThrottlesViewer::rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin)
{
  roll_range_->setValue(rcin->roll);
  pitch_range_->setValue(rcin->pitch);
  yaw_range_->setValue(rcin->yaw);
  throt_range_->setValue(rcin->throttle);

  roll_range_->update();
  pitch_range_->update();
  yaw_range_->update();
  throt_range_->update();
}
}  // namespace rcin
}  // namespace control_system
}  // namespace gui
