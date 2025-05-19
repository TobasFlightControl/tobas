#include "tobas_control_system/rcin_viewer/throttles_viewer.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace gcs
{
namespace rcin
{
ThrottlesViewer::ThrottlesViewer(const RosQtBridge& bridge)
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
  connect(&bridge, &RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void ThrottlesViewer::reset()
{
  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();
}

void ThrottlesViewer::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  roll_range_->setValue(rcin->roll);
  pitch_range_->setValue(rcin->pitch);
  yaw_range_->setValue(rcin->yaw);
  throt_range_->setValue(rcin->throttle);

  if (rcin->enable) {
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
