#include "tobas_control_system/status_viewer/other_status_viewer.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace gcs
{
OtherStatusViewerWidget::OtherStatusViewerWidget(const RosQtBridge& bridge)
{
  arming_status_ = new StatusWidget("Rotors Armed");
  gnss_status_ = new StatusWidget("GNSS 3D Fix");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(arming_status_);
  rows->addWidget(gnss_status_);
  rows->addStretch();
  setLayout(rows);

  // Connection
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
}

void OtherStatusViewerWidget::reset()
{
  arming_status_->reset();
  gnss_status_->reset();
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
