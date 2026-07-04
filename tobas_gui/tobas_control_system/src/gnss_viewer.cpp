// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/gnss_viewer.hpp"

#include <QDebug>

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
GnssViewerWidget::GnssViewerWidget(const RosQtBridge& bridge)
{
  status_ = new qt::FramedLabel();
  nsats_ = new qt::FramedLabel();

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Status", kLabelPSize), status_);
  form->addVAlignedRow(new qt::Label("NSats", kLabelPSize), nsats_);
  setLayout(form);

  // Connection
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
}

void GnssViewerWidget::reset()
{
  status_->clear();
  nsats_->clear();
}

void GnssViewerWidget::setStatus(uint8_t status)
{
  switch (status) {
    case tobas_msgs::msg::Gnss::NO_FIX:
      status_->setText("No Fix");
      break;
    case tobas_msgs::msg::Gnss::DEAD_RECHONING_ONLY:
      status_->setText("Dead Rechoning Only");
      break;
    case tobas_msgs::msg::Gnss::FIX_2D:
      status_->setText("Fix 2D");
      break;
    case tobas_msgs::msg::Gnss::FIX_3D:
      status_->setText("Fix 3D");
      break;
    case tobas_msgs::msg::Gnss::GPS_DEAD_RECHONING_COMBINED:
      status_->setText("Dead Rechoning Combined");
      break;
    case tobas_msgs::msg::Gnss::TIME_ONLY_FIX:
      status_->setText("Time Fix Only");
      break;
    default:
      qWarning() << "Unknown status:" << (int)status;
      status_->setText("Unknown");
      break;
  }
}

void GnssViewerWidget::setNSats(int nsats)
{
  nsats_->setText(QString::number(nsats));
}

void GnssViewerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  setStatus(gnss->fix_type);
  setNSats(gnss->num_satellites_used);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
