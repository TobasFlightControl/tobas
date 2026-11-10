// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/gnss_viewer.hpp"

#include <QDebug>
#include <QFormLayout>

#include <tobas_qt_tools/widgets/progress_bar.hpp>

namespace tobas::gui::ctrl
{
GnssViewerWidget::GnssViewerWidget(const rqt::RosQtBridge& _bridge)
{
  status_ = new qt::FramedLabel();
  nsats_ = new qt::FramedLabel();

  constexpr int kLabelMinWidth = 48;
  const auto row_height = qt::ProgressBar().sizeHint().height();
  status_->setMinimumSize(kLabelMinWidth, row_height);
  nsats_->setMinimumSize(kLabelMinWidth, row_height);

  // Layout
  const auto form = new QFormLayout(this);
  form->setContentsMargins(1, 0, 1, 0);
  form->setHorizontalSpacing(2);
  form->setVerticalSpacing(1);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->addRow("GNSS Status", status_);
  form->addRow("GNSS NSats", nsats_);

  // Connection
  connect(&_bridge, &rqt::RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
}

void GnssViewerWidget::reset()
{
  status_->clear();
  nsats_->clear();
}

void GnssViewerWidget::setStatus(uint8_t _status)
{
  switch (_status) {
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
      qWarning() << "Unknown status:" << (int)_status;
      status_->setText("Unknown");
      break;
  }
}

void GnssViewerWidget::setNSats(int _nsats)
{
  nsats_->setText(QString::number(_nsats));
}

void GnssViewerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& _gnss)
{
  setStatus(_gnss->fix_type);
  setNSats(_gnss->num_satellites_used);
}
}  // namespace tobas::gui::ctrl
