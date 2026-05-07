// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/base.hpp"

#include <QTimer>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
BaseWidget::BaseWidget()
{
  setBackgroundColor(QPalette::Base);

  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(cmn::kTitlePSize, QFont::Bold));

  rows_ = new QVBoxLayout();
  rows_->addWidget(title_, 0, Qt::AlignTop);
  rows_->addSpacing(30);

  setLayout(rows_);

  QTimer::singleShot(0, this, &BaseWidget::initialize);
}

void BaseWidget::initialize()
{
  title_->setText(title());
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
