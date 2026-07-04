// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/health_viewer/status.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QPalette>

#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
StatusWidget::StatusWidget(const QString& text)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

  status_ = new qt::FramedLabel();
  status_->setAlignment(Qt::AlignCenter);
  status_->setFixedWidth(kStatusWidth);

  label_ = new qt::Label(text, kTextPSize);

  reset();

  const auto cols = new QHBoxLayout();
  cols->addWidget(status_);
  cols->addWidget(label_);
  setLayout(cols);
}

void StatusWidget::reset()
{
  setStatusText("---", palette().color(QPalette::Disabled, QPalette::WindowText));
}

void StatusWidget::setStatus(Status status)
{
  switch (status) {
    case PASSED:
      setStatusText("OK", Qt::darkGreen);
      break;
    case FAILED:
      setStatusText("NG", Qt::darkRed);
      break;
    case IGNORED:
      setStatusText("Skip", Qt::darkYellow);
      break;
    case UNKNOWN:
      reset();
      break;
    default:
      qWarning() << "Unknown status:" << status;
      reset();
      break;
  }
}

void StatusWidget::setStatus(uint8_t status)
{
  setStatus(static_cast<Status>(status));
}

void StatusWidget::setStatus(bool ok)
{
  const auto status = ok ? PASSED : FAILED;
  setStatus(status);
}

void StatusWidget::setStatusText(const QString& text, const QColor& color)
{
  status_->setText(text);

  auto pal = status_->palette();
  pal.setColor(QPalette::WindowText, color);
  status_->setPalette(pal);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
