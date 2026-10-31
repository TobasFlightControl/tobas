// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/health_viewer/status.hpp"

#include <QDebug>
#include <QHBoxLayout>
#include <QPalette>

#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas::gui::ctrl
{
StatusWidget::StatusWidget(const QString& _text)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

  status_ = new qt::FramedLabel();
  status_->setAlignment(Qt::AlignCenter);
  status_->setFixedWidth(40);

  label_ = new qt::Label(_text, 12);

  reset();

  const auto cols = new QHBoxLayout(this);
  cols->addWidget(status_);
  cols->addWidget(label_);
}

void StatusWidget::reset()
{
  setStatusText("---", palette().color(QPalette::Disabled, QPalette::WindowText));
}

void StatusWidget::setStatus(Status _status)
{
  switch (_status) {
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
      qWarning() << "Unknown status:" << _status;
      reset();
      break;
  }
}

void StatusWidget::setStatus(uint8_t _status)
{
  setStatus(static_cast<Status>(_status));
}

void StatusWidget::setStatus(bool _ok)
{
  const auto status = _ok ? PASSED : FAILED;
  setStatus(status);
}

void StatusWidget::setStatusText(const QString& _text, const QColor& _color)
{
  status_->setText(_text);

  auto pal = status_->palette();
  pal.setColor(QPalette::WindowText, _color);
  status_->setPalette(pal);
}
}  // namespace tobas::gui::ctrl
