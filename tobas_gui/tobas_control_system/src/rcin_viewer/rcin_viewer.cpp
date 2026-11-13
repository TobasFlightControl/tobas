// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPalette>
#include <QVBoxLayout>

#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas::gui::ctrl::rcin
{
RCInputViewerWidget::RCInputViewerWidget(const rqt::RosQtBridge& _bridge)
{
  yaw_pitch_ = new StickPanel("Yaw / Pitch", "Yaw", "Pitch");
  roll_throttle_ = new StickPanel("Roll / Throttle", "Roll", "Throttle");

  kill_ = makeBadge();
  sub_mode_ = makeBadge();
  acrobat_ = makeBadge();
  stabilize_ = makeBadge();
  loiter_ = makeBadge();

  constexpr int kModeBatchMinWidth = 44;
  acrobat_->setMinimumWidth(kModeBatchMinWidth);
  stabilize_->setMinimumWidth(kModeBatchMinWidth);
  loiter_->setMinimumWidth(kModeBatchMinWidth);

  // Layout
  const auto switch_cols = new QHBoxLayout();
  switch_cols->addWidget(kill_, 1);
  switch_cols->addWidget(sub_mode_, 1);

  const auto mode_cols = new QHBoxLayout();
  mode_cols->setSpacing(0);
  mode_cols->addWidget(acrobat_, 1);
  mode_cols->addWidget(stabilize_, 1);
  mode_cols->addWidget(loiter_, 1);

  const auto status_rows = new QVBoxLayout();
  status_rows->setSpacing(0);
  status_rows->addLayout(switch_cols, 5);
  status_rows->addLayout(mode_cols, 8);

  const auto cols = new QHBoxLayout(this);
  cols->setSpacing(4);
  cols->setContentsMargins(2, 0, 2, 0);
  cols->addWidget(yaw_pitch_, 3);
  cols->addWidget(roll_throttle_, 3);
  cols->addLayout(status_rows, 1);

  // Connection
  connect(&_bridge, &rqt::RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void RCInputViewerWidget::reset()
{
  yaw_pitch_->reset();
  roll_throttle_->reset();

  setBadge(kill_, "Kill: ---", qt::color::gray90());
  setBadge(sub_mode_, "Sub: ---", qt::color::gray90());
  setMode(static_cast<FlightMode>(-1));
}

QLabel* RCInputViewerWidget::makeBadge() const
{
  const auto badge = new QLabel();
  badge->setAlignment(Qt::AlignCenter);
  badge->setAutoFillBackground(true);
  badge->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  return badge;
}

void RCInputViewerWidget::setBadge(QLabel* _badge, const QString& _text, const QColor& _background)
{
  _badge->setText(_text);

  auto pal = _badge->palette();
  pal.setColor(QPalette::Window, _background);
  pal.setColor(QPalette::WindowText, palette().color(QPalette::WindowText));
  _badge->setPalette(pal);
}

void RCInputViewerWidget::setMode(FlightMode _mode)
{
  setBadge(acrobat_, "Acro", _mode == FlightMode::kAcrobat ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(stabilize_, "Stab", _mode == FlightMode::kStabilize ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(loiter_, "Loit", _mode == FlightMode::kLoiter ? qt::color::lightBlue() : qt::color::gray90());
}

void RCInputViewerWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& _rcin)
{
  if (_rcin->status != tobas_msgs::msg::RCInput::STATUS_OK) {
    reset();
    return;
  }

  yaw_pitch_->setValues(-_rcin->yaw, _rcin->pitch, true, _rcin->enable);
  roll_throttle_->setValues(_rcin->roll, _rcin->throttle, true, _rcin->enable);

  setBadge(
    kill_,
    QString("Kill: ") + (_rcin->kill ? "ON" : "OFF"),
    _rcin->enable && _rcin->kill ? qt::color::lightRed() : qt::color::gray90());
  setBadge(
    sub_mode_,
    QString("Sub: ") + (_rcin->sub_mode ? "ON" : "OFF"),
    _rcin->enable && _rcin->sub_mode ? qt::color::lightBlue() : qt::color::gray90());
  setMode(_rcin->enable ? _rcin->mode : static_cast<FlightMode>(-1));
}
}  // namespace tobas::gui::ctrl::rcin
