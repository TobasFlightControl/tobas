// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/playback_control.hpp"

#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QStyle>

#include <tobas_std_tools/check.hpp>

namespace tobas::gui::log
{
namespace
{
constexpr int kTimerInterval = 40;  // [ms]

QString formatTime(int _msec)
{
  auto seconds = _msec / 1000;
  const auto minutes = seconds / 60;
  seconds %= 60;

  return QString("%1:%2").arg(minutes, 2, 10, QLatin1Char('0')).arg(seconds, 2, 10, QLatin1Char('0'));
}
}  // namespace

PlaybackControlWidget::PlaybackControlWidget()
{
  play_button_ = new QPushButton();
  play_button_->setCheckable(true);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setMinimum(0);

  current_time_ = new QLabel();
  remaining_time_ = new QLabel();

  // Layout
  const auto cols = new QHBoxLayout(this);
  cols->setContentsMargins(0, 0, 0, 0);
  cols->addWidget(play_button_);
  cols->addWidget(current_time_);
  cols->addWidget(slider_);
  cols->addWidget(remaining_time_);

  // Connection
  connect(play_button_, &QPushButton::toggled, this, &self::onPlayButtonToggled);
  connect(slider_, &QSlider::valueChanged, this, &self::onSliderValueChanged);
  connect(&timer_, &QTimer::timeout, this, &self::onTimerTimeout);
}

void PlaybackControlWidget::reset()
{
  play_button_->setChecked(false);
  play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));

  // The viewer clears its plots separately, so resetting the slider must not trigger a redraw.
  const QSignalBlocker slider_blocker(slider_);
  slider_->setSliderDown(false);
  slider_->setMaximum(0);

  current_time_->setText("00:00");
  remaining_time_->setText("00:00");

  timer_.stop();
}

double PlaybackControlWidget::getDuration() const
{
  return slider_->maximum() * 1e-3;
}

double PlaybackControlWidget::getCurrentTime() const
{
  return slider_->value() * 1e-3;
}

void PlaybackControlWidget::setDuration(double _sec)
{
  TOBAS_CHECK(_sec >= 0.0);

  slider_->setMaximum(_sec * 1e+3);
  updateTimeLabels(slider_->value());
}

void PlaybackControlWidget::setCurrentTime(double _sec)
{
  TOBAS_CHECK(_sec >= 0.0);

  slider_->setValue(_sec * 1e+3);
}

void PlaybackControlWidget::updateTimeLabels(int _cur_msec)
{
  current_time_->setText(formatTime(_cur_msec));
  remaining_time_->setText(formatTime(slider_->maximum() - _cur_msec));
}

void PlaybackControlWidget::onPlayButtonToggled(bool _checked)
{
  if (_checked) {
    // If playback has finished, return to the beginning and start.
    if (slider_->value() == slider_->maximum()) {
      slider_->setValue(0);
    }

    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    timer_.start(kTimerInterval);
  }
  else {
    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    timer_.stop();
  }
}

void PlaybackControlWidget::onSliderValueChanged(int _value)
{
  updateTimeLabels(_value);
  Q_EMIT timeChanged(_value * 1e-3);
}

void PlaybackControlWidget::onTimerTimeout()
{
  if (slider_->isSliderDown()) {
    return;
  }

  auto next_time = slider_->value() + kTimerInterval;  // [ms]

  // Stop at the end.
  if (next_time > slider_->maximum()) {
    next_time = slider_->maximum();
    play_button_->setChecked(false);
  }

  slider_->setValue(next_time);
}
}  // namespace tobas::gui::log
