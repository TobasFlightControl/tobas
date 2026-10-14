// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

namespace tobas
{
namespace gui
{
namespace log
{
class PlaybackControlWidget : public QWidget
{
  Q_OBJECT

  using self = PlaybackControlWidget;

Q_SIGNALS:
  void timeChanged(double _sec);

public:
  explicit PlaybackControlWidget();

  void reset();

  double getDuration() const;
  double getCurrentTime() const;

  void setDuration(double _sec);
  void setCurrentTime(double _sec);

private:
  QPushButton* play_button_;
  QSlider* slider_;
  QLabel* current_time_;
  QLabel* remaining_time_;

  bool is_slider_held_;

  QTimer timer_;

  QString formatTime(int _msec);

  void updateCurrentTimeLabel(int _cur_msec);
  void updateRemainingTimeLabel(int _cur_msec);
  void updateTimeLabels(int _cur_msec);

private Q_SLOTS:
  void onPlayButtonToggled(bool _checked);
  void onSliderValueChanged(int _value);
  void onSliderPressed();
  void onSliderReleased();
  void onTimerTimeout();
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
