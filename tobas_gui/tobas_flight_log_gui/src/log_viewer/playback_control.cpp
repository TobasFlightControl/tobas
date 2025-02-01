#include <QStyle>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/log_viewer/playback_control.hpp"

namespace gui
{
namespace log
{
PlaybackControlWidget::PlaybackControlWidget()
{
  play_button_ = new QPushButton();
  play_button_->setCheckable(true);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setMinimum(0);

  current_time_ = new QLabel();
  remaining_time_ = new QLabel();

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(play_button_);
  cols->addWidget(current_time_);
  cols->addWidget(slider_);
  cols->addWidget(remaining_time_);
  setLayout(cols);

  // Connection
  connect(play_button_, &QPushButton::toggled, this, &self::onPlayButtonToggled);
  connect(slider_, &QSlider::valueChanged, this, &self::onSliderValueChanged);
  connect(slider_, &QSlider::sliderPressed, this, &self::onSliderPressed);
  connect(slider_, &QSlider::sliderReleased, this, &self::onSliderReleased);
  connect(&timer_, &QTimer::timeout, this, &self::onTimerTimeout);
}

void PlaybackControlWidget::reset()
{
  play_button_->setChecked(false);
  play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));

  slider_->setMaximum(0);

  current_time_->setText("00:00");
  remaining_time_->setText("00:00");

  is_slider_held_ = false;

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

void PlaybackControlWidget::setDuration(double sec)
{
  TOBAS_CHECK(sec >= 0.);

  slider_->setMaximum(sec * 1e+3);
  updateTimeLabels(slider_->value());
}

void PlaybackControlWidget::setCurrentTime(double sec)
{
  TOBAS_CHECK(sec >= 0.);

  slider_->setValue(sec * 1e+3);
}

void PlaybackControlWidget::updateCurrentTimeLabel(int cur_msec)
{
  current_time_->setText(formatTime(cur_msec));
}

void PlaybackControlWidget::updateRemainingTimeLabel(int cur_msec)
{
  remaining_time_->setText(formatTime(slider_->maximum() - cur_msec));
}

void PlaybackControlWidget::updateTimeLabels(int cur_msec)
{
  updateCurrentTimeLabel(cur_msec);
  updateRemainingTimeLabel(cur_msec);
}

QString PlaybackControlWidget::formatTime(int msec)
{
  auto seconds = msec / 1000;
  const auto minutes = seconds / 60;
  seconds %= 60;

  return QString("%1:%2").arg(minutes, 2, 10, QLatin1Char('0')).arg(seconds, 2, 10, QLatin1Char('0'));
}

void PlaybackControlWidget::onPlayButtonToggled(bool checked)
{
  if (checked)
  {
    // 終了してたら先頭に戻してスタート
    if (slider_->value() == slider_->maximum())
      slider_->setValue(0);

    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    timer_.start(kTimerInterval);
  }
  else
  {
    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    timer_.stop();
  }

  Q_EMIT timeChanged(getCurrentTime());
}

void PlaybackControlWidget::onSliderValueChanged(int value)
{
  updateTimeLabels(value);
  Q_EMIT timeChanged(value * 1e-3);
}

void PlaybackControlWidget::onSliderPressed()
{
  is_slider_held_ = true;
}

void PlaybackControlWidget::onSliderReleased()
{
  is_slider_held_ = false;
}

void PlaybackControlWidget::onTimerTimeout()
{
  if (is_slider_held_)
    return;

  auto next_time = slider_->value() + kTimerInterval;  // [ms]

  // 終了時点で停止
  if (next_time > slider_->maximum())
  {
    next_time = slider_->maximum();
    play_button_->setChecked(false);
  }

  slider_->setValue(next_time);
}
}  // namespace log
}  // namespace gui
