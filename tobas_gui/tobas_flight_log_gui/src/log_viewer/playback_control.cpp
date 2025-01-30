#include <QStyle>
#include <QHBoxLayout>

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

  reset();
}

void PlaybackControlWidget::reset()
{
  play_button_->setChecked(false);
  play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));

  slider_->setMaximum(0);

  current_time_->setText("00:00");
  remaining_time_->setText("00:00");
}

int PlaybackControlWidget::getDuration() const
{
  return slider_->maximum();
}

int PlaybackControlWidget::getCurrentTime() const
{
  return slider_->value();
}

void PlaybackControlWidget::setDuration(int msec)
{
  slider_->setMaximum(msec);
  updateTimeLabels(getCurrentTime());
}

void PlaybackControlWidget::setCurrentTime(int msec)
{
  slider_->setValue(msec);
}

void PlaybackControlWidget::updateCurrentTimeLabel(int cur_time)
{
  current_time_->setText(formatTime(cur_time));
}

void PlaybackControlWidget::updateRemainingTimeLabel(int cur_time)
{
  remaining_time_->setText(formatTime(getDuration() - cur_time));
}

void PlaybackControlWidget::updateTimeLabels(int cur_time)
{
  updateCurrentTimeLabel(cur_time);
  updateRemainingTimeLabel(cur_time);
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
    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
  else
    play_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));

  Q_EMIT timeChanged(getCurrentTime());
}

void PlaybackControlWidget::onSliderValueChanged(int value)
{
  updateTimeLabels(value);
  Q_EMIT timeChanged(value);
}
}  // namespace log
}  // namespace gui
