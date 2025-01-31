#pragma once

#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QTimer>

namespace gui
{
namespace log
{
class PlaybackControlWidget : public QWidget
{
  Q_OBJECT

  using self = PlaybackControlWidget;
  using super = QWidget;

  static constexpr int kTimerInterval = 40;  // [ms]

Q_SIGNALS:
  void timeChanged(double sec);

public:
  explicit PlaybackControlWidget();

  void reset();

  double getDuration() const;
  double getCurrentTime() const;

  void setDuration(double sec);
  void setCurrentTime(double sec);

private:
  QPushButton* play_button_;
  QSlider* slider_;
  QLabel* current_time_;
  QLabel* remaining_time_;

  bool is_slider_held_;

  QTimer timer_;

  QString formatTime(int msec);

  void updateCurrentTimeLabel(int cur_msec);
  void updateRemainingTimeLabel(int cur_msec);
  void updateTimeLabels(int cur_msec);

private Q_SLOTS:
  void onPlayButtonToggled(bool checked);
  void onSliderValueChanged(int value);
  void onSliderPressed();
  void onSliderReleased();
  void onTimerTimeout();
};
}  // namespace log
}  // namespace gui
