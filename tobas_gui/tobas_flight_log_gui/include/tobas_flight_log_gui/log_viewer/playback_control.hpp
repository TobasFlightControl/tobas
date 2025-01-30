#pragma once

#include <QPushButton>
#include <QSlider>
#include <QLabel>

namespace gui
{
namespace log
{
class PlaybackControlWidget : public QWidget
{
  Q_OBJECT

  using self = PlaybackControlWidget;
  using super = QWidget;

Q_SIGNALS:
  void timeChanged(int msec);

public:
  explicit PlaybackControlWidget();

  void reset();

  int getDuration() const;
  int getCurrentTime() const;

  void setDuration(int msec);
  void setCurrentTime(int msec);

private:
  QPushButton* play_button_;
  QSlider* slider_;
  QLabel* current_time_;
  QLabel* remaining_time_;

  QString formatTime(int msec);

  void updateCurrentTimeLabel(int cur_time);
  void updateRemainingTimeLabel(int cur_time);
  void updateTimeLabels(int cur_time);

private Q_SLOTS:
  void onPlayButtonToggled(bool checked);
  void onSliderValueChanged(int value);
};
}  // namespace log
}  // namespace gui
