#pragma once

#include <QWidget>

namespace gui
{
namespace log
{
class PlaybackControlWidget : public QWidget
{
  Q_OBJECT

  using self = PlaybackControlWidget;
  using super = QWidget;

public:
  explicit PlaybackControlWidget();

private:
};
}  // namespace log
}  // namespace gui
