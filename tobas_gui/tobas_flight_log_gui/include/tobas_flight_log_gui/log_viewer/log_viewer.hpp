#pragma once

#include "./plot_tab.hpp"
#include "./playback_control.hpp"

namespace gui
{
namespace log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;
  using super = QWidget;

public:
  explicit FlightLogViewerWidget();

private:
  std::array<PlotTabWidget*, 2> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;
};
}  // namespace log
}  // namespace gui
