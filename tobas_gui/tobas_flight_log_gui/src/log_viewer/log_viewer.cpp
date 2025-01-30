#include <QVBoxLayout>

#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

namespace gui
{
namespace log
{
FlightLogViewerWidget::FlightLogViewerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  for (size_t i = 0; i < plot_tabs_.size(); ++i)
  {
    plot_tabs_[i] = new PlotTabWidget();
    rows->addWidget(plot_tabs_[i]);
  }

  playback_ctrl_ = new PlaybackControlWidget();
  rows->addWidget(playback_ctrl_);
}
}  // namespace log
}  // namespace gui
