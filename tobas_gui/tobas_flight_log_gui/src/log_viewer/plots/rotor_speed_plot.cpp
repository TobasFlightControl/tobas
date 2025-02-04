

#include <tobas_ros2_tools/time.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/rotor_speed_plot.hpp"

namespace gui
{
namespace log
{
RotorSpeedPlotWidget::RotorSpeedPlotWidget()
{
  grid_ = new QGridLayout();
}

void RotorSpeedPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_)
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void RotorSpeedPlotWidget::setData(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  if (msgs.size() == 0)
    return;

  const auto& rotor_states = msgs.at(0).states;
  if (rotor_states.size() != num_rotors_)
  {
    clear();

    num_rotors_ = rotor_states.size();

    // TODO
  }

  for (const auto& msg : msgs)
  {
    if (msg.states.size() != num_rotors_)
    {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    // TODO
  }
}

void RotorSpeedPlotWidget::clear()
{
  qt::clearLayout(grid_);

  for (auto cur_speed_curve : cur_speed_curves_)
    delete cur_speed_curve;

  for (auto tar_speed_curve : tar_speed_curves_)
    delete tar_speed_curve;

  plots_.clear();
  cur_speed_curves_.clear();
  tar_speed_curves_.clear();
}
}  // namespace log
}  // namespace gui
