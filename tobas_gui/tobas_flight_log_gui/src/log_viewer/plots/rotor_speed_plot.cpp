#include <ranges>

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
  setLayout(grid_);
}

void RotorSpeedPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_)
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void RotorSpeedPlotWidget::setData(
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_msgs,
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_msgs)
{
  if (cur_msgs.size() == 0)
    return;

  const auto& cur_states = cur_msgs.at(0);
  if (cur_states.states.size() != num_rotors_)
    updateInternalDataStructures(cur_states);

  updateCurrentSpeedSamples(cur_msgs);
  updateTargetSpeedSamples(tar_msgs);

  for (auto& plot : plots_)
    plot->replot();
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

  num_rotors_ = 0;
  channel2idx_.clear();
}

void RotorSpeedPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg)
{
  clear();

  num_rotors_ = msg.states.size();

  plots_.resize(num_rotors_);
  cur_speed_curves_.resize(num_rotors_);
  tar_speed_curves_.resize(num_rotors_);

  for (const auto& [idx, state] : std::views::enumerate(msg.states))
  {
    channel2idx_[state.channel] = idx;

    plots_[idx] = new QwtPlot2();

    // N行2列の格子状に配置
    grid_->addWidget(plots_[idx], idx / 2, idx % 2);

    cur_speed_curves_[idx] = new qwt::QwtPlotCurveWrapper("Current Speed (CH" + QString::number(state.channel) + ")");
    cur_speed_curves_[idx]->setPen(kCurrentValueColor, kLineWidth);
    cur_speed_curves_[idx]->attach(plots_[idx]);

    tar_speed_curves_[idx] = new qwt::QwtPlotCurveWrapper("Target Speed (CH" + QString::number(state.channel) + ")");
    tar_speed_curves_[idx]->setPen(kTargetValueColor, kLineWidth);
    tar_speed_curves_[idx]->attach(plots_[idx]);
  }
}

void RotorSpeedPlotWidget::updateCurrentSpeedSamples(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);

  for (const auto& msg : msgs)
  {
    if (msg.states.size() != num_rotors_)
    {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& state : msg.states)
    {
      if (!channel2idx_.contains(state.channel))
      {
        qWarning() << "Rotor channel " << QString::number(state.channel) << " is not registered.";
        continue;
      }

      if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
        continue;

      const auto& idx = channel2idx_[state.channel];

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(state.speed);
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i)
    cur_speed_curves_[i]->setSamples(t_data[i], speed_data[i]);
}

void RotorSpeedPlotWidget::updateTargetSpeedSamples(const QVector<tobas_msgs::msg::RotorSpeedArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);

  for (const auto& msg : msgs)
  {
    if (msg.speeds.size() != num_rotors_)
    {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& speed : msg.speeds)
    {
      if (!channel2idx_.contains(speed.channel))
      {
        qWarning() << "Rotor channel " << QString::number(speed.channel) << " is not registered.";
        continue;
      }

      const auto& idx = channel2idx_[speed.channel];

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(speed.speed);
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i)
    tar_speed_curves_[i]->setSamples(t_data[i], speed_data[i]);
}
}  // namespace log
}  // namespace gui
