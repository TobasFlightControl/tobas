#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/engine_plot.hpp"

namespace gui
{
namespace log
{
EnginePlotWidget::EnginePlotWidget() : throttle_curve_("Throttle")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  throttle_plot_ = new QwtPlot2();
  throttle_curve_.setPen(Qt::black, kLineWidth);
  throttle_curve_.attach(throttle_plot_);
  rows->addWidget(throttle_plot_);
}

void EnginePlotWidget::setTimeScale(double t_start, double t_stop)
{
  throttle_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void EnginePlotWidget::setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs)
{
  QVector<double> t_data;
  QVector<double> throttle_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    throttle_data.push_back(msg.engine_throttle);
  }

  throttle_curve_.setSamples(t_data, throttle_data);
  throttle_plot_->replot();
}
}  // namespace log
}  // namespace gui
