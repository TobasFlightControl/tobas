#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/latency_plot.hpp"

namespace gui
{
namespace log
{
LatencyPlotWidget::LatencyPlotWidget()
  : sampling_time_curve_("IMU Sampling Time"), ctrl_latency_curve_("Control Latency")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  sampling_time_plot_ = new QwtPlot2();
  sampling_time_curve_.setPen(Qt::black, kLineWidth);
  sampling_time_curve_.attach(sampling_time_plot_);
  rows->addWidget(sampling_time_plot_);

  ctrl_latency_plot_ = new QwtPlot2();
  ctrl_latency_curve_.setPen(Qt::black, kLineWidth);
  ctrl_latency_curve_.attach(ctrl_latency_plot_);
  rows->addWidget(ctrl_latency_plot_);
}

void LatencyPlotWidget::setTimeScale(double t_start, double t_stop)
{
  sampling_time_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  ctrl_latency_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void LatencyPlotWidget::setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs)
{
  QVector<double> t_data;
  QVector<double> sampling_time_data;

  for (const auto& sampling_time : sampling_time_msgs) {
    t_data.push_back(ros2::seconds(sampling_time.header.stamp));
    sampling_time_data.push_back(ros2::microseconds(sampling_time.data));
  }

  sampling_time_curve_.setSamples(t_data, sampling_time_data);
  sampling_time_plot_->replot();
}

void LatencyPlotWidget::setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs)
{
  QVector<double> t_data;
  QVector<double> ctrl_latency_data;

  for (const auto& ctrl_latency : ctrl_latency_msgs) {
    t_data.push_back(ros2::seconds(ctrl_latency.header.stamp));
    ctrl_latency_data.push_back(ros2::microseconds(ctrl_latency.data));
  }

  ctrl_latency_curve_.setSamples(t_data, ctrl_latency_data);
  ctrl_latency_plot_->replot();
}
}  // namespace log
}  // namespace gui
