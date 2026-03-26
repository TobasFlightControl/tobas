#include "tobas_flight_log_gui/log_viewer/plots/latency_plot.hpp"

#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
LatencyPlotWidget::LatencyPlotWidget()
  : sampling_time_curve_("IMU Sampling Time [us]"), ctrl_latency_curve_("Control Latency [us]")
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  sampling_time_plot_ = new QwtPlot2();
  sampling_time_plot_->setAxisNoLabel(QwtPlot::xBottom);
  sampling_time_plot_->setAxisScale(QwtPlot::yLeft, 5, 2500);
  sampling_time_curve_.setPen(Qt::black, kLineWidth);
  sampling_time_curve_.attach(sampling_time_plot_);
  rows->addWidget(sampling_time_plot_, 1);

  ctrl_latency_plot_ = new QwtPlot2();
  ctrl_latency_plot_->setAxisNoLabel(QwtPlot::xBottom);
  ctrl_latency_plot_->setAxisScale(QwtPlot::yLeft, 0, 2000);
  ctrl_latency_curve_.setPen(Qt::black, kLineWidth);
  ctrl_latency_curve_.attach(ctrl_latency_plot_);
  rows->addWidget(ctrl_latency_plot_, 1);
}

void LatencyPlotWidget::clear()
{
  sampling_time_curve_.clear();
  sampling_time_plot_->replot();

  ctrl_latency_curve_.clear();
  ctrl_latency_plot_->replot();
}

void LatencyPlotWidget::setTimeScale(double t_start, double t_stop)
{
  sampling_time_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  ctrl_latency_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void LatencyPlotWidget::setData(
  const QVector<tobas_msgs::msg::Latency>& sampling_time_msgs,
  const QVector<tobas_msgs::msg::Latency>& ctrl_latency_msgs)
{
  setSamplingTimeData(sampling_time_msgs);
  setControlLatencyData(ctrl_latency_msgs);
}

void LatencyPlotWidget::setSamplingTimeData(const QVector<tobas_msgs::msg::Latency>& msgs)
{
  QVector<double> t_data;
  QVector<double> val_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    val_data.push_back(ros2::microseconds(msg.data));
  }

  sampling_time_curve_.setSamples(t_data, val_data);
  sampling_time_plot_->replot();
}

void LatencyPlotWidget::setControlLatencyData(const QVector<tobas_msgs::msg::Latency>& msgs)
{
  QVector<double> t_data;
  QVector<double> val_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));
    val_data.push_back(ros2::microseconds(msg.data));
  }

  ctrl_latency_curve_.setSamples(t_data, val_data);
  ctrl_latency_plot_->replot();
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
