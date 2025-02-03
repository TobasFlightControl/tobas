#include <QVBoxLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/latency_plot.hpp"

namespace gui
{
namespace log
{
LatencyPlotWidget::LatencyPlotWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  curve_ = new QwtPlotCurve("Latency");
  plot_ = new QwtPlot2();
  curve_->setPen(kDefaultColor, kLineWidth);
  curve_->attach(plot_);
  rows->addWidget(plot_);
}

void LatencyPlotWidget::setTimeScale(double t_start, double t_stop)
{
  plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void LatencyPlotWidget::setData(const QVector<tobas_msgs::msg::Latency>& latency_msgs)
{
  QVector<double> t_data;
  QVector<double> latency_data;

  for (const auto& latency : latency_msgs)
  {
    t_data.push_back(ros2::seconds(latency.header.stamp));
    latency_data.push_back(ros2::microseconds(latency.data));
  }

  curve_->setSamples(t_data, latency_data);

  plot_->replot();
}
}  // namespace log
}  // namespace gui
