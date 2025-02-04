#include <QVBoxLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/battery_plot.hpp"

namespace gui
{
namespace log
{
BatteryPlotWidget::BatteryPlotWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  voltage_curve_ = new QwtPlotCurve("Voltage");
  current_curve_ = new QwtPlotCurve("Current");

  voltage_plot_ = new QwtPlot2();
  current_plot_ = new QwtPlot2();

  voltage_curve_->setPen(Qt::black, kLineWidth);
  current_curve_->setPen(Qt::black, kLineWidth);

  voltage_curve_->attach(voltage_plot_);
  current_curve_->attach(current_plot_);

  rows->addWidget(voltage_plot_);
  rows->addWidget(current_plot_);
}

void BatteryPlotWidget::setTimeScale(double t_start, double t_stop)
{
  voltage_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  current_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void BatteryPlotWidget::setData(const QVector<tobas_msgs::msg::Battery>& batt_msgs)
{
  QVector<double> t_data;
  QVector<double> voltage_data;
  QVector<double> current_data;

  for (const auto& batt : batt_msgs)
  {
    t_data.push_back(ros2::seconds(batt.header.stamp));

    voltage_data.push_back(batt.voltage);
    current_data.push_back(batt.current);
  }

  voltage_curve_->setSamples(t_data, voltage_data);
  current_curve_->setSamples(t_data, current_data);

  voltage_plot_->replot();
  current_plot_->replot();
}
}  // namespace log
}  // namespace gui
