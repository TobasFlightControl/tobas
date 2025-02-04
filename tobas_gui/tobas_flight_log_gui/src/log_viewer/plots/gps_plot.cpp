#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/gps_plot.hpp"

namespace gui
{
namespace log
{
GpsPlotWidget::GpsPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  latitude_curve_ = new QwtPlotCurve("Latitude");
  longitude_curve_ = new QwtPlotCurve("Longitude");
  altitude_curve_ = new QwtPlotCurve("Altitude");
  north_speed_curve_ = new QwtPlotCurve("North Speed");
  west_speed_curve_ = new QwtPlotCurve("West Speed");
  up_speed_curve_ = new QwtPlotCurve("Up Speed");

  latitude_plot_ = new QwtPlot2();
  longitude_plot_ = new QwtPlot2();
  altitude_plot_ = new QwtPlot2();
  north_speed_plot_ = new QwtPlot2();
  west_speed_plot_ = new QwtPlot2();
  up_speed_plot_ = new QwtPlot2();

  latitude_curve_->setPen(Qt::black, kLineWidth);
  longitude_curve_->setPen(Qt::black, kLineWidth);
  altitude_curve_->setPen(Qt::black, kLineWidth);
  north_speed_curve_->setPen(Qt::black, kLineWidth);
  west_speed_curve_->setPen(Qt::black, kLineWidth);
  up_speed_curve_->setPen(Qt::black, kLineWidth);

  latitude_curve_->attach(latitude_plot_);
  longitude_curve_->attach(longitude_plot_);
  altitude_curve_->attach(altitude_plot_);
  north_speed_curve_->attach(north_speed_plot_);
  west_speed_curve_->attach(west_speed_plot_);
  up_speed_curve_->attach(up_speed_plot_);

  grid->addWidget(latitude_plot_, 0, 0);
  grid->addWidget(longitude_plot_, 1, 0);
  grid->addWidget(altitude_plot_, 2, 0);
  grid->addWidget(north_speed_plot_, 0, 1);
  grid->addWidget(west_speed_plot_, 1, 1);
  grid->addWidget(up_speed_plot_, 2, 1);
}

void GpsPlotWidget::setTimeScale(double t_start, double t_stop)
{
  latitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  longitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  altitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  north_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  west_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  up_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void GpsPlotWidget::setData(const QVector<tobas_msgs::msg::Gps>& gps_msgs)
{
  QVector<double> t_data;
  QVector<double> latitude_data;
  QVector<double> longitude_data;
  QVector<double> altitude_data;
  QVector<double> north_speed_data;
  QVector<double> west_speed_data;
  QVector<double> up_speed_data;

  for (const auto& gps : gps_msgs)
  {
    t_data.push_back(ros2::seconds(gps.header.stamp));

    latitude_data.push_back(gps.latitude);
    longitude_data.push_back(gps.longitude);
    altitude_data.push_back(gps.altitude);

    north_speed_data.push_back(gps.ground_speed.x);
    west_speed_data.push_back(gps.ground_speed.y);
    up_speed_data.push_back(gps.ground_speed.z);
  }

  latitude_curve_->setSamples(t_data, latitude_data);
  longitude_curve_->setSamples(t_data, longitude_data);
  altitude_curve_->setSamples(t_data, altitude_data);
  north_speed_curve_->setSamples(t_data, north_speed_data);
  west_speed_curve_->setSamples(t_data, west_speed_data);
  up_speed_curve_->setSamples(t_data, up_speed_data);

  latitude_plot_->replot();
  longitude_plot_->replot();
  altitude_plot_->replot();
  north_speed_plot_->replot();
  west_speed_plot_->replot();
  up_speed_plot_->replot();
}
}  // namespace log
}  // namespace gui
