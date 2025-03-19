#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/gnss_plot.hpp"

namespace gui
{
namespace log
{
GnssPlotWidget::GnssPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  latitude_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("Latitude");
  latitude_plot_ = new QwtPlot2();
  latitude_curve_->setPen(Qt::black, kLineWidth);
  latitude_curve_->attach(latitude_plot_);
  grid->addWidget(latitude_plot_, 0, 0);

  longitude_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("Longitude");
  longitude_plot_ = new QwtPlot2();
  longitude_curve_->setPen(Qt::black, kLineWidth);
  longitude_curve_->attach(longitude_plot_);
  grid->addWidget(longitude_plot_, 1, 0);

  altitude_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("Altitude");
  altitude_plot_ = new QwtPlot2();
  altitude_curve_->setPen(Qt::black, kLineWidth);
  altitude_curve_->attach(altitude_plot_);
  grid->addWidget(altitude_plot_, 2, 0);

  north_speed_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("North Speed");
  north_speed_plot_ = new QwtPlot2();
  north_speed_curve_->setPen(Qt::black, kLineWidth);
  north_speed_curve_->attach(north_speed_plot_);
  grid->addWidget(north_speed_plot_, 0, 1);

  west_speed_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("West Speed");
  west_speed_plot_ = new QwtPlot2();
  west_speed_curve_->setPen(Qt::black, kLineWidth);
  west_speed_curve_->attach(west_speed_plot_);
  grid->addWidget(west_speed_plot_, 1, 1);

  up_speed_curve_ = std::make_shared<qwt::QwtPlotCurveWrapper>("Up Speed");
  up_speed_plot_ = new QwtPlot2();
  up_speed_curve_->setPen(Qt::black, kLineWidth);
  up_speed_curve_->attach(up_speed_plot_);
  grid->addWidget(up_speed_plot_, 2, 1);
}

void GnssPlotWidget::setTimeScale(double t_start, double t_stop)
{
  latitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  longitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  altitude_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  north_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  west_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  up_speed_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void GnssPlotWidget::setData(const QVector<tobas_msgs::msg::Gnss>& gnss_msgs)
{
  QVector<double> t_data;
  QVector<double> latitude_data;
  QVector<double> longitude_data;
  QVector<double> altitude_data;
  QVector<double> north_speed_data;
  QVector<double> west_speed_data;
  QVector<double> up_speed_data;

  for (const auto& gnss : gnss_msgs)
  {
    t_data.push_back(ros2::seconds(gnss.header.stamp));

    latitude_data.push_back(gnss.latitude);
    longitude_data.push_back(gnss.longitude);
    altitude_data.push_back(gnss.altitude);
    north_speed_data.push_back(gnss.ground_speed.x);
    west_speed_data.push_back(gnss.ground_speed.y);
    up_speed_data.push_back(gnss.ground_speed.z);
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
