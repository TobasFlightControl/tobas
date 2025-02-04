#pragma once

#include <tobas_msgs/msg/gps.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class GpsPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit GpsPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Gps>& gps_msgs);

private:
  QwtPlot2* latitude_plot_;
  QwtPlot2* longitude_plot_;
  QwtPlot2* altitude_plot_;
  QwtPlot2* north_speed_plot_;
  QwtPlot2* west_speed_plot_;
  QwtPlot2* up_speed_plot_;

  qwt::QwtPlotCurveWrapper* latitude_curve_;
  qwt::QwtPlotCurveWrapper* longitude_curve_;
  qwt::QwtPlotCurveWrapper* altitude_curve_;
  qwt::QwtPlotCurveWrapper* north_speed_curve_;
  qwt::QwtPlotCurveWrapper* west_speed_curve_;
  qwt::QwtPlotCurveWrapper* up_speed_curve_;
};
}  // namespace log
}  // namespace gui
