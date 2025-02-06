#pragma once

#include <tobas_msgs/msg/gnss.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class GnssPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit GnssPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Gnss>& gnss_msgs);

private:
  QwtPlot2* latitude_plot_;
  QwtPlot2* longitude_plot_;
  QwtPlot2* altitude_plot_;
  QwtPlot2* north_speed_plot_;
  QwtPlot2* west_speed_plot_;
  QwtPlot2* up_speed_plot_;

  qwt::QwtPlotCurveWrapper::SharedPtr latitude_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr longitude_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr altitude_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr north_speed_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr west_speed_curve_;
  qwt::QwtPlotCurveWrapper::SharedPtr up_speed_curve_;
};
}  // namespace log
}  // namespace gui
