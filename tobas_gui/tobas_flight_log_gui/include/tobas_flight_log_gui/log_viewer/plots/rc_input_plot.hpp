#pragma once

#include <tobas_msgs/msg/rc_input.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class RcInputPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit RcInputPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::RCInput>& msgs);

private:
  QwtPlot2* roll_plot_;
  QwtPlot2* pitch_plot_;
  QwtPlot2* throt_plot_;
  QwtPlot2* yaw_plot_;
  QwtPlot2* mode_plot_;
  QwtPlot2* sub_mode_plot_;
  QwtPlot2* enable_plot_;
  QwtPlot2* kill_plot_;

  qwt::QwtPlotCurveWrapper roll_curve_;
  qwt::QwtPlotCurveWrapper pitch_curve_;
  qwt::QwtPlotCurveWrapper throt_curve_;
  qwt::QwtPlotCurveWrapper yaw_curve_;
  qwt::QwtPlotCurveWrapper mode_curve_;
  qwt::QwtPlotCurveWrapper sub_mode_curve_;
  qwt::QwtPlotCurveWrapper enable_curve_;
  qwt::QwtPlotCurveWrapper kill_curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
