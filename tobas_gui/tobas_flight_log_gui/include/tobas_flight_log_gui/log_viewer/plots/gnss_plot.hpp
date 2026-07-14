// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include <tobas_msgs/msg/gnss.hpp>

#include "./utilities/utilities.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class GnssPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit GnssPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::Gnss>& gnss_msgs);

private:
  QwtPlot2* latitude_plot_;
  QwtPlot2* longitude_plot_;
  QwtPlot2* altitude_plot_;
  QwtPlot2* east_speed_plot_;
  QwtPlot2* north_speed_plot_;
  QwtPlot2* up_speed_plot_;

  qwt::QwtPlotCurveWrapper latitude_curve_;
  qwt::QwtPlotCurveWrapper longitude_curve_;
  qwt::QwtPlotCurveWrapper altitude_curve_;
  qwt::QwtPlotCurveWrapper east_speed_curve_;
  qwt::QwtPlotCurveWrapper north_speed_curve_;
  qwt::QwtPlotCurveWrapper up_speed_curve_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
