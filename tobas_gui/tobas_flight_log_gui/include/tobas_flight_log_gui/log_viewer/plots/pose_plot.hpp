#pragma once

#include <tobas_msgs/msg/odometry.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class PosePlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PosePlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);

private:
  std::array<QwtPlot2*, 3> pos_plots_;
  std::array<QwtPlot2*, 3> rpy_plots_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> pos_curves_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> rpy_curves_;

  double roll_, pitch_, yaw_;
};
}  // namespace log
}  // namespace gui
