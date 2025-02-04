#pragma once

#include <tobas_msgs/msg/odometry.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class TwistPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit TwistPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);

private:
  std::array<QwtPlot2*, 3> linear_plots_;
  std::array<QwtPlot2*, 3> angular_plots_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> linear_curves_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> angular_curves_;
};
}  // namespace log
}  // namespace gui
