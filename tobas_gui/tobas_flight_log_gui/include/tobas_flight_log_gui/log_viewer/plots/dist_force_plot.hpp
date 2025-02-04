#pragma once

#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class DisturbanceForcePlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DisturbanceForcePlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_msgs);

private:
  std::array<QwtPlot2*, 3> force_plots_;
  std::array<QwtPlot2*, 3> torque_plots_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> force_curves_;
  std::array<qwt::QwtPlotCurveWrapper*, 3> torque_curves_;
};
}  // namespace log
}  // namespace gui
