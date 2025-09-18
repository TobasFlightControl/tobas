#pragma once

#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class DisturbanceForcePlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit DisturbanceForcePlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> curves_;
};
}  // namespace log
}  // namespace gui
