#pragma once

#include <tobas_msgs/msg/vibration_level.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class VibrationLevelPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 3;

public:
  explicit VibrationLevelPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::VibrationLevel>& msgs);

private:
  std::array<QwtPlot2*, 3> plots_;
  std::array<qwt::QwtPlotCurveWrapper, 3> curves_;
};
}  // namespace log
}  // namespace gui
