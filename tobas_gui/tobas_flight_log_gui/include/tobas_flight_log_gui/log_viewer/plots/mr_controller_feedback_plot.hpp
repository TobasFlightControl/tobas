#pragma once

#include <tobas_debug_msgs/msg/multicopter_controller_feedback.hpp>

#include "./common.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class MRControllerFeedbackPlotWidget : public BasePlotWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit MRControllerFeedbackPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& msgs);

private:
  std::array<QwtPlot2*, kNumAxes> ei_plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> ei_curves_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
