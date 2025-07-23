#pragma once

#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class MRControllerFeedbackPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit MRControllerFeedbackPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& msgs);

private:
  std::array<QwtPlot2*, kNumAxes> ei_plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> ei_curves_;
};
}  // namespace log
}  // namespace gui
