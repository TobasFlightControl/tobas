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

public:
  explicit MRControllerFeedbackPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& msgs);

private:
  std::array<QwtPlot2*, 3> pos_ei_plots_;
  std::array<QwtPlot2*, 3> rot_ei_plots_;
  std::array<qwt::QwtPlotCurveWrapper::SharedPtr, 3> pos_ei_curves_;
  std::array<qwt::QwtPlotCurveWrapper::SharedPtr, 3> rot_ei_curves_;
};
}  // namespace log
}  // namespace gui
