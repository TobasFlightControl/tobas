#pragma once

#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>
#include <tobas_msgs/msg/odometry.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class TwistPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 3;

public:
  explicit TwistPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(
    const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
    const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> lin_plots_;
  std::array<QwtPlot2*, kNumAxes> ang_plots_;

  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> cur_lin_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> cur_ang_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> tar_lin_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> tar_ang_curves_;

  void updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);
  void updateTargetSamples(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);
};
}  // namespace log
}  // namespace gui
