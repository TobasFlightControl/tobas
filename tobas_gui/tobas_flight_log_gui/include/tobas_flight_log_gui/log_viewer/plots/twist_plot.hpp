#pragma once

#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>

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
  void setData(
    const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
    const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);

private:
  std::array<QwtPlot2*, 3> lin_plots_;
  std::array<QwtPlot2*, 3> ang_plots_;

  std::array<qwt::QwtPlotCurveWrapper, 3> cur_lin_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> cur_ang_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> tar_lin_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> tar_ang_curves_;

  void updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);
  void updateTargetSamples(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);
};
}  // namespace log
}  // namespace gui
