#pragma once

#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>
#include <tobas_msgs/msg/odometry.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class AccelPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit AccelPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(
    const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
    const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;

  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> cur_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> tar_curves_;

  void updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);
  void updateTargetSamples(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);
};
}  // namespace log
}  // namespace gui
