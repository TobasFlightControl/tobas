#pragma once

#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>
#include <tobas_msgs/msg/odometry.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class PosePlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PosePlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(
    const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
    const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);

private:
  std::array<QwtPlot2*, 3> pos_plots_;
  std::array<QwtPlot2*, 3> rot_plots_;

  std::array<qwt::QwtPlotCurveWrapper, 3> cur_pos_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> cur_rot_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> tar_pos_curves_;
  std::array<qwt::QwtPlotCurveWrapper, 3> tar_rot_curves_;

  void updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs);
  void updateTargetSamples(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs);
};
}  // namespace log
}  // namespace gui
