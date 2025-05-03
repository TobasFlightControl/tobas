#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/twist_plot.hpp"

namespace gui
{
namespace log
{
TwistPlotWidget::TwistPlotWidget()
  : cur_lin_curves_{ "Current Linear Velocity X", "Current Linear Velocity Y", "Current Linear Velocity Z" }
  , cur_ang_curves_{ "Current Angular Velocity X", "Current Angular Velocity Y", "Current Angular Velocity Z" }
  , tar_lin_curves_{ "Target Linear Velocity X", "Target Linear Velocity Y", "Target Linear Velocity Z" }
  , tar_ang_curves_{ "Target Angular Velocity X", "Target Angular Velocity Y", "Target Angular Velocity Z" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < 3; ++i) {
    lin_plots_[i] = new QwtPlot2();
    ang_plots_[i] = new QwtPlot2();

    grid->addWidget(lin_plots_[i], i, 0);
    grid->addWidget(ang_plots_[i], i, 1);

    cur_lin_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    cur_ang_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    tar_lin_curves_[i].setPen(kTargetValueColor, kLineWidth);
    tar_ang_curves_[i].setPen(kTargetValueColor, kLineWidth);

    cur_lin_curves_[i].attach(lin_plots_[i]);
    cur_ang_curves_[i].attach(ang_plots_[i]);
    tar_lin_curves_[i].attach(lin_plots_[i]);
    tar_ang_curves_[i].attach(ang_plots_[i]);
  }
}

void TwistPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i) {
    lin_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    ang_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void TwistPlotWidget::setData(
  const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(ctrl_fb_msgs);

  for (size_t i = 0; i < 3; ++i) {
    lin_plots_[i]->replot();
    ang_plots_[i]->replot();
  }
}

void TwistPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> lin_data;
  std::array<QVector<double>, 3> ang_data;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_vel = odom.twist.linear;
    lin_data[0].push_back(lin_vel.x);
    lin_data[1].push_back(lin_vel.y);
    lin_data[2].push_back(lin_vel.z);

    const auto& ang_vel = odom.twist.angular;
    ang_data[0].push_back(ang_vel.x);
    ang_data[1].push_back(ang_vel.y);
    ang_data[2].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < 3; ++i) {
    cur_lin_curves_[i].setSamples(t_data, lin_data[i]);
    cur_ang_curves_[i].setSamples(t_data, ang_data[i]);
  }
}

void TwistPlotWidget::updateTargetSamples(
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> lin_data;
  std::array<QVector<double>, 3> ang_data;

  for (const auto& ctrl_fb : ctrl_fb_msgs) {
    t_data.push_back(ros2::seconds(ctrl_fb.header.stamp));

    const auto& lin_vel = ctrl_fb.target_velocity;
    lin_data[0].push_back(lin_vel.x);
    lin_data[1].push_back(lin_vel.y);
    lin_data[2].push_back(lin_vel.z);

    const auto& ang_vel = ctrl_fb.target_gyro;
    ang_data[0].push_back(ang_vel.x);
    ang_data[1].push_back(ang_vel.y);
    ang_data[2].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < 3; ++i) {
    tar_lin_curves_[i].setSamples(t_data, lin_data[i]);
    tar_ang_curves_[i].setSamples(t_data, ang_data[i]);
  }
}
}  // namespace log
}  // namespace gui
