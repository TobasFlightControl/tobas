#include "tobas_flight_log_gui/log_viewer/plots/twist_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
TwistPlotWidget::TwistPlotWidget()
  : cur_curves_{ "Current Linear Velocity X",  "Current Linear Velocity Y",  "Current Linear Velocity Z",
                 "Current Angular Velocity X", "Current Angular Velocity Y", "Current Angular Velocity Z" }
  , tar_curves_{ "Target Linear Velocity X",  "Target Linear Velocity Y",  "Target Linear Velocity Z",
                 "Target Angular Velocity X", "Target Angular Velocity Y", "Target Angular Velocity Z" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3);

    cur_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    cur_curves_[i].attach(plots_[i]);

    tar_curves_[i].setPen(kTargetValueColor, kLineWidth);
    tar_curves_[i].attach(plots_[i]);
  }
}

void TwistPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void TwistPlotWidget::setData(
  const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(ctrl_fb_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void TwistPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_vel = odom.twist.linear;
    val_data[0].push_back(lin_vel.x);
    val_data[1].push_back(lin_vel.y);
    val_data[2].push_back(lin_vel.z);

    const auto& ang_vel = odom.twist.angular;
    val_data[3].push_back(ang_vel.x);
    val_data[4].push_back(ang_vel.y);
    val_data[5].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }
}

void TwistPlotWidget::updateTargetSamples(
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& ctrl_fb : ctrl_fb_msgs) {
    t_data.push_back(ros2::seconds(ctrl_fb.header.stamp));

    const auto& lin_vel = ctrl_fb.target_velocity;
    val_data[0].push_back(lin_vel.x);
    val_data[1].push_back(lin_vel.y);
    val_data[2].push_back(lin_vel.z);

    const auto& ang_vel = ctrl_fb.target_gyro;
    val_data[3].push_back(ang_vel.x);
    val_data[4].push_back(ang_vel.y);
    val_data[5].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    tar_curves_[i].setSamples(t_data, val_data[i]);
  }
}
}  // namespace log
}  // namespace gui
