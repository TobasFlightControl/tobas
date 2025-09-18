#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"

#include <QGridLayout>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace log
{
PosePlotWidget::PosePlotWidget()
  : cur_curves_{ "Current X [m]",      "Current Y [m]",       "Current Z [m]",
                 "Current Roll [deg]", "Current Pitch [deg]", "Current Yaw [deg]" }
  , tar_curves_{ "Target X [m]",      "Target Y [m]",       "Target Z [m]",
                 "Target Roll [deg]", "Target Pitch [deg]", "Target Yaw [deg]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    cur_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    cur_curves_[i].attach(plots_[i]);

    tar_curves_[i].setPen(kTargetValueColor, kLineWidth);
    tar_curves_[i].attach(plots_[i]);
  }
}

void PosePlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].clear();
    tar_curves_[i].clear();
    plots_[i]->replot();
  }
}

void PosePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PosePlotWidget::setData(
  const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
  const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& ctrl_fb_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(ctrl_fb_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void PosePlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& pos = odom.frame.trans;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const kdl::Rotation rot(odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();
    val_data[3].push_back(tobas_std::rad2deg(roll));
    val_data[4].push_back(tobas_std::rad2deg(pitch));
    val_data[5].push_back(tobas_std::rad2deg(yaw));
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    cur_curves_[i].setSamples(t_data, val_data[i]);
  }
}

void PosePlotWidget::updateTargetSamples(
  const QVector<tobas_debug_msgs::msg::MulticopterControllerFeedback>& ctrl_fb_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& ctrl_fb : ctrl_fb_msgs) {
    t_data.push_back(ros2::seconds(ctrl_fb.header.stamp));

    const auto& pos = ctrl_fb.target_position;
    val_data[0].push_back(pos.x);
    val_data[1].push_back(pos.y);
    val_data[2].push_back(pos.z);

    const auto& rot = ctrl_fb.target_angle;
    val_data[3].push_back(tobas_std::rad2deg(rot.roll));
    val_data[4].push_back(tobas_std::rad2deg(rot.pitch));
    val_data[5].push_back(tobas_std::rad2deg(rot.yaw));
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    tar_curves_[i].setSamples(t_data, val_data[i]);
  }
}
}  // namespace log
}  // namespace gui
