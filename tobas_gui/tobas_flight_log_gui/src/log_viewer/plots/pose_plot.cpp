#include <QGridLayout>

#include <tobas_kdl/rotation.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"

namespace gui
{
namespace log
{
PosePlotWidget::PosePlotWidget()
  : cur_pos_curves_{ "Current X", "Current Y", "Current Z" },
    cur_rot_curves_{ "Current Roll", "Current Pitch", "Current Yaw" },
    tar_pos_curves_{ "Target X", "Target Y", "Target Z" },
    tar_rot_curves_{ "Target Roll", "Target Pitch", "Target Yaw" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < 3; ++i) {
    pos_plots_[i] = new QwtPlot2();
    rot_plots_[i] = new QwtPlot2();

    grid->addWidget(pos_plots_[i], i, 0);
    grid->addWidget(rot_plots_[i], i, 1);

    cur_pos_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    cur_rot_curves_[i].setPen(kCurrentValueColor, kLineWidth);
    tar_pos_curves_[i].setPen(kTargetValueColor, kLineWidth);
    tar_rot_curves_[i].setPen(kTargetValueColor, kLineWidth);

    cur_pos_curves_[i].attach(pos_plots_[i]);
    cur_rot_curves_[i].attach(rot_plots_[i]);
    tar_pos_curves_[i].attach(pos_plots_[i]);
    tar_rot_curves_[i].attach(rot_plots_[i]);
  }
}

void PosePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i) {
    pos_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    rot_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PosePlotWidget::setData(
  const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(ctrl_fb_msgs);

  for (size_t i = 0; i < 3; ++i) {
    pos_plots_[i]->replot();
    rot_plots_[i]->replot();
  }
}

void PosePlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> pos_data;
  std::array<QVector<double>, 3> rot_data;

  for (const auto& odom : odom_msgs) {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& pos = odom.frame.trans;
    pos_data[0].push_back(pos.x);
    pos_data[1].push_back(pos.y);
    pos_data[2].push_back(pos.z);

    const kdl::Rotation rot(odom.frame.rot.data);
    const auto [roll, pitch, yaw] = rot.getRPY();
    rot_data[0].push_back(roll);
    rot_data[1].push_back(pitch);
    rot_data[2].push_back(yaw);
  }

  for (size_t i = 0; i < 3; ++i) {
    cur_pos_curves_[i].setSamples(t_data, pos_data[i]);
    cur_rot_curves_[i].setSamples(t_data, rot_data[i]);
  }
}

void PosePlotWidget::updateTargetSamples(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> pos_data;
  std::array<QVector<double>, 3> rot_data;

  for (const auto& ctrl_fb : ctrl_fb_msgs) {
    t_data.push_back(ros2::seconds(ctrl_fb.header.stamp));

    const auto& pos = ctrl_fb.target_position;
    pos_data[0].push_back(pos.x);
    pos_data[1].push_back(pos.y);
    pos_data[2].push_back(pos.z);

    const auto& rot = ctrl_fb.target_angle;
    rot_data[0].push_back(rot.roll);
    rot_data[1].push_back(rot.pitch);
    rot_data[2].push_back(rot.yaw);
  }

  for (size_t i = 0; i < 3; ++i) {
    tar_pos_curves_[i].setSamples(t_data, pos_data[i]);
    tar_rot_curves_[i].setSamples(t_data, rot_data[i]);
  }
}
}  // namespace log
}  // namespace gui
