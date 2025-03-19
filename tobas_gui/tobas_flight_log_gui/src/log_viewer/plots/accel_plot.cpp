#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/accel_plot.hpp"

namespace gui
{
namespace log
{
AccelPlotWidget::AccelPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  cur_lin_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Linear Accel X");
  cur_lin_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Linear Accel Y");
  cur_lin_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Linear Accel Z");
  cur_ang_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Angular Accel X");
  cur_ang_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Angular Accel Y");
  cur_ang_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Current Angular Accel Z");
  tar_lin_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Linear Accel X");
  tar_lin_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Linear Accel Y");
  tar_lin_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Linear Accel Z");
  tar_ang_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Angular Accel X");
  tar_ang_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Angular Accel Y");
  tar_ang_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Target Angular Accel Z");

  for (size_t i = 0; i < 3; ++i)
  {
    lin_plots_[i] = new QwtPlot2();
    ang_plots_[i] = new QwtPlot2();

    grid->addWidget(lin_plots_[i], i, 0);
    grid->addWidget(ang_plots_[i], i, 1);

    cur_lin_curves_[i]->setPen(kCurrentValueColor, kLineWidth);
    cur_ang_curves_[i]->setPen(kCurrentValueColor, kLineWidth);
    tar_lin_curves_[i]->setPen(kTargetValueColor, kLineWidth);
    tar_ang_curves_[i]->setPen(kTargetValueColor, kLineWidth);

    cur_lin_curves_[i]->attach(lin_plots_[i]);
    cur_ang_curves_[i]->attach(ang_plots_[i]);
    tar_lin_curves_[i]->attach(lin_plots_[i]);
    tar_ang_curves_[i]->attach(ang_plots_[i]);
  }
}

void AccelPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
  {
    lin_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    ang_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void AccelPlotWidget::setData(
  const QVector<tobas_msgs::msg::Odometry>& odom_msgs,
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  updateCurrentSamples(odom_msgs);
  updateTargetSamples(ctrl_fb_msgs);

  for (size_t i = 0; i < 3; ++i)
  {
    lin_plots_[i]->replot();
    ang_plots_[i]->replot();
  }
}

void AccelPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> lin_data;
  std::array<QVector<double>, 3> ang_data;

  for (const auto& odom : odom_msgs)
  {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_vel = odom.accel.linear;
    lin_data[0].push_back(lin_vel.x);
    lin_data[1].push_back(lin_vel.y);
    lin_data[2].push_back(lin_vel.z);

    const auto& ang_vel = odom.accel.angular;
    ang_data[0].push_back(ang_vel.x);
    ang_data[1].push_back(ang_vel.y);
    ang_data[2].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    cur_lin_curves_[i]->setSamples(t_data, lin_data[i]);
    cur_ang_curves_[i]->setSamples(t_data, ang_data[i]);
  }
}

void AccelPlotWidget::updateTargetSamples(
  const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& ctrl_fb_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> lin_data;
  std::array<QVector<double>, 3> ang_data;

  for (const auto& ctrl_fb : ctrl_fb_msgs)
  {
    t_data.push_back(ros2::seconds(ctrl_fb.header.stamp));

    const auto& lin_vel = ctrl_fb.target_accel;
    lin_data[0].push_back(lin_vel.x);
    lin_data[1].push_back(lin_vel.y);
    lin_data[2].push_back(lin_vel.z);

    const auto& ang_vel = ctrl_fb.target_dgyro;
    ang_data[0].push_back(ang_vel.x);
    ang_data[1].push_back(ang_vel.y);
    ang_data[2].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    tar_lin_curves_[i]->setSamples(t_data, lin_data[i]);
    tar_ang_curves_[i]->setSamples(t_data, ang_data[i]);
  }
}
}  // namespace log
}  // namespace gui
