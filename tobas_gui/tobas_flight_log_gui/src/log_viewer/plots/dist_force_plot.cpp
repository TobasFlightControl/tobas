#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/dist_force_plot.hpp"

namespace gui
{
namespace log
{
DisturbanceForcePlotWidget::DisturbanceForcePlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  force_curves_[0] = new QwtPlotCurve("Force X");
  force_curves_[1] = new QwtPlotCurve("Force Y");
  force_curves_[2] = new QwtPlotCurve("Force Z");
  torque_curves_[0] = new QwtPlotCurve("Torque X");
  torque_curves_[1] = new QwtPlotCurve("Torque Y");
  torque_curves_[2] = new QwtPlotCurve("Torque Z");

  for (size_t i = 0; i < 3; ++i)
  {
    force_plots_[i] = new QwtPlot2();
    force_curves_[i]->setPen(kDefaultColor, kLineWidth);
    force_curves_[i]->attach(force_plots_[i]);
    grid->addWidget(force_plots_[i], i, 0);

    torque_plots_[i] = new QwtPlot2();
    torque_curves_[i]->setPen(kDefaultColor, kLineWidth);
    torque_curves_[i]->attach(torque_plots_[i]);
    grid->addWidget(torque_plots_[i], i, 1);
  }
}

void DisturbanceForcePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
  {
    force_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    torque_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void DisturbanceForcePlotWidget::setData(const QVector<tobas_kdl_msgs::msg::WrenchStamped>& dist_force_msg)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> force_data;
  std::array<QVector<double>, 3> torque_data;

  for (const auto& dist_force : dist_force_msg)
  {
    t_data.push_back(ros2::seconds(dist_force.header.stamp));

    const auto& force = dist_force.wrench.force;
    force_data[0].push_back(force.x);
    force_data[1].push_back(force.y);
    force_data[2].push_back(force.z);

    const auto& torque = dist_force.wrench.torque;
    torque_data[0].push_back(torque.x);
    torque_data[1].push_back(torque.y);
    torque_data[2].push_back(torque.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    force_curves_[i]->setSamples(t_data, force_data[i]);
    force_plots_[i]->replot();

    torque_curves_[i]->setSamples(t_data, torque_data[i]);
    torque_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
