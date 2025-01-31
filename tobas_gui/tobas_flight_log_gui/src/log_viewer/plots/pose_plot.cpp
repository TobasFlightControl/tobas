#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/pose_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/constants.hpp"

namespace gui
{
namespace log
{
PosePlotWidget::PosePlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  pos_curves_[0] = new QwtPlotCurve("X");
  pos_curves_[1] = new QwtPlotCurve("Y");
  pos_curves_[2] = new QwtPlotCurve("Z");
  rpy_curves_[0] = new QwtPlotCurve("Roll");
  rpy_curves_[1] = new QwtPlotCurve("Pitch");
  rpy_curves_[2] = new QwtPlotCurve("Yaw");

  for (size_t i = 0; i < 3; ++i)
  {
    pos_plots_[i] = new QwtPlot2();
    pos_curves_[i]->setPen(kColor, kLineWidth);
    pos_curves_[i]->attach(pos_plots_[i]);
    grid->addWidget(pos_plots_[i], i, 0);

    rpy_plots_[i] = new QwtPlot2();
    rpy_curves_[i]->setPen(kColor, kLineWidth);
    rpy_curves_[i]->attach(rpy_plots_[i]);
    grid->addWidget(rpy_plots_[i], i, 1);
  }
}

void PosePlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
  {
    pos_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    rpy_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PosePlotWidget::setData(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> pos_data;
  std::array<QVector<double>, 3> rpy_data;

  for (const auto& odom : odom_msgs)
  {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& pos = odom.frame.trans;
    pos_data[0].push_back(pos.x);
    pos_data[1].push_back(pos.y);
    pos_data[2].push_back(pos.z);

    const kdl::Rotation rot(odom.frame.rot.data);
    rot.getRPY(roll_, pitch_, yaw_);
    rpy_data[0].push_back(roll_);
    rpy_data[1].push_back(pitch_);
    rpy_data[2].push_back(yaw_);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    pos_curves_[i]->setSamples(t_data, pos_data[i]);
    pos_plots_[i]->replot();

    rpy_curves_[i]->setSamples(t_data, rpy_data[i]);
    rpy_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
