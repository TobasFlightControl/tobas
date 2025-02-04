#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/twist_plot.hpp"

namespace gui
{
namespace log
{
TwistPlotWidget::TwistPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  linear_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Linear Velocity X");
  linear_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Linear Velocity Y");
  linear_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Linear Velocity Z");
  angular_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Angular Velocity X");
  angular_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Angular Velocity Y");
  angular_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Angular Velocity Z");

  for (size_t i = 0; i < 3; ++i)
  {
    linear_plots_[i] = new QwtPlot2();
    linear_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    linear_curves_[i]->attach(linear_plots_[i]);
    grid->addWidget(linear_plots_[i], i, 0);

    angular_plots_[i] = new QwtPlot2();
    angular_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    angular_curves_[i]->attach(angular_plots_[i]);
    grid->addWidget(angular_plots_[i], i, 1);
  }
}

void TwistPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
  {
    linear_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    angular_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void TwistPlotWidget::setData(const QVector<tobas_msgs::msg::Odometry>& odom_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> linear_data;
  std::array<QVector<double>, 3> angular_data;

  for (const auto& odom : odom_msgs)
  {
    t_data.push_back(ros2::seconds(odom.header.stamp));

    const auto& lin_vel = odom.twist.linear;
    linear_data[0].push_back(lin_vel.x);
    linear_data[1].push_back(lin_vel.y);
    linear_data[2].push_back(lin_vel.z);

    const auto& ang_vel = odom.twist.angular;
    angular_data[0].push_back(ang_vel.x);
    angular_data[1].push_back(ang_vel.y);
    angular_data[2].push_back(ang_vel.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    linear_curves_[i]->setSamples(t_data, linear_data[i]);
    linear_plots_[i]->replot();

    angular_curves_[i]->setSamples(t_data, angular_data[i]);
    angular_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
