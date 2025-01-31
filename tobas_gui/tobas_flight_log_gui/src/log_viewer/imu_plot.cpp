#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/imu_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/constants.hpp"

namespace gui
{
namespace log
{
ImuPlotWidget::ImuPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  acc_curves_[0] = new QwtPlotCurve("Accel X");
  acc_curves_[1] = new QwtPlotCurve("Accel Y");
  acc_curves_[2] = new QwtPlotCurve("Accel Z");
  gyro_curves_[0] = new QwtPlotCurve("Gyro X");
  gyro_curves_[1] = new QwtPlotCurve("Gyro Y");
  gyro_curves_[2] = new QwtPlotCurve("Gyro Z");

  for (size_t i = 0; i < 3; ++i)
  {
    acc_plots_[i] = new qt::QwtPlot2();
    acc_curves_[i]->setPen(kColor, kLineWidth);
    acc_curves_[i]->attach(acc_plots_[i]);
    grid->addWidget(acc_plots_[i], i, 0);

    gyro_plots_[i] = new qt::QwtPlot2();
    gyro_curves_[i]->setPen(kColor, kLineWidth);
    gyro_curves_[i]->attach(gyro_plots_[i]);
    grid->addWidget(gyro_plots_[i], i, 1);
  }
}

void ImuPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
  {
    acc_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    gyro_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void ImuPlotWidget::setData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& imu_data)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> acc_data;
  std::array<QVector<double>, 3> gyro_data;

  for (const auto& imu : imu_data)
  {
    t_data.push_back(ros2::seconds(imu.header.stamp));

    const auto& accel = imu.imu.imu.accel;
    acc_data[0].push_back(accel.x);
    acc_data[1].push_back(accel.y);
    acc_data[2].push_back(accel.z);

    const auto& gyro = imu.imu.imu.gyro;
    gyro_data[0].push_back(gyro.x);
    gyro_data[1].push_back(gyro.y);
    gyro_data[2].push_back(gyro.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    acc_curves_[i]->setSamples(t_data, acc_data[i]);
    acc_plots_[i]->replot();

    gyro_curves_[i]->setSamples(t_data, gyro_data[i]);
    gyro_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
