#include <QGridLayout>

#include <qwt/qwt_plot_curve.h>

#include "tobas_flight_log_gui/log_viewer/imu_plot.hpp"

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
    acc_curves_[i]->setPen(kColor);
    acc_curves_[i]->attach(acc_plots_[i]);
    grid->addWidget(acc_plots_[i], i, 0);

    gyro_plots_[i] = new qt::QwtPlot2();
    gyro_curves_[i]->setPen(kColor);
    gyro_curves_[i]->attach(gyro_plots_[i]);
    grid->addWidget(gyro_plots_[i], i, 1);
  }
}
}  // namespace log
}  // namespace gui
