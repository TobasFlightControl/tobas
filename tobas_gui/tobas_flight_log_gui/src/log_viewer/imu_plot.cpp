#include <QGridLayout>

#include "tobas_flight_log_gui/log_viewer/imu_plot.hpp"

namespace gui
{
namespace log
{
ImuPlotWidget::ImuPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  acc_x_ = new qt::QwtPlot2();
  grid->addWidget(acc_x_, 0, 0);

  acc_y_ = new qt::QwtPlot2();
  grid->addWidget(acc_y_, 1, 0);

  acc_z_ = new qt::QwtPlot2();
  grid->addWidget(acc_z_, 2, 0);

  gyro_x_ = new qt::QwtPlot2();
  grid->addWidget(gyro_x_, 0, 1);

  gyro_y_ = new qt::QwtPlot2();
  grid->addWidget(gyro_y_, 1, 1);

  gyro_z_ = new qt::QwtPlot2();
  grid->addWidget(gyro_z_, 2, 1);
}
}  // namespace log
}  // namespace gui
