#pragma once

#include <tobas_qt_tools/widgets/qwt_plot.hpp>

namespace gui
{
namespace log
{
class ImuPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ImuPlotWidget();

private:
  qt::QwtPlot2* acc_x_;
  qt::QwtPlot2* acc_y_;
  qt::QwtPlot2* acc_z_;
  qt::QwtPlot2* gyro_x_;
  qt::QwtPlot2* gyro_y_;
  qt::QwtPlot2* gyro_z_;
};
}  // namespace log
}  // namespace gui
