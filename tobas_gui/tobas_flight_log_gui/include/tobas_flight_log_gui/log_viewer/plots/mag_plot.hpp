#pragma once

#include <tobas_msgs/msg/magnetic_field.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class MagPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MagPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::MagneticField>& mag_msgs);

private:
  std::array<QwtPlot2*, 3> mag_plots_;
  std::array<qwt::QwtPlotCurveWrapper, 3> mag_curves_;
};
}  // namespace log
}  // namespace gui
