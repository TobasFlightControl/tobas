#pragma once

#include <tobas_msgs/msg/magnetic_field_with_covariance_stamped.hpp>

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
  void setData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& mag_msgs);

private:
  std::array<QwtPlot2*, 3> mag_plots_;
  std::array<qwt::QwtPlotCurveWrapper::SharedPtr, 3> mag_curves_;
};
}  // namespace log
}  // namespace gui
