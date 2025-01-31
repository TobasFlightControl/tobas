#pragma once

#include <tobas_qt_tools/widgets/qwt_plot.hpp>
#include <tobas_msgs/msg/imu_with_covariance_stamped.hpp>

#include "./forward_declaration.hpp"

namespace gui
{
namespace log
{
class ImuPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr auto kColor = Qt::black;

public:
  explicit ImuPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& imu_data);

private:
  std::array<qt::QwtPlot2*, 3> acc_plots_;
  std::array<qt::QwtPlot2*, 3> gyro_plots_;
  std::array<QwtPlotCurve*, 3> acc_curves_;
  std::array<QwtPlotCurve*, 3> gyro_curves_;
};
}  // namespace log
}  // namespace gui
