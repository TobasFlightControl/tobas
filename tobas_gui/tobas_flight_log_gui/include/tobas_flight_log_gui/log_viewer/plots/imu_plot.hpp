#pragma once

#include <tobas_msgs/msg/imu_stamped.hpp>
#include <tobas_msgs/msg/imu_with_covariance_stamped.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class ImuPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 3;

public:
  explicit ImuPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(
    const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs,
    const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> acc_plots_;
  std::array<QwtPlot2*, kNumAxes> gyro_plots_;

  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> raw_acc_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> raw_gyro_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> filt_acc_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes> filt_gyro_curves_;

  void updateRawSamples(const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs);
  void updateFilteredSamples(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs);
};
}  // namespace log
}  // namespace gui
