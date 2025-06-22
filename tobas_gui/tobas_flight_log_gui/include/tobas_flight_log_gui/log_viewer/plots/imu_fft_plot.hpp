#pragma once

#include <tobas_msgs/msg/imu_stamped.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class ImuFftPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kNumAxes = 6;

public:
  explicit ImuFftPlotWidget();

  void setData(const QVector<tobas_msgs::msg::ImuStamped>& imu_msgs);

private:
  std::array<QwtPlot2*, kNumAxes> plots_;
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes > curves_;
};
}  // namespace log
}  // namespace gui
