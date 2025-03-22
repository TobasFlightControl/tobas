#pragma once

#include <tobas_debug_msgs/msg/observer_feedback.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class ObserverFeedbackPlotWidget : public QWidget
{
  Q_OBJECT

  static constexpr size_t kAccelBiasSize = 3;
  static constexpr size_t kGyroBiasSize = 3;
  static constexpr size_t kMagHardBiasSize = 3;
  static constexpr size_t kMagSoftBiasSize = 6;

  static constexpr Qt::GlobalColor kSoftBiasColor[] = {
    Qt::red, Qt::green, Qt::blue, Qt::magenta, Qt::yellow, Qt::black
  };

public:
  explicit ObserverFeedbackPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_debug_msgs::msg::ObserverFeedback>& msgs);

private:
  QwtPlot2* acc_bias_plot_;
  QwtPlot2* gyro_bias_plot_;
  QwtPlot2* mag_hard_bias_plot_;
  QwtPlot2* mag_soft_bias_plot_;
  QwtPlot2* gravity_plot_;
  QwtPlot2* gnss_anormaly_score_plot_;

  std::array<qwt::QwtPlotCurveWrapper, kAccelBiasSize> acc_bias_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kGyroBiasSize> gyro_bias_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kMagHardBiasSize> mag_hard_bias_curves_;
  std::array<qwt::QwtPlotCurveWrapper, kMagSoftBiasSize> mag_soft_bias_curves_;
  qwt::QwtPlotCurveWrapper gravity_curve_;
  qwt::QwtPlotCurveWrapper gnss_anormaly_score_curve_;
};
}  // namespace log
}  // namespace gui
