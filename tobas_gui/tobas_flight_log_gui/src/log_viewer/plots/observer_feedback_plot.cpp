#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/observer_feedback_plot.hpp"

namespace gui
{
namespace log
{
ObserverFeedbackPlotWidget::ObserverFeedbackPlotWidget()
  : acc_bias_curves_{ "Accel Bias X", "Accel Bias Y", "Accel Bias Z" }
  , gyro_bias_curves_{ "Gyro Bias X", "Gyro Bias Y", "Gyro Bias Z" }
  , mag_hard_bias_curves_{ "Mag Hard-Iron Bias X", "Mag Hard-Iron Bias Y", "Mag Hard-Iron Bias Z" }
  , mag_soft_bias_curves_{ "Mag Soft-Iron Bias XX", "Mag Soft-Iron Bias YY", "Mag Soft-Iron Bias ZZ",
                           "Mag Soft-Iron Bias XY", "Mag Soft-Iron Bias YZ", "Mag Soft-Iron Bias ZX" }
  , gravity_curve_("Gravity")
  , gnss_anomaly_score_curve_("GNSS Anomaly Score")

{
  acc_bias_plot_ = new QwtPlot2();
  for (size_t i = 0; i < kAccelBiasSize; ++i) {
    acc_bias_curves_[i].setPen(kColorXYZ[i], kLineWidth);
    acc_bias_curves_[i].attach(acc_bias_plot_);
  }

  gyro_bias_plot_ = new QwtPlot2();
  for (size_t i = 0; i < kGyroBiasSize; ++i) {
    gyro_bias_curves_[i].setPen(kColorXYZ[i], kLineWidth);
    gyro_bias_curves_[i].attach(gyro_bias_plot_);
  }

  mag_hard_bias_plot_ = new QwtPlot2();
  for (size_t i = 0; i < kMagHardBiasSize; ++i) {
    mag_hard_bias_curves_[i].setPen(kColorXYZ[i], kLineWidth);
    mag_hard_bias_curves_[i].attach(mag_hard_bias_plot_);
  }

  mag_soft_bias_plot_ = new QwtPlot2();
  for (size_t i = 0; i < kMagSoftBiasSize; ++i) {
    mag_soft_bias_curves_[i].setPen(kSoftBiasColor[i], kLineWidth);
    mag_soft_bias_curves_[i].attach(mag_soft_bias_plot_);
  }

  gravity_plot_ = new QwtPlot2();
  gravity_curve_.setPen(Qt::black, kLineWidth);
  gravity_curve_.attach(gravity_plot_);

  gnss_anomaly_score_plot_ = new QwtPlot2();
  gnss_anomaly_score_curve_.setPen(Qt::black, kLineWidth);
  gnss_anomaly_score_curve_.attach(gnss_anomaly_score_plot_);

  // Layout
  const auto grid = new QGridLayout();
  setLayout(grid);
  grid->addWidget(acc_bias_plot_, 0, 0);
  grid->addWidget(gyro_bias_plot_, 1, 0);
  grid->addWidget(mag_hard_bias_plot_, 0, 1);
  grid->addWidget(mag_soft_bias_plot_, 1, 1);
  grid->addWidget(gravity_plot_, 2, 0);
  grid->addWidget(gnss_anomaly_score_plot_, 2, 1);
}

void ObserverFeedbackPlotWidget::setTimeScale(double t_start, double t_stop)
{
  acc_bias_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  gyro_bias_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  mag_hard_bias_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  mag_soft_bias_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  gravity_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  gnss_anomaly_score_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
}

void ObserverFeedbackPlotWidget::setData(const QVector<tobas_debug_msgs::msg::ObserverFeedback>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kAccelBiasSize> acc_bias_data;
  std::array<QVector<double>, kGyroBiasSize> gyro_bias_data;
  std::array<QVector<double>, kMagHardBiasSize> mag_hard_bias_data;
  std::array<QVector<double>, kMagSoftBiasSize> mag_soft_bias_data;
  QVector<double> gravity_data;
  QVector<double> gnss_anomaly_score_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    for (size_t i = 0; i < kAccelBiasSize; ++i) {
      acc_bias_data[i].push_back(msg.accel_bias.data[i]);
    }

    for (size_t i = 0; i < kGyroBiasSize; ++i) {
      gyro_bias_data[i].push_back(msg.gyro_bias.data[i]);
    }

    for (size_t i = 0; i < kMagHardBiasSize; ++i) {
      mag_hard_bias_data[i].push_back(msg.mag_hard_bias.data[i]);
    }

    mag_soft_bias_data[0].push_back(msg.mag_soft_bias.data[0]);
    mag_soft_bias_data[1].push_back(msg.mag_soft_bias.data[4]);
    mag_soft_bias_data[2].push_back(msg.mag_soft_bias.data[8]);
    mag_soft_bias_data[3].push_back(msg.mag_soft_bias.data[1]);
    mag_soft_bias_data[4].push_back(msg.mag_soft_bias.data[5]);
    mag_soft_bias_data[5].push_back(msg.mag_soft_bias.data[2]);

    gravity_data.push_back(msg.gravity);
    gnss_anomaly_score_data.push_back(msg.gnss_anomaly_score);
  }

  for (size_t i = 0; i < kAccelBiasSize; ++i) {
    acc_bias_curves_[i].setSamples(t_data, acc_bias_data[i]);
  }

  for (size_t i = 0; i < kGyroBiasSize; ++i) {
    gyro_bias_curves_[i].setSamples(t_data, gyro_bias_data[i]);
  }

  for (size_t i = 0; i < kMagHardBiasSize; ++i) {
    mag_hard_bias_curves_[i].setSamples(t_data, mag_hard_bias_data[i]);
  }

  for (size_t i = 0; i < kMagSoftBiasSize; ++i) {
    mag_soft_bias_curves_[i].setSamples(t_data, mag_soft_bias_data[i]);
  }

  gravity_curve_.setSamples(t_data, gravity_data);
  gnss_anomaly_score_curve_.setSamples(t_data, gnss_anomaly_score_data);

  acc_bias_plot_->replot();
  gyro_bias_plot_->replot();
  mag_hard_bias_plot_->replot();
  mag_soft_bias_plot_->replot();
  gravity_plot_->replot();
  gnss_anomaly_score_plot_->replot();
}
}  // namespace log
}  // namespace gui
