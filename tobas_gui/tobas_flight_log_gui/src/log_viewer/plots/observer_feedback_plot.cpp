#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/observer_feedback_plot.hpp"

namespace gui
{
namespace log
{
ObserverFeedbackPlotWidget::ObserverFeedbackPlotWidget()
{
  acc_bias_plot_ = new QwtPlot2();
  acc_bias_curves_[0] = new QwtPlotCurve("Accel Bias X");
  acc_bias_curves_[1] = new QwtPlotCurve("Accel Bias Y");
  acc_bias_curves_[2] = new QwtPlotCurve("Accel Bias Z");
  for (size_t i = 0; i < kAccelBiasSize; ++i)
  {
    acc_bias_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    acc_bias_curves_[i]->attach(acc_bias_plot_);
  }

  gyro_bias_plot_ = new QwtPlot2();
  gyro_bias_curves_[0] = new QwtPlotCurve("Gyro Bias X");
  gyro_bias_curves_[1] = new QwtPlotCurve("Gyro Bias Y");
  gyro_bias_curves_[2] = new QwtPlotCurve("Gyro Bias Z");
  for (size_t i = 0; i < kGyroBiasSize; ++i)
  {
    gyro_bias_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    gyro_bias_curves_[i]->attach(gyro_bias_plot_);
  }

  mag_hard_bias_plot_ = new QwtPlot2();
  mag_hard_bias_curves_[0] = new QwtPlotCurve("Mag Hard-Iron Bias X");
  mag_hard_bias_curves_[1] = new QwtPlotCurve("Mag Hard-Iron Bias Y");
  mag_hard_bias_curves_[2] = new QwtPlotCurve("Mag Hard-Iron Bias Z");
  for (size_t i = 0; i < kMagHardBiasSize; ++i)
  {
    mag_hard_bias_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    mag_hard_bias_curves_[i]->attach(mag_hard_bias_plot_);
  }

  mag_soft_bias_plot_ = new QwtPlot2();
  mag_soft_bias_curves_[0] = new QwtPlotCurve("Mag Soft-Iron Bias XX");
  mag_soft_bias_curves_[1] = new QwtPlotCurve("Mag Soft-Iron Bias YY");
  mag_soft_bias_curves_[2] = new QwtPlotCurve("Mag Soft-Iron Bias ZZ");
  mag_soft_bias_curves_[3] = new QwtPlotCurve("Mag Soft-Iron Bias XY");
  mag_soft_bias_curves_[4] = new QwtPlotCurve("Mag Soft-Iron Bias YZ");
  mag_soft_bias_curves_[5] = new QwtPlotCurve("Mag Soft-Iron Bias ZX");
  for (size_t i = 0; i < kMagSoftBiasSize; ++i)
  {
    mag_soft_bias_curves_[i]->setPen(kSoftBiasColor[i], kLineWidth);
    mag_soft_bias_curves_[i]->attach(mag_soft_bias_plot_);
  }

  gravity_plot_ = new QwtPlot2();
  gravity_curve_ = new QwtPlotCurve("Gravity");
  gravity_curve_->setPen(Qt::black, kLineWidth);
  gravity_curve_->attach(gravity_plot_);

  gnss_anormaly_score_plot_ = new QwtPlot2();
  gnss_anormaly_score_curve_ = new QwtPlotCurve("GNSS Anormaly Score");
  gnss_anormaly_score_curve_->setPen(Qt::black, kLineWidth);
  gnss_anormaly_score_curve_->attach(gnss_anormaly_score_plot_);

  // Layout
  const auto grid = new QGridLayout();
  setLayout(grid);
  grid->addWidget(acc_bias_plot_, 0, 0);
  grid->addWidget(gyro_bias_plot_, 1, 0);
  grid->addWidget(mag_hard_bias_plot_, 0, 1);
  grid->addWidget(mag_soft_bias_plot_, 1, 1);
  grid->addWidget(gravity_plot_, 2, 0);
  grid->addWidget(gnss_anormaly_score_plot_, 2, 1);
}

void ObserverFeedbackPlotWidget::setTimeScale(double t_start, double t_stop)
{
  acc_bias_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  gyro_bias_plot_->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  mag_hard_bias_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  mag_soft_bias_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  gravity_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
  gnss_anormaly_score_plot_->setAxisScale(QwtPlot2::xBottom, t_start, t_stop);
}

void ObserverFeedbackPlotWidget::setData(const QVector<tobas_debug_msgs::msg::ObserverFeedback>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kAccelBiasSize> acc_bias_data;
  std::array<QVector<double>, kGyroBiasSize> gyro_bias_data;
  std::array<QVector<double>, kMagHardBiasSize> mag_hard_bias_data;
  std::array<QVector<double>, kMagSoftBiasSize> mag_soft_bias_data;
  QVector<double> gravity_data;
  QVector<double> gnss_anormaly_score_data;

  for (const auto& msg : msgs)
  {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    for (size_t i = 0; i < kAccelBiasSize; ++i)
      acc_bias_data[i].push_back(msg.accel_bias.data[i]);

    for (size_t i = 0; i < kGyroBiasSize; ++i)
      gyro_bias_data[i].push_back(msg.gyro_bias.data[i]);

    for (size_t i = 0; i < kMagHardBiasSize; ++i)
      mag_hard_bias_data[i].push_back(msg.mag_hard_bias.data[i]);

    mag_soft_bias_data[0].push_back(msg.mag_soft_bias.data[0]);
    mag_soft_bias_data[1].push_back(msg.mag_soft_bias.data[4]);
    mag_soft_bias_data[2].push_back(msg.mag_soft_bias.data[8]);
    mag_soft_bias_data[3].push_back(msg.mag_soft_bias.data[1]);
    mag_soft_bias_data[4].push_back(msg.mag_soft_bias.data[5]);
    mag_soft_bias_data[5].push_back(msg.mag_soft_bias.data[2]);

    gravity_data.push_back(msg.gravity);
    gnss_anormaly_score_data.push_back(msg.gps_anormaly_score);
  }

  for (size_t i = 0; i < kAccelBiasSize; ++i)
    acc_bias_curves_[i]->setSamples(t_data, acc_bias_data[i]);

  for (size_t i = 0; i < kGyroBiasSize; ++i)
    gyro_bias_curves_[i]->setSamples(t_data, gyro_bias_data[i]);

  for (size_t i = 0; i < kMagHardBiasSize; ++i)
    mag_hard_bias_curves_[i]->setSamples(t_data, mag_hard_bias_data[i]);

  for (size_t i = 0; i < kMagSoftBiasSize; ++i)
    mag_soft_bias_curves_[i]->setSamples(t_data, mag_soft_bias_data[i]);

  gravity_curve_->setSamples(t_data, gravity_data);
  gnss_anormaly_score_curve_->setSamples(t_data, gnss_anormaly_score_data);

  acc_bias_plot_->replot();
  gyro_bias_plot_->replot();
  mag_hard_bias_plot_->replot();
  mag_soft_bias_plot_->replot();
  gravity_plot_->replot();
  gnss_anormaly_score_plot_->replot();
}
}  // namespace log
}  // namespace gui
