#include "tobas_flight_log_gui/log_viewer/plots/imu_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
ImuPlotWidget::ImuPlotWidget()
  : raw_acc_curves_{ "Raw Accel X", "Raw Accel Y", "Raw Accel Z" }
  , raw_gyro_curves_{ "Raw Gyro X", "Raw Gyro Y", "Raw Gyro Z" }
  , filt_acc_curves_{ "Filtered Accel X", "Filtered Accel Y", "Filtered Accel Z" }
  , filt_gyro_curves_{ "Filtered Gyro X", "Filtered Gyro Y", "Filtered Gyro Z" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    acc_plots_[i] = new QwtPlot2();
    grid->addWidget(acc_plots_[i], i, 0);

    gyro_plots_[i] = new QwtPlot2();
    grid->addWidget(gyro_plots_[i], i, 1);

    raw_acc_curves_[i].setPen(kRawValueColor, kLineWidth);
    raw_acc_curves_[i].attach(acc_plots_[i]);

    raw_gyro_curves_[i].setPen(kRawValueColor, kLineWidth);
    raw_gyro_curves_[i].attach(gyro_plots_[i]);

    filt_acc_curves_[i].setPen(kFilteredValueColor, kLineWidth);
    filt_acc_curves_[i].attach(acc_plots_[i]);

    filt_gyro_curves_[i].setPen(kFilteredValueColor, kLineWidth);
    filt_gyro_curves_[i].attach(gyro_plots_[i]);
  }
}

void ImuPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    acc_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    gyro_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void ImuPlotWidget::setData(
  const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs,
  const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs)
{
  updateRawSamples(raw_msgs);
  updateFilteredSamples(filt_msgs);

  for (size_t i = 0; i < kNumAxes; ++i) {
    acc_plots_[i]->replot();
    gyro_plots_[i]->replot();
  }
}

void ImuPlotWidget::updateRawSamples(const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> acc_data;
  std::array<QVector<double>, kNumAxes> gyro_data;

  for (const auto& imu : raw_msgs) {
    t_data.push_back(ros2::seconds(imu.header.stamp));

    const auto& accel = imu.imu.accel;
    acc_data[0].push_back(accel.x);
    acc_data[1].push_back(accel.y);
    acc_data[2].push_back(accel.z);

    const auto& gyro = imu.imu.gyro;
    gyro_data[0].push_back(gyro.x);
    gyro_data[1].push_back(gyro.y);
    gyro_data[2].push_back(gyro.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    raw_acc_curves_[i].setSamples(t_data, acc_data[i]);
    raw_gyro_curves_[i].setSamples(t_data, gyro_data[i]);
  }
}

void ImuPlotWidget::updateFilteredSamples(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> acc_data;
  std::array<QVector<double>, kNumAxes> gyro_data;

  for (const auto& imu : filt_msgs) {
    t_data.push_back(ros2::seconds(imu.header.stamp));

    const auto& accel = imu.imu.imu.accel;
    acc_data[0].push_back(accel.x);
    acc_data[1].push_back(accel.y);
    acc_data[2].push_back(accel.z);

    const auto& gyro = imu.imu.imu.gyro;
    gyro_data[0].push_back(gyro.x);
    gyro_data[1].push_back(gyro.y);
    gyro_data[2].push_back(gyro.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    filt_acc_curves_[i].setSamples(t_data, acc_data[i]);
    filt_gyro_curves_[i].setSamples(t_data, gyro_data[i]);
  }
}
}  // namespace log
}  // namespace gui
