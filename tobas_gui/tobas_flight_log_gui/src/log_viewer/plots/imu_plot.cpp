#include "tobas_flight_log_gui/log_viewer/plots/imu_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
ImuPlotWidget::ImuPlotWidget()
  : raw_curves_{ "Raw Accel X", "Raw Accel Y", "Raw Accel Z", "Raw Gyro X", "Raw Gyro Y", "Raw Gyro Z" }
  , filt_curves_{ "Filtered Accel X", "Filtered Accel Y", "Filtered Accel Z",
                  "Filtered Gyro X",  "Filtered Gyro Y",  "Filtered Gyro Z" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    raw_curves_[i].setPen(kRawValueColor, kLineWidth);
    raw_curves_[i].attach(plots_[i]);

    filt_curves_[i].setPen(kFilteredValueColor, kLineWidth);
    filt_curves_[i].attach(plots_[i]);
  }
}

void ImuPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void ImuPlotWidget::setData(
  const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs,
  const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs)
{
  updateRawSamples(raw_msgs);
  updateFilteredSamples(filt_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void ImuPlotWidget::updateRawSamples(const QVector<tobas_msgs::msg::ImuStamped>& raw_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& imu : raw_msgs) {
    t_data.push_back(ros2::seconds(imu.header.stamp));

    const auto& accel = imu.imu.accel;
    val_data[0].push_back(accel.x);
    val_data[1].push_back(accel.y);
    val_data[2].push_back(accel.z);

    const auto& gyro = imu.imu.gyro;
    val_data[3].push_back(gyro.x);
    val_data[4].push_back(gyro.y);
    val_data[5].push_back(gyro.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    raw_curves_[i].setSamples(t_data, val_data[i]);
  }
}

void ImuPlotWidget::updateFilteredSamples(const QVector<tobas_msgs::msg::ImuWithCovarianceStamped>& filt_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> val_data;

  for (const auto& imu : filt_msgs) {
    t_data.push_back(ros2::seconds(imu.header.stamp));

    const auto& accel = imu.imu.imu.accel;
    val_data[0].push_back(accel.x);
    val_data[1].push_back(accel.y);
    val_data[2].push_back(accel.z);

    const auto& gyro = imu.imu.imu.gyro;
    val_data[3].push_back(gyro.x);
    val_data[4].push_back(gyro.y);
    val_data[5].push_back(gyro.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    filt_curves_[i].setSamples(t_data, val_data[i]);
  }
}
}  // namespace log
}  // namespace gui
