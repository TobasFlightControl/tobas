#include "tobas_flight_log_gui/log_viewer/plots/imu_fft_plot.hpp"

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
ImuFftPlotWidget::ImuFftPlotWidget()
  : curves_{ "Accel X [m/s²]", "Accel Y [m/s²]", "Accel Z [m/s²]", "Gyro X [rad/s]", "Gyro Y [rad/s]", "Gyro Z [rad/s]" }
{
  // 1次元なので虚数部分は不要
  fft_.SetFlag(Eigen::FFT<double>::HalfSpectrum);

  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    plots_[i] = new QwtPlot2();
    if (i % 3 == 2) {
      plots_[i]->setAxisLabelUnit(QwtPlot::xBottom, "Hz");
    }
    else {
      plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    }
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    curves_[i].setPen(Qt::black, kLineWidth);
    curves_[i].attach(plots_[i]);
  }
}

void ImuFftPlotWidget::setData(const QVector<tobas_msgs::msg::Imu>& imu_msgs)
{
  const auto n = static_cast<size_t>(imu_msgs.size());

  if (n < 2) {
    return;
  }

  // データ収集
  std::array<std::vector<double>, kNumAxes> imu_data;
  for (const auto& imu : imu_msgs) {
    const auto& accel = imu.accel;
    imu_data[0].push_back(accel.x);
    imu_data[1].push_back(accel.y);
    imu_data[2].push_back(accel.z);

    const auto& gyro = imu.gyro;
    imu_data[3].push_back(gyro.x);
    imu_data[4].push_back(gyro.y);
    imu_data[5].push_back(gyro.z);
  }

  // サンプリング周波数を計算
  const auto& first_time = imu_msgs.first().header.stamp;
  const auto& last_time = imu_msgs.back().header.stamp;
  const auto duration = (last_time - first_time).seconds();  // [s]
  const auto fs = n / duration;                              // [Hz]

  // 周波数変換して表示
  for (size_t i = 0; i < kNumAxes; ++i) {
    // フーリエ変換
    std::vector<std::complex<double>> spec;
    fft_.fwd(spec, imu_data.at(i));
    assert(spec.size() == n / 2 + 1);

    // FFTの結果から周波数と振幅を計算
    // 平均値 (k = 0) は含めない
    QVector<double> freqs, amps;
    for (size_t k = 1; k < spec.size(); ++k) {
      const auto freq = k * fs / n;  // [Hz]
      freqs.push_back(freq);

      const auto is_edge = (k == 0 || (n % 2 == 0 && k == n / 2));
      const auto scale = is_edge ? 1. : 2.;
      const auto amp = scale * std::abs(spec.at(k)) / n;  // RMS
      amps.push_back(amp);
    }

    curves_[i].setSamples(freqs, amps);
    plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
