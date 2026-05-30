// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/imu_fft_plot.hpp"

#include <future>

#include <QGridLayout>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
ImuFftPlotWidget::ImuFftPlotWidget()
  : raw_curves_{ "Raw Accel X [m/s²]", "Raw Accel Y [m/s²]", "Raw Accel Z [m/s²]",
                 "Raw Gyro X [rad/s]", "Raw Gyro Y [rad/s]", "Raw Gyro Z [rad/s]" }
  , filt_curves_{ "Filtered Accel X [m/s²]", "Filtered Accel Y [m/s²]", "Filtered Accel Z [m/s²]",
                  "Filtered Gyro X [rad/s]", "Filtered Gyro Y [rad/s]", "Filtered Gyro Z [rad/s]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    // 1次元なので虚数部分は不要
    raw_ffts_[i].SetFlag(Eigen::FFT<double>::HalfSpectrum);
    filt_ffts_[i].SetFlag(Eigen::FFT<double>::HalfSpectrum);

    plots_[i] = new QwtPlot2();
    if (i % 3 == 2) {
      plots_[i]->setAxisLabelUnit(QwtPlot::xBottom, "Hz");
    }
    else {
      plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    }
    grid->addWidget(plots_[i], i % 3, i / 3, 1, 1);

    raw_curves_[i].setPen(kRawValueColor, kLineWidth);
    raw_curves_[i].attach(plots_[i]);

    filt_curves_[i].setPen(kFilteredValueColor, kLineWidth);
    filt_curves_[i].attach(plots_[i]);
  }
}

void ImuFftPlotWidget::clear()
{
  for (size_t i = 0; i < kNumAxes; ++i) {
    raw_curves_[i].clear();
    filt_curves_[i].clear();
    plots_[i]->replot();
  }
}

void ImuFftPlotWidget::setTimeScale(double, double)
{
}

void ImuFftPlotWidget::setData(
  const QVector<tobas_msgs::msg::Imu>& raw_msgs,
  const QVector<tobas_msgs::msg::Imu>& filt_msgs)
{
  // 並列実行
  auto f1 = std::async(std::launch::async, [this, raw_msgs] { updateSamples(raw_msgs, raw_ffts_, raw_curves_); });
  auto f2 = std::async(std::launch::async, [this, filt_msgs] { updateSamples(filt_msgs, filt_ffts_, filt_curves_); });
  f1.wait();
  f2.wait();

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void ImuFftPlotWidget::updateSamples(
  const QVector<tobas_msgs::msg::Imu>& msgs,
  std::array<Eigen::FFT<double>, kNumAxes>& ffts,
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& curves)
{
  const auto n = static_cast<size_t>(msgs.size());

  if (n < 2) {
    return;
  }

  // データ収集
  std::array<std::vector<double>, kNumAxes> imu_data;
  for (const auto& imu : msgs) {
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
  const auto& first_time = msgs.first().header.stamp;
  const auto& last_time = msgs.back().header.stamp;
  const auto duration = (last_time - first_time).seconds();  // [s]

  // 周波数変換して表示
  // FFTが重い (N log(N)) ため各軸に対して並列実行
#pragma omp parallel for num_threads(kNumAxes)
  for (size_t i = 0; i < kNumAxes; ++i) {
    // フーリエ変換
    std::vector<std::complex<double>> spec;
    ffts[i].fwd(spec, imu_data.at(i));
    assert(spec.size() == n / 2 + 1);

    // FFTの結果から周波数と振幅を計算
    // 平均値 (k = 0) は含めない
    QVector<double> freqs, amps;
    for (size_t k = 1; k < spec.size(); ++k) {
      const auto freq = k / duration;  // [Hz]
      freqs.push_back(freq);

      const auto is_edge = (n % 2 == 0 && k == n / 2);
      const auto scale = is_edge ? 1. : 2.;
      const auto amp = scale * std::abs(spec.at(k)) / n;  // RMS
      amps.push_back(amp);
    }

    curves[i].setSamples(freqs, amps);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
