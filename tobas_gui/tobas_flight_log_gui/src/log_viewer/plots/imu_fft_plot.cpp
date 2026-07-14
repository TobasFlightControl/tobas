// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/imu_fft_plot.hpp"

#include <future>

#include <QGridLayout>

#include <tobas_constants/imu.hpp>
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
    // The imaginary part is unnecessary because the data is one-dimensional.
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
  // Run in parallel.
  auto f1 =
    std::async(std::launch::async, [this, raw_msgs] { return updateSamples(raw_msgs, raw_ffts_, raw_curves_); });
  auto f2 =
    std::async(std::launch::async, [this, filt_msgs] { return updateSamples(filt_msgs, filt_ffts_, filt_curves_); });
  auto ranges = f1.get();
  const auto filt_ranges = f2.get();

  for (size_t group = 0; group < kNumGroups; ++group) {
    ranges[group].include(filt_ranges[group]);
    setSharedVerticalScale(std::span(plots_).subspan(group * kNumAxesPerGroup, kNumAxesPerGroup), ranges[group]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

ImuFftPlotWidget::ValueRanges ImuFftPlotWidget::updateSamples(
  const QVector<tobas_msgs::msg::Imu>& msgs,
  std::array<Eigen::FFT<double>, kNumAxes>& ffts,
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& curves)
{
  const auto n = static_cast<size_t>(msgs.size());

  if (n < 2) {
    for (auto& curve : curves) {
      curve.clear();
    }
    return {};
  }

  // Collect data.
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

  // Transform to the frequency domain and display.
  // Run each axis in parallel because FFT is expensive (N log(N)).
  std::array<VerticalScaleRange, kNumAxes> axis_ranges;
#pragma omp parallel for num_threads(kNumAxes)
  for (size_t i = 0; i < kNumAxes; ++i) {
    // Fourier transform.
    std::vector<std::complex<double>> spec;
    ffts[i].fwd(spec, imu_data.at(i));
    assert(spec.size() == n / 2 + 1);

    // Calculate frequency and amplitude from the FFT result.
    // Exclude the mean value (k = 0).
    QVector<double> freqs, amps;
    for (size_t k = 1; k < spec.size(); ++k) {
      const auto freq = static_cast<double>(kImuSamplingRate) * k / n;  // [Hz]
      freqs.push_back(freq);

      const auto is_edge = (n % 2 == 0 && k == n / 2);
      const auto scale = is_edge ? 1.0 : 2.0;
      const auto amp = scale * std::abs(spec.at(k)) / n;  // RMS
      amps.push_back(amp);
      axis_ranges[i].include(amp);
    }

    curves[i].setSamples(freqs, amps);
  }

  ValueRanges ranges;
  for (size_t i = 0; i < kNumAxes; ++i) {
    ranges[i / kNumAxesPerGroup].include(axis_ranges[i]);
  }
  return ranges;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
