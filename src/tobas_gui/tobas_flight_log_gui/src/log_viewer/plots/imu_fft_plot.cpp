// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/imu_fft_plot.hpp"

#include <vector>

#include <QGridLayout>

#include <tobas_constants/imu.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas::gui::log
{
ImuFftPlotWidget::ImuFftPlotWidget()
  : raw_curves_{ "Raw Accel X [m/s²]", "Raw Accel Y [m/s²]", "Raw Accel Z [m/s²]",
                 "Raw Gyro X [rad/s]", "Raw Gyro Y [rad/s]", "Raw Gyro Z [rad/s]" }
  , filt_curves_{ "Filtered Accel X [m/s²]", "Filtered Accel Y [m/s²]", "Filtered Accel Z [m/s²]",
                  "Filtered Gyro X [rad/s]", "Filtered Gyro Y [rad/s]", "Filtered Gyro Z [rad/s]" }
{
  const auto grid = new QGridLayout(this);

  for (size_t i = 0; i < kNumAxes; ++i) {
    // The imaginary part is unnecessary because the data is one-dimensional.
    ffts_[i].SetFlag(Eigen::FFT<double>::HalfSpectrum);

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
  const QVector<tobas_msgs::msg::Imu>& _raw_msgs,
  const QVector<tobas_msgs::msg::Imu>& _filt_msgs)
{
  // Keep these calls on the GUI thread: `updateSamples` also updates Qwt curves.
  // Each call already parallelizes its FFT axes with OpenMP; wrapping both in `std::async`
  // would create two worker teams and add thread startup and synchronization overhead.
  auto ranges = updateSamples(_raw_msgs, raw_curves_);
  const auto filt_ranges = updateSamples(_filt_msgs, filt_curves_);

  for (size_t group = 0; group < kNumGroups; ++group) {
    ranges[group].include(filt_ranges[group]);
    setSharedVerticalScale(std::span(plots_).subspan(group * kNumAxesPerGroup, kNumAxesPerGroup), ranges[group]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

ImuFftPlotWidget::ValueRanges ImuFftPlotWidget::updateSamples(
  const QVector<tobas_msgs::msg::Imu>& _msgs,
  std::array<qwt::QwtPlotCurveWrapper, kNumAxes>& _curves)
{
  const auto n = static_cast<size_t>(_msgs.size());

  if (n < 2) {
    for (auto& curve : _curves) {
      curve.clear();
    }
    return {};
  }

  // Collect data.
  std::array<std::vector<double>, kNumAxes> imu_data;
  for (auto& axis_data : imu_data) {
    axis_data.reserve(n);
  }
  for (const auto& imu : _msgs) {
    const auto& accel = imu.accel;
    const auto& gyro = imu.gyro;
    imu_data[0].push_back(accel.x);
    imu_data[1].push_back(accel.y);
    imu_data[2].push_back(accel.z);
    imu_data[3].push_back(gyro.x);
    imu_data[4].push_back(gyro.y);
    imu_data[5].push_back(gyro.z);
  }

  // All axes use the same frequency bins. Exclude the mean value (k = 0).
  const auto num_bins = n / 2;
  QVector<double> freqs;
  freqs.reserve(num_bins);
  for (size_t k = 1; k <= num_bins; ++k) {
    freqs.push_back(static_cast<double>(kImuSamplingRate) * k / n);
  }

  // Compute each axis's amplitude spectrum and value range in parallel.
  std::array<QVector<double>, kNumAxes> amplitudes;
  std::array<VerticalScaleRange, kNumAxes> axis_ranges;
#pragma omp parallel for num_threads(kNumAxes)
  for (size_t i = 0; i < kNumAxes; ++i) {
    std::vector<Eigen::dcomplex> spec;
    ffts_[i].fwd(spec, imu_data[i]);
    TOBAS_CHECK(spec.size() == num_bins + 1);

    auto& amps = amplitudes[i];
    amps.reserve(num_bins);
    for (size_t k = 1; k < spec.size(); ++k) {
      const auto is_nyquist = n % 2 == 0 && k == num_bins;
      const auto scale = is_nyquist ? 1.0 : 2.0;
      const auto amp = scale * std::abs(spec.at(k)) / n;
      amps.push_back(amp);
      axis_ranges[i].include(amp);
    }
  }

  // Update curves on the GUI thread after all FFT workers finish.
  // Merge per-axis ranges serially to avoid concurrent writes to the shared group ranges.
  ValueRanges ranges;
  for (size_t i = 0; i < kNumAxes; ++i) {
    _curves[i].setSamples(freqs, amplitudes[i]);
    ranges[i / kNumAxesPerGroup].include(axis_ranges[i]);
  }

  return ranges;
}
}  // namespace tobas::gui::log
