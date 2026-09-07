// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qwt_wrapper/qwt_plot_histogram.hpp"

#include <qwt/qwt_plot_histogram.h>  // <= C++17

namespace tobas
{
namespace qwt
{
QwtPlotHistogramWrapper::QwtPlotHistogramWrapper(const QString& title)
{
  impl_ = std::make_shared<QwtPlotHistogram>(title);
}

void QwtPlotHistogramWrapper::setPen(const QColor& color, qreal width, Qt::PenStyle style)
{
  impl_->setPen(color, width, style);
}

void QwtPlotHistogramWrapper::setBrush(const QColor& color)
{
  impl_->setBrush(color);
}

void QwtPlotHistogramWrapper::setSamples(
  const QVector<double>& values,
  double min_value,
  double max_value,
  double bin_width)
{
  assert(min_value <= max_value);
  assert(bin_width > 0.0);

  QVector<QwtIntervalSample> samples;

  const auto bin_count = static_cast<int>(std::ceil((max_value - min_value) / bin_width));
  QVector<int> counts(bin_count, 0);

  for (const auto& value : values) {
    if (!std::isfinite(value)) {
      continue;
    }
    if (value < min_value || max_value < value) {
      continue;
    }

    auto index = static_cast<int>((value - min_value) / bin_width);

    // Only `value == max_value` falls outside the range, so put it in the last bin.
    if (index >= bin_count) {
      index = bin_count - 1;
    }

    ++counts[index];
  }

  for (int i = 0; i < bin_count; ++i) {
    const double left = min_value + i * bin_width;
    const double right = left + bin_width;

    samples.append(QwtIntervalSample(static_cast<double>(counts[i]), left, right));
  }

  impl_->setSamples(samples);
}

void QwtPlotHistogramWrapper::attach(QwtPlot* plot)
{
  impl_->attach(plot);
}

void QwtPlotHistogramWrapper::clear()
{
  QVector<QwtIntervalSample> empty;
  impl_->setSamples(empty);
}
}  // namespace qwt
}  // namespace tobas
