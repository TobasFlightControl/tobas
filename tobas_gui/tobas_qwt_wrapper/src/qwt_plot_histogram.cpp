// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qwt_wrapper/qwt_plot_histogram.hpp"

#include <qwt/qwt_plot_histogram.h>  // <= C++17

namespace tobas::qwt
{
QwtPlotHistogramWrapper::QwtPlotHistogramWrapper(const QString& _title)
{
  impl_ = std::make_shared<QwtPlotHistogram>(_title);
}

void QwtPlotHistogramWrapper::setPen(const QColor& _color, qreal _width, Qt::PenStyle _style)
{
  impl_->setPen(_color, _width, _style);
}

void QwtPlotHistogramWrapper::setBrush(const QColor& _color)
{
  impl_->setBrush(_color);
}

void QwtPlotHistogramWrapper::setSamples(
  const QVector<double>& _values,
  double _min_value,
  double _max_value,
  double _bin_width)
{
  assert(_min_value <= _max_value);
  assert(_bin_width > 0.0);

  QVector<QwtIntervalSample> samples;

  const auto bin_count = static_cast<int>(std::ceil((_max_value - _min_value) / _bin_width));
  QVector<int> counts(bin_count, 0);

  for (const auto& value : _values) {
    if (!std::isfinite(value)) {
      continue;
    }
    if (value < _min_value || _max_value < value) {
      continue;
    }

    auto index = static_cast<int>((value - _min_value) / _bin_width);

    // Only `value == max_value` falls outside the range, so put it in the last bin.
    if (index >= bin_count) {
      index = bin_count - 1;
    }

    ++counts[index];
  }

  for (int i = 0; i < bin_count; ++i) {
    const double left = _min_value + i * _bin_width;
    const double right = left + _bin_width;

    samples.append(QwtIntervalSample(static_cast<double>(counts[i]), left, right));
  }

  impl_->setSamples(samples);
}

void QwtPlotHistogramWrapper::attach(QwtPlot* _plot)
{
  impl_->attach(_plot);
}

void QwtPlotHistogramWrapper::clear()
{
  QVector<QwtIntervalSample> empty;
  impl_->setSamples(empty);
}
}  // namespace tobas::qwt
