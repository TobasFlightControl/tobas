// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <qwt/qwt_scale_engine.h>

#include <tobas_qwt_wrapper/qwt_plot_curve.hpp>

#include "./base_plot.hpp"
#include "./constants.hpp"
#include "./qwt_plot.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class VerticalScaleRange
{
public:
  void include(double value)
  {
    if (std::isfinite(value)) {
      min_ = std::min(min_, value);
      max_ = std::max(max_, value);
    }
  }

  void include(const VerticalScaleRange& other)
  {
    if (!other.empty()) {
      include(other.min_);
      include(other.max_);
    }
  }

  [[nodiscard]] bool empty() const
  {
    return max_ < min_;
  }

  [[nodiscard]] double min() const
  {
    return min_;
  }

  [[nodiscard]] double max() const
  {
    return max_;
  }

private:
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
};

template <typename PlotIterator>
void setSharedVerticalScale(PlotIterator first, PlotIterator last, const VerticalScaleRange& range)
{
  double scale_min = range.empty() ? -1.0 : range.min();
  double scale_max = range.empty() ? 1.0 : range.max();
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale_min, scale_max, scale_step);

  for (auto it = first; it != last; ++it) {
    (*it)->setAxisScale(QwtPlot::yLeft, scale_min, scale_max, scale_step);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
