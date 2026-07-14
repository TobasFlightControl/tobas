// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <limits>
#include <span>

namespace tobas
{
namespace gui
{
namespace log
{
class QwtPlot2;

class VerticalScaleRange
{
public:
  void include(double value);
  void include(const VerticalScaleRange& other);

  bool empty() const;
  double min() const;
  double max() const;
  double center() const;

private:
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
};

void setVerticalScale(QwtPlot2& plot, const VerticalScaleRange& range);
void setCenteredVerticalScale(QwtPlot2& plot, const VerticalScaleRange& range, double minimum_half_range);
void setTargetCenteredVerticalScale(
  QwtPlot2& plot,
  const VerticalScaleRange& range,
  const VerticalScaleRange& target_range,
  double minimum_half_range);
void setSharedVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range);
void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range);
}  // namespace log
}  // namespace gui
}  // namespace tobas
