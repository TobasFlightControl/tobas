// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPointF>
#include <QWidget>

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_kdl/vector.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class LevelIndicatorWidget : public QWidget
{
  using super = QWidget;

public:
  explicit LevelIndicatorWidget(QWidget* parent = nullptr);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

  void clear();
  void setRawAccel(const kdl::Vector& acc, double dt);
  void setCalibratedAccel(const kdl::Vector& acc, double dt);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  dsp::LowPassFilter<kdl::Vector> acc_raw_lpf_, acc_calib_lpf_;
  bool has_raw_data_ = false;
  bool has_calib_data_ = false;
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
