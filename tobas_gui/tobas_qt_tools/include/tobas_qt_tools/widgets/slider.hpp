// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSlider>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QSlider` =====
 * - Disable mouse wheel events
 */
class Slider : public QSlider
{
  Q_OBJECT

  using self = Slider;
  using super = QSlider;

public:
  using QSlider::QSlider;

protected:
  void wheelEvent(QWheelEvent* event) override;
};
}  // namespace qt
}  // namespace tobas
