// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

namespace tobas
{
namespace qt
{
/**
 * @brief Label with a black frame.
 * Similar to `QLineEdit` with ReadOnly + NoFocus, but more efficient.
 */
class FramedLabel : public QLabel
{
  using super = QLabel;

public:
  explicit FramedLabel(const QString& text = "", QWidget* parent = nullptr);
};
}  // namespace qt
}  // namespace tobas
