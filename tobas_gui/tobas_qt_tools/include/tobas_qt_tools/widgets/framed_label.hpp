// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

namespace tobas
{
namespace qt
{
/**
 * @brief 黒枠付きのラベル．
 * QLineEditをReadOnly + NoFocusにしたものに近いが，こちらの方が効率的．
 */
class FramedLabel : public QLabel
{
  using super = QLabel;

public:
  explicit FramedLabel(const QString& text = "", QWidget* parent = nullptr);
};
}  // namespace qt
}  // namespace tobas
