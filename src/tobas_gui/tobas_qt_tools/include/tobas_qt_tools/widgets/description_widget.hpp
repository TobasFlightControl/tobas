// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

namespace tobas::qt
{
/** `QLabel` configured for placing longer text. */
class DescriptionWidget : public QLabel
{
  Q_OBJECT

  using super = QLabel;

public:
  explicit DescriptionWidget(const QString& _text, int _point_size, QWidget* _parent = nullptr);
};
}  // namespace tobas::qt
