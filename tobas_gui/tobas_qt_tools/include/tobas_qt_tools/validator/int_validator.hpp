// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QIntValidator>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QFormLayout` =====
 * - Accurately handle `Intermediate` in `validate()`
 */
class IntValidator : public QIntValidator
{
  Q_OBJECT

  using super = QIntValidator;

public:
  using super::QIntValidator;

  State validate(QString& input, int& pos) const override;
};
}  // namespace qt
}  // namespace tobas
