// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QCheckBox>

namespace tobas
{
namespace qt
{
/**
 * ===== QCheckBox との違い =====
 * - 追加メソッド
 */
class CheckBox : public QCheckBox
{
  Q_OBJECT

  using super = QCheckBox;

public:
  using super::QCheckBox;

  /* 無効化したときでもテキストだけは通常表示する． */
  void setDisabledTextNormal();
};
}  // namespace qt
}  // namespace tobas
