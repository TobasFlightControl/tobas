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
 * - 初期状態を引数にもつコンストラクタ
 * - 追加メソッド
 */
class CheckBox : public QCheckBox
{
  Q_OBJECT

  using super = QCheckBox;

public:
  using super::QCheckBox;

  explicit CheckBox(const QString& text, bool checked, QWidget* parent = nullptr);

  /* 無効化したときでもテキストだけは通常表示する． */
  void setDisabledTextNormal();
};
}  // namespace qt
}  // namespace tobas
