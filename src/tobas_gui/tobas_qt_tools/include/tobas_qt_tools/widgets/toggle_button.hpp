// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

namespace tobas::qt
{
/** Simple toggle button whose text changes each time it is pressed. */
class ToggleButton : public QPushButton
{
  Q_OBJECT

  using self = ToggleButton;
  using super = QPushButton;

Q_SIGNALS:
  void checked();
  void unchecked();

public:
  explicit ToggleButton(const QString& _off_text, const QString& _on_text, QWidget* _parent = nullptr);

  bool isChecked() const;
  void setChecked(bool _checked);

private:
  const QString off_text_;
  const QString on_text_;

  bool checked_ = false;

private Q_SLOTS:
  void onClicked();
};
}  // namespace tobas::qt
