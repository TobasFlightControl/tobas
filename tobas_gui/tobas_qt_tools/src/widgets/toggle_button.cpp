// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/toggle_button.hpp"

#include <QDebug>

namespace tobas
{
namespace qt
{
ToggleButton::ToggleButton(const QString& off_text, const QString& on_text, QWidget* parent)
  : super(parent), off_text_(off_text), on_text_(on_text)
{
  setText(off_text);

  connect(this, &super::clicked, this, &self::onClicked);
}

bool ToggleButton::isChecked() const
{
  return checked_;
}

void ToggleButton::setChecked(bool _checked)
{
  // Do nothing if the state does not change.
  if (_checked == checked_) {
    return;
  }

  // Update the state.
  checked_ = _checked;

  // Set text according to the new state.
  // Do not emit signals.
  if (_checked) {
    setText(on_text_);
  }
  else {
    setText(off_text_);
  }
}

void ToggleButton::onClicked()
{
  setChecked(!checked_);

  if (checked_) {
    Q_EMIT checked();
  }
  else {
    Q_EMIT unchecked();
  }
}
}  // namespace qt
}  // namespace tobas
