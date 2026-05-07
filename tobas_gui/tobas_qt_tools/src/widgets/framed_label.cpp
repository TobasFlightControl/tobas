// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/framed_label.hpp"

namespace tobas
{
namespace qt
{
FramedLabel::FramedLabel(const QString& text, QWidget* parent) : super(text, parent)
{
  setStyleSheet("QLabel { border: 1px solid black; background-color: white; }");
  setAlignment(Qt::AlignRight | Qt::AlignVCenter);
}
}  // namespace qt
}  // namespace tobas
