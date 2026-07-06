// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/framed_label.hpp"

#include <QFrame>

namespace tobas
{
namespace qt
{
FramedLabel::FramedLabel(const QString& text, QWidget* parent) : super(text, parent)
{
  setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  setAlignment(Qt::AlignRight | Qt::AlignVCenter);
}
}  // namespace qt
}  // namespace tobas
