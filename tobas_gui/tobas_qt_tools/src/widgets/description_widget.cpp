// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/description_widget.hpp"

#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
DescriptionWidget::DescriptionWidget(const QString& _text, int _point_size, QWidget* _parent) : super(_text, _parent)
{
  setFont(DefaultFont(_point_size));
  setAlignment(Qt::AlignTop);
  setWordWrap(true);
  setOpenExternalLinks(true);
}
}  // namespace qt
}  // namespace tobas
