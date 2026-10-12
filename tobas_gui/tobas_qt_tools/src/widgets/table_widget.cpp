// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/table_widget.hpp"

#include <QHeaderView>

namespace tobas
{
namespace qt
{
void TableWidget::removeAll()
{
  while (rowCount() > 0) {
    removeRow(0);
  }
}

void TableWidget::setColumnsWidth(int _width)
{
  for (int col = 0; col < columnCount(); ++col) {
    setColumnWidth(col, _width);
  }
}

void TableWidget::setHeaderSectionsClickable(bool _clickable)
{
  horizontalHeader()->setSectionsClickable(_clickable);
  verticalHeader()->setSectionsClickable(_clickable);
}
}  // namespace qt
}  // namespace tobas
