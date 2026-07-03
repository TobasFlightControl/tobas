// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFormLayout>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QFormLayout` =====
 * - Additional methods
 */
class FormLayout : public QFormLayout
{
  Q_OBJECT

  using self = FormLayout;
  using super = QFormLayout;

public:
  using QFormLayout::QFormLayout;

  /* Add a row with the label centered on the left. */
  void addVAlignedRow(QWidget* label, QWidget* field);

  /* Add a row with the label centered on the left. */
  void addVAlignedRow(const QString& label_text, QWidget* field);

  /* Add an expanding stretch. */
  void addStretch();

  /* Delete all forms. */
  void clear();

  /* Get the label of the specified row. */
  QWidget* getLabel(int row);

  /* Get the widget of the specified row. */
  QWidget* getWidget(int row);

  /* Enable or disable the specified row. */
  void setEnabled(int row, bool enabled);

  /* Show or hide the specified row. */
  void setVisible(int row, bool visible);
};
}  // namespace qt
}  // namespace tobas
