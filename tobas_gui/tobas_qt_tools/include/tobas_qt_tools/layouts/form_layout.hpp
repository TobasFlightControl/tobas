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

public:
  using QFormLayout::QFormLayout;

  /** Add a row with the label centered on the left. */
  void addVAlignedRow(QWidget* _label, QWidget* _field);

  /** Add a row with the label centered on the left. */
  void addVAlignedRow(QWidget* _label, QLayout* _field);

  /** Add a row with the label centered on the left. */
  void addVAlignedRow(const QString& _label_text, QWidget* _field);

  /** Add a row with the label centered on the left. */
  void addVAlignedRow(const QString& _label_text, QLayout* _field);

  /** Add an expanding stretch. */
  void addStretch();

  /** Delete all forms. */
  void clear();

  /** Get the label of the specified row. */
  QWidget* getLabel(int _row);

  /** Get the widget of the specified row. */
  QWidget* getWidget(int _row);

  /** Enable or disable the specified row. */
  void setEnabled(int _row, bool _enabled);

  /** Show or hide the specified row. */
  void setVisible(int _row, bool _visible);
};
}  // namespace qt
}  // namespace tobas
