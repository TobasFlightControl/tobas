// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileDialog>
#include <QSortFilterProxyModel>

namespace tobas::gui::cmn
{
class LoadProjectDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit LoadProjectDialog(QWidget* _parent, const QString& _dir);

private:
  QSortFilterProxyModel* proxy_;

private Q_SLOTS:
  void onItemActivated(const QModelIndex& _index);
};
}  // namespace tobas::gui::cmn
