// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/load_project_dialog.hpp"

#include <QAbstractItemView>
#include <QDebug>
#include <QFileSystemModel>

#include "tobas_gui_common/constants.hpp"

namespace tobas::gui::cmn
{
LoadProjectDialog::LoadProjectDialog(QWidget* _parent, const QString& _dir)
  : QFileDialog(_parent, "Select Tobas Project (*.TBS)", _dir, "Tobas Project (*.TBS)")
{
  setOptions(ShowDirsOnly | DontUseNativeDialog);  // Use the Qt dialog for custom settings.
  setFileMode(Directory);                          // Mode for selecting an existing directory.
  setFilter(QDir::AllDirs | QDir::Hidden | QDir::NoDotAndDotDot);

  proxy_ = new QSortFilterProxyModel(this);
  setProxyModel(proxy_);

  for (const auto& view_name : QStringList{ "treeView", "listView" }) {
    // Get the internal view of the dialog.
    const auto view = findChild<QAbstractItemView*>(view_name);
    if (!view) {
      qWarning() << "Dialog view model" << view_name << "not found.";
      continue;
    }

    // Confirm when double-clicked or activated.
    connect(view, &QAbstractItemView::activated, this, &LoadProjectDialog::onItemActivated);
  }
}

void LoadProjectDialog::onItemActivated(const QModelIndex& _index)
{
  const auto path = proxy_->data(_index, QFileSystemModel::FilePathRole).toString();
  if (path.endsWith(kProjectExtension)) {
    accept();
  }
}
}  // namespace tobas::gui::cmn
