// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/load_project_dialog.hpp"

#include <QAbstractItemView>
#include <QFileSystemModel>

#include "tobas_gui_common/constants.hpp"

namespace tobas
{
namespace gui
{
namespace cmn
{
LoadProjectDialog::LoadProjectDialog(QWidget* parent, const QString& dir)
  : QFileDialog(parent, "Select Tobas Project (*.TBS)", dir, "Tobas Project (*.TBS)")
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

void LoadProjectDialog::onItemActivated(const QModelIndex& index)
{
  const auto path = proxy_->data(index, QFileSystemModel::FilePathRole).toString();
  if (path.endsWith(kProjectExtension)) {
    accept();
  }
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
