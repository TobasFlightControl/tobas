#include "tobas_gui_common/load_project_dialog.hpp"

#include <QAbstractItemView>
#include <QFileSystemModel>

#include <tobas_constants/constants.hpp>

namespace gui
{
namespace cmn
{
LoadProjectDialog::LoadProjectDialog(QWidget* parent, const QString& dir)
  : QFileDialog(parent, "Select Tobas Project (*.TBS)", dir, "Tobas Project (*.TBS)")
{
  setFileMode(Directory);
  setOptions(ShowDirsOnly | DontUseNativeDialog);
  setFilter(QDir::AllEntries | QDir::Hidden | QDir::NoDotAndDotDot);

  proxy_ = new QSortFilterProxyModel(this);
  setProxyModel(proxy_);

  for (const auto& view_name : QStringList{ "treeView", "listView" }) {
    // ダイアログ内部のビューを取得
    const auto view = findChild<QAbstractItemView*>(view_name);
    if (!view) {
      throw std::runtime_error("Dialog view model \"" + view_name.toStdString() + "\" not found.");
    }

    // ダブルクリック (activated) されたら確定させる
    connect(view, &QAbstractItemView::activated, this, &LoadProjectDialog::onItemActivated);
  }
}

void LoadProjectDialog::onItemActivated(const QModelIndex& index)
{
  const auto path = proxy_->data(index, QFileSystemModel::FilePathRole).toString();
  if (path.endsWith(tobas::kProjectExtension)) {
    accept();
  }
}
}  // namespace cmn
}  // namespace gui
