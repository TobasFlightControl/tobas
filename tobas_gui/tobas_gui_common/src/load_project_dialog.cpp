#include "tobas_gui_common/load_project_dialog.hpp"

#include <QAbstractItemView>
#include <QFileSystemModel>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace cmn
{
LoadProjectDialog::LoadProjectDialog(QWidget* parent, const QString& dir) : QFileDialog(parent)
{
  setWindowTitle("Select Tobas Project (*.TBS)");
  setNameFilter("Tobas Project (*.TBS)");
  setDirectory(dir);
  setFileMode(Directory);
  setOptions(DontUseNativeDialog | ShowDirsOnly);

  proxy_ = new QSortFilterProxyModel(this);
  setProxyModel(proxy_);

  // ダイアログ内部のビューを取得
  const auto view = findChild<QAbstractItemView*>("treeView");
  if (!view) {
    throw std::runtime_error("Dialog tree view model not found.");
  }

  // ダブルクリック (activated) されたら確定させる
  connect(view, &QAbstractItemView::activated, this, &LoadProjectDialog::onItemActivated);
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
