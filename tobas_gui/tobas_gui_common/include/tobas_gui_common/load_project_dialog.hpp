#pragma once

#include <QFileDialog>
#include <QSortFilterProxyModel>

namespace gui
{
namespace cmn
{
class LoadProjectDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit LoadProjectDialog(QWidget* parent, const QString& dir);

private:
  QSortFilterProxyModel* proxy_;

private Q_SLOTS:
  void onItemActivated(const QModelIndex& index);
};
}  // namespace cmn
}  // namespace gui
