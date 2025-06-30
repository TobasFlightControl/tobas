#pragma once

#include <QFileDialog>
#include <QSortFilterProxyModel>

namespace gui
{
namespace common
{
class TbsProjectDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit TbsProjectDialog(QWidget* parent, const QString& dir);

private:
  QSortFilterProxyModel* proxy_;

private Q_SLOTS:
  void onItemActivated(const QModelIndex& index);
};
}  // namespace common
}  // namespace gui
