#pragma once

#include <QAbstractListModel>

namespace gui
{
namespace gcs
{
namespace map
{
template <typename... Args>
class MapItemModel : public QAbstractListModel
{
public:
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  QHash<int, QByteArray> roleNames() const override;

  void add(const Args&... args);
  void clear();

  virtual QString modelName() const = 0;
  virtual QByteArrayList argNames() const = 0;

private:
  QVector<QVector<QVariant>> list_;
};

template <typename... Args>
int MapItemModel<Args...>::rowCount(const QModelIndex&) const
{
  return list_.size();
}

template <typename... Args>
QVariant MapItemModel<Args...>::data(const QModelIndex& index, int role) const
{
  return list_[index.row()][role - Qt::UserRole];
}

template <typename... Args>
QHash<int, QByteArray> MapItemModel<Args...>::roleNames() const
{
  QHash<int, QByteArray> res;
  const auto arg_names = argNames();
  for (int i = 0; i < arg_names.size(); ++i) {
    res[Qt::UserRole + i] = arg_names[i];
  }
  return res;
}

template <typename... Args>
void MapItemModel<Args...>::add(const Args&... args)
{
  beginInsertRows(QModelIndex(), rowCount(), rowCount());
  list_.append({ QVariant::fromValue(args)... });
  endInsertRows();
}

template <typename... Args>
void MapItemModel<Args...>::clear()
{
  beginRemoveRows(QModelIndex(), 0, rowCount() - 1);
  list_.clear();
  endRemoveRows();
}
}  // namespace map
}  // namespace gcs
}  // namespace gui
