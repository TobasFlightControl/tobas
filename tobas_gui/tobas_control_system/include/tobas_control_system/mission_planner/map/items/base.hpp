// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QAbstractListModel>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace map
{
template <typename... Args>
class MapItemModel : public QAbstractListModel
{
public:
  int rowCount(const QModelIndex& _parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& _index, int _role = Qt::DisplayRole) const override;
  QHash<int, QByteArray> roleNames() const override;

  void add(const Args&... _args);
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
QVariant MapItemModel<Args...>::data(const QModelIndex& _index, int _role) const
{
  return list_[_index.row()][_role - Qt::UserRole];
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
void MapItemModel<Args...>::add(const Args&... _args)
{
  beginInsertRows(QModelIndex(), rowCount(), rowCount());
  list_.append({ QVariant::fromValue(_args)... });
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
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
