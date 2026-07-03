// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QObject>
#include <QString>

namespace tobas
{
namespace gui
{
namespace ctrl
{
/* QML constructor arguments. */
class SystemInfo : public QObject
{
  Q_OBJECT

  Q_PROPERTY(QString homeDirectory READ homeDirectory NOTIFY homeDirectoryChanged)

Q_SIGNALS:
  void homeDirectoryChanged();

public:
  explicit SystemInfo(QObject* parent = nullptr);

  QString modelName() const;

  QString homeDirectory() const;

private:
  QString home_dir_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
