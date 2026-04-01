// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileSystemWatcher>

namespace tobas
{
namespace qt
{
/**
 * ===== QFileSystemWatcher との違い =====
 * - 追加メソッド
 */
class FileSystemWatcher : public QFileSystemWatcher
{
  Q_OBJECT

  using super = QFileSystemWatcher;

public:
  using super::QFileSystemWatcher;

  /* 全ての監視パスを削除する． */
  void clear();
};
}  // namespace qt
}  // namespace tobas
