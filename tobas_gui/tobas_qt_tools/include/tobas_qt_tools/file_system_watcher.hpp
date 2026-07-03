// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileSystemWatcher>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QFileSystemWatcher` =====
 * - Additional methods
 */
class FileSystemWatcher : public QFileSystemWatcher
{
  Q_OBJECT

  using super = QFileSystemWatcher;

public:
  using super::QFileSystemWatcher;

  /* Delete all watched paths. */
  void clear();
};
}  // namespace qt
}  // namespace tobas
