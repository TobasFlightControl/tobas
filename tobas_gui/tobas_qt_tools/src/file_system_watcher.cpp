// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/file_system_watcher.hpp"

namespace tobas
{
namespace qt
{
void FileSystemWatcher::clear()
{
  for (const auto& p : directories()) {
    removePath(p);
  }

  for (const auto& p : files()) {
    removePath(p);
  }
}
}  // namespace qt
}  // namespace tobas
