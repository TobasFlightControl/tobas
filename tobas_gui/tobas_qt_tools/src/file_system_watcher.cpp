#include "tobas_qt_tools/file_system_watcher.hpp"

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
