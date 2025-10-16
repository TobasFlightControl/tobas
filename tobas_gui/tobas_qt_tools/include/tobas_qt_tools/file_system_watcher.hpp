#pragma once

#include <QFileSystemWatcher>

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
