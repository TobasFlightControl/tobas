// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/colcon.hpp"

#include <QThread>

#include <tobas_qt_tools/thread.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace cmn
{
namespace
{
class ColconBuildThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit ColconBuildThread(colcon::Colcon& colcon, const fs::path& pkg_path, const fs::path& ws_path)
    : colcon_(colcon), pkg_path_(pkg_path), ws_path_(ws_path)
  {
  }

  void run() override
  {
    if (!colcon_.build(pkg_path_, ws_path_)) {
      Q_EMIT finished(false, QString::fromStdString(colcon_.errorMessage()));
      return;
    }

    Q_EMIT finished(true, "");
  }

private:
  colcon::Colcon& colcon_;
  const fs::path pkg_path_;
  const fs::path ws_path_;
};
}  // namespace

std::expected<void, QString> colconBuild(colcon::Colcon& colcon, const fs::path& pkg_path, const fs::path& ws_path)
{
  ColconBuildThread thread(colcon, pkg_path, ws_path);
  const auto [success, message] = qt::startThreadAndWait(thread, &ColconBuildThread::finished);

  if (success) {
    return {};
  }
  else {
    return std::unexpected(message);
  }
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "colcon.moc"
