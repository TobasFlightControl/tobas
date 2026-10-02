// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/local_project_builder.hpp"

#include <QThread>

#include <tobas_colcon_cpp/core.hpp>
#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/thread.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace tobas
{
namespace gui
{
namespace cmn
{
namespace
{
class LocalProjectBuilderThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit LocalProjectBuilderThread(const QString& proj_path) : proj_path_(proj_path)
  {
  }

  void run() override
  {
    const auto meta_pkg_path = ProjectPaths(proj_path_).metaPkgPath();
    const auto ws_path = qt::expandUser(kColconWSPathHome);

    // Use a merged install so the workspace install directory can be added directly to the path.
    // Clear the CMake cache to avoid reusing another package with the same name from a previous build.
    const colcon::BuildOptions options{ .merge_install = true, .cmake_clean_cache = true };
    if (const auto result = colcon::build(meta_pkg_path.toStdString(), ws_path.toStdString(), options); !result) {
      Q_EMIT finished(false, QString::fromStdString(result.error()));
      return;
    }

    Q_EMIT finished(true, "");
  }

private:
  const QString proj_path_;
};
}  // namespace

std::expected<void, QString> buildLocalProject(const QString& proj_path)
{
  LocalProjectBuilderThread thread(proj_path);
  const auto [success, message] = qt::startThreadAndWait(thread, &LocalProjectBuilderThread::finished);

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

#include "local_project_builder.moc"
