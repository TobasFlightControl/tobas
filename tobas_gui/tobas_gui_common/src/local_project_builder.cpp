// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/local_project_builder.hpp"

#include <QThread>

#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

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
  explicit LocalProjectBuilderThread(const fs::path& proj_path) : proj_path_(proj_path)
  {
  }

  void run() override
  {
    if (!builder_.build(proj_path_)) {
      Q_EMIT finished(false, QString::fromStdString(builder_.errorMessage()));
      return;
    }

    Q_EMIT finished(true, "");
  }

private:
  const fs::path proj_path_;

  LocalProjectBuilder builder_;
};
}  // namespace

LocalProjectBuilder::LocalProjectBuilder()
{
  // `--merge-install` is required to add the workspace install directory directly to the path.
  colcon_.setMergeInstall(true);

  // If a cache for another package with the same name as the target package remains under `/build`,
  // `colcon` reuses that package regardless of the current directory.
  // Therefore, clear the cache to ensure only the target package is built.
  colcon_.setCmakeCleanCache(true);
}

bool LocalProjectBuilder::build(const fs::path& proj_path)
{
  const auto meta_pkg_path = ProjectPaths(proj_path).metaPkgPath();
  const auto ws_path = ros2::expandUser(kColconWSPathHome);

  return colcon_.build(meta_pkg_path, ws_path);
}

const std::string& LocalProjectBuilder::errorMessage() const
{
  return colcon_.errorMessage();
}

std::expected<void, QString> buildLocalProject(const fs::path& proj_path)
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
