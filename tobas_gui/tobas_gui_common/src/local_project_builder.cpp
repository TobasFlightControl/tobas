#include "tobas_gui_common/local_project_builder.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
LocalProjectBuilder::LocalProjectBuilder()
{
  // ワークスペースの install ディレクトリをそのままパスに追加するために --merge-install が必要
  colcon_.setMergeInstall(true);
}

bool LocalProjectBuilder::build(const fs::path& proj_path)
{
  const auto meta_pkg_path = ProjectPaths(proj_path).metaPkgPath();
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);

  return colcon_.build(meta_pkg_path, ws_path);
}

const std::string& LocalProjectBuilder::errorMessage() const
{
  return colcon_.errorMessage();
}

LocalProjectBuilderThread::LocalProjectBuilderThread(const std::filesystem::path& proj_path) : proj_path_(proj_path)
{
}

void LocalProjectBuilderThread::run()
{
  if (!builder_.build(proj_path_)) {
    Q_EMIT finished(false, QString::fromStdString(builder_.errorMessage()));
    return;
  }

  Q_EMIT finished(true, "");
}

std::expected<void, QString> buildLocalProject(const std::filesystem::path& proj_path)
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
