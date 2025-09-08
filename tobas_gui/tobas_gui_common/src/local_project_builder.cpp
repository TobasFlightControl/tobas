#include "tobas_gui_common/local_project_builder.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_gui_common/project_paths.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
LocalProjectBuilder::LocalProjectBuilder()
{
}

bool LocalProjectBuilder::build(const fs::path& proj_path)
{
  const auto meta_pkg_path = cmn::ProjectPaths(proj_path).metaPkgPath();
  const auto ws_path = ros2::expandUser(tobas::kColconWSPathHome);
  return colcon_.build(meta_pkg_path, ws_path);
}

const std::string& LocalProjectBuilder::errorMessage() const
{
  return colcon_.errorMessage();
}
}  // namespace cmn
}  // namespace gui
