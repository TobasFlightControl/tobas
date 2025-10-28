#include "tobas_gui_common/project_paths.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/check.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
ProjectPaths::ProjectPaths(const fs::path& proj_path) : proj_path_(proj_path)
{
}

const fs::path& ProjectPaths::getProjPath() const
{
  return proj_path_;
}

void ProjectPaths::setProjPath(const fs::path& proj_path)
{
  proj_path_ = proj_path;
}

fs::path ProjectPaths::remoteProjPath() const
{
  return "/etc/tobas/colcon_ws/src" / proj_path_.filename();
}

std::string ProjectPaths::projName() const
{
  TOBAS_CHECK(proj_path_.extension() == tobas::kProjectExtension);
  return proj_path_.stem();
}

std::string ProjectPaths::metaPkgName() const
{
  return projName();
}

std::string ProjectPaths::cfgPkgName() const
{
  return projName() + "_config";
}

std::string ProjectPaths::userMsgPkgName() const
{
  return projName() + "_user_msgs";
}

std::string ProjectPaths::userCppPkgName() const
{
  return projName() + "_user_cpp";
}

std::string ProjectPaths::userPyPkgName() const
{
  return projName() + "_user_py";
}

fs::path ProjectPaths::metaPkgPath() const
{
  return proj_path_ / metaPkgName();
}

fs::path ProjectPaths::cfgPkgPath() const
{
  return proj_path_ / cfgPkgName();
}

fs::path ProjectPaths::userMsgPkgPath() const
{
  return proj_path_ / userMsgPkgName();
}

fs::path ProjectPaths::userCppPkgPath() const
{
  return proj_path_ / userCppPkgName();
}

fs::path ProjectPaths::userPyPkgPath() const
{
  return proj_path_ / userPyPkgName();
}

fs::path ProjectPaths::cfgConfigDirPath() const
{
  return cfgPkgPath() / "config";
}

fs::path ProjectPaths::cfgLaunchDirPath() const
{
  return cfgPkgPath() / "launch";
}

fs::path ProjectPaths::cfgMeshDirPath() const
{
  return cfgPkgPath() / "meshes";
}

fs::path ProjectPaths::cfgUrdfDirPath() const
{
  return cfgPkgPath() / "urdf";
}

fs::path ProjectPaths::originalUadfPath() const
{
  return cfgUrdfDirPath() / "original.uadf";
}

fs::path ProjectPaths::xacroPath() const
{
  return cfgUrdfDirPath() / "drone.xacro";
}

fs::path ProjectPaths::tbsdrnPath() const
{
  return cfgConfigDirPath() / "drone.tbsdrn";
}

fs::path ProjectPaths::sshEndpointPath() const
{
  return cfgConfigDirPath() / "ssh_endpoint.yaml";
}

fs::path ProjectPaths::imuFiltDynParamsPath() const
{
  return cfgConfigDirPath() / kImuFilterDynamicParamFileName;
}

fs::path ProjectPaths::obsvDynParamsPath() const
{
  return cfgConfigDirPath() / kObserverDynamicParamFileName;
}

fs::path ProjectPaths::ctrlDynParamsPath() const
{
  return cfgConfigDirPath() / kControllerDynamicParamFileName;
}

fs::path ProjectPaths::rcTeleopDynParamsPath() const
{
  return cfgConfigDirPath() / kRcTeleopDynamicParamFileName;
}

fs::path ProjectPaths::projBackupDirPath() const
{
  return proj_path_ / "backup";
}

fs::path ProjectPaths::backupSettingsPath() const
{
  return projBackupDirPath() / "settings.yaml";
}

fs::path ProjectPaths::versionPath() const
{
  return proj_path_ / "version";
}
}  // namespace cmn
}  // namespace gui
