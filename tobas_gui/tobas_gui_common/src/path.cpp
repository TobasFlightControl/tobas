#include "tobas_gui_common/path.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/check.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
fs::path getProjRemotePath(const fs::path& proj_path)
{
  return "/etc/tobas/colcon_ws/src" / proj_path.filename();
}

std::string getProjName(const fs::path& proj_path)
{
  TOBAS_CHECK(proj_path.extension() == tobas::kProjectExtension);
  return proj_path.stem();
}

std::string getProjMetaPkgName(const fs::path& proj_path)
{
  return getProjName(proj_path);
}

std::string getProjCfgPkgName(const fs::path& proj_path)
{
  return getProjName(proj_path) + "_config";
}

std::string getProjUserMsgPkgName(const fs::path& proj_path)
{
  return getProjName(proj_path) + "_user_msgs";
}

std::string getProjUserCppPkgName(const fs::path& proj_path)
{
  return getProjName(proj_path) + "_user_cpp";
}

std::string getProjUserPyPkgName(const fs::path& proj_path)
{
  return getProjName(proj_path) + "_user_py";
}

fs::path getProjMetaPkgPath(const fs::path& proj_path)
{
  return proj_path / getProjMetaPkgName(proj_path);
}

fs::path getProjCfgPkgPath(const fs::path& proj_path)
{
  return proj_path / getProjCfgPkgName(proj_path);
}

fs::path getProjUserMsgPkgPath(const fs::path& proj_path)
{
  return proj_path / getProjUserMsgPkgName(proj_path);
}

fs::path getProjUserCppPkgPath(const fs::path& proj_path)
{
  return proj_path / getProjUserCppPkgName(proj_path);
}

fs::path getProjUserPyPkgPath(const fs::path& proj_path)
{
  return proj_path / getProjUserPyPkgName(proj_path);
}

fs::path getProjCfgConfigDirPath(const fs::path& proj_path)
{
  return getProjCfgPkgPath(proj_path) / "config";
}

fs::path getProjCfgLaunchDirPath(const fs::path& proj_path)
{
  return getProjCfgPkgPath(proj_path) / "launch";
}

fs::path getProjCfgMeshDirPath(const fs::path& proj_path)
{
  return getProjCfgPkgPath(proj_path) / "meshes";
}

fs::path getProjCfgUrdfDirPath(const fs::path& proj_path)
{
  return getProjCfgPkgPath(proj_path) / "urdf";
}

fs::path getProjOriginalUadfPath(const fs::path& proj_path)
{
  return getProjCfgUrdfDirPath(proj_path) / "original.uadf";
}

fs::path getProjXacroPath(const fs::path& proj_path)
{
  return getProjCfgUrdfDirPath(proj_path) / "drone.xacro";
}

fs::path getProjTbsDrnPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / "drone.tbsdrn";
}

fs::path getProjSshEndpointPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / "ssh_endpoint.yaml";
}

fs::path getProjImuFiltDynParamsPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / kImuFilterDynamicParamFileName;
}

fs::path getProjObsvDynParamsPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / kObserverDynamicParamFileName;
}

fs::path getProjCtrlDynParamsPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / kControllerDynamicParamFileName;
}

fs::path getProjRcTeleopDynParamsPath(const fs::path& proj_path)
{
  return getProjCfgConfigDirPath(proj_path) / kRcTeleopDynamicParamFileName;
}

fs::path getProjBackupDirPath(const fs::path& proj_path)
{
  return proj_path / "backup";
}

fs::path getProjBackupSettingsPath(const fs::path& proj_path)
{
  return getProjBackupDirPath(proj_path) / "settings.yaml";
}
}  // namespace common
}  // namespace gui
