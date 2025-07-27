#include "tobas_gui_common/path.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/check.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
fs::path getProjRemotePath(const fs::path& tbs_path)
{
  return "/etc/tobas/colcon_ws/src" / tbs_path.filename();
}

std::string getProjName(const fs::path& tbs_path)
{
  TOBAS_CHECK(tbs_path.extension() == tobas::kProjectExtension);
  return tbs_path.stem();
}

std::string getProjMetaPkgName(const fs::path& tbs_path)
{
  return getProjName(tbs_path);
}

std::string getProjCfgPkgName(const fs::path& tbs_path)
{
  return getProjName(tbs_path) + "_config";
}

std::string getProjUserMsgPkgName(const fs::path& tbs_path)
{
  return getProjName(tbs_path) + "_user_msgs";
}

std::string getProjUserCppPkgName(const fs::path& tbs_path)
{
  return getProjName(tbs_path) + "_user_cpp";
}

std::string getProjUserPyPkgName(const fs::path& tbs_path)
{
  return getProjName(tbs_path) + "_user_py";
}

fs::path getProjMetaPkgPath(const fs::path& tbs_path)
{
  return tbs_path / getProjMetaPkgName(tbs_path);
}

fs::path getProjCfgPkgPath(const fs::path& tbs_path)
{
  return tbs_path / getProjCfgPkgName(tbs_path);
}

fs::path getProjUserMsgPkgPath(const fs::path& tbs_path)
{
  return tbs_path / getProjUserMsgPkgName(tbs_path);
}

fs::path getProjUserCppPkgPath(const fs::path& tbs_path)
{
  return tbs_path / getProjUserCppPkgName(tbs_path);
}

fs::path getProjUserPyPkgPath(const fs::path& tbs_path)
{
  return tbs_path / getProjUserPyPkgName(tbs_path);
}

fs::path getProjCfgConfigDirPath(const fs::path& tbs_path)
{
  return getProjCfgPkgPath(tbs_path) / "config";
}

fs::path getProjCfgLaunchDirPath(const fs::path& tbs_path)
{
  return getProjCfgPkgPath(tbs_path) / "launch";
}

fs::path getProjCfgMeshDirPath(const fs::path& tbs_path)
{
  return getProjCfgPkgPath(tbs_path) / "meshes";
}

fs::path getProjCfgUrdfDirPath(const fs::path& tbs_path)
{
  return getProjCfgPkgPath(tbs_path) / "urdf";
}

fs::path getProjOriginalUadfPath(const fs::path& tbs_path)
{
  return getProjCfgUrdfDirPath(tbs_path) / "original.uadf";
}

fs::path getProjXacroPath(const fs::path& tbs_path)
{
  return getProjCfgUrdfDirPath(tbs_path) / "drone.xacro";
}

fs::path getProjTbsDrnPath(const fs::path& tbs_path)
{
  return getProjCfgConfigDirPath(tbs_path) / "drone.tbsdrn";
}

fs::path getProjImuFiltDynParamsPath(const fs::path& tbs_path)
{
  return getProjCfgConfigDirPath(tbs_path) / kImuFilterDynamicParamFileName;
}

fs::path getProjObsvDynParamsPath(const fs::path& tbs_path)
{
  return getProjCfgConfigDirPath(tbs_path) / kObserverDynamicParamFileName;
}

fs::path getProjCtrlDynParamsPath(const fs::path& tbs_path)
{
  return getProjCfgConfigDirPath(tbs_path) / kControllerDynamicParamFileName;
}

fs::path getProjRcTeleopDynParamsPath(const fs::path& tbs_path)
{
  return getProjCfgConfigDirPath(tbs_path) / kRcTeleopDynamicParamFileName;
}

fs::path getProjBackupDirPath(const fs::path& tbs_path)
{
  return tbs_path / "backup";
}

fs::path getProjBackupSettingsPath(const fs::path& tbs_path)
{
  return getProjBackupDirPath(tbs_path) / "settings.yaml";
}
}  // namespace common
}  // namespace gui
