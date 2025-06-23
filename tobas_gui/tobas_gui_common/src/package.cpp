#include "tobas_gui_common/package.hpp"

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/check.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
fs::path getRemoteTBSPath(const fs::path& tbs_path)
{
  return "/etc/tobas/colcon_ws/src" / tbs_path.filename();
}

std::string getTBSName(const fs::path& tbs_path)
{
  TOBAS_CHECK(tbs_path.extension() == tobas::kTBSExtension);
  return tbs_path.stem();
}

std::string getTBSMetaName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path);
}

std::string getTBSConfigName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_config";
}

std::string getTBSUserMsgName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_user_msgs";
}

std::string getTBSUserCppName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_user_cpp";
}

std::string getTBSUserPyName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_user_py";
}

fs::path getBackupPath(const fs::path& tbs_path)
{
  return tbs_path / "backup";
}

fs::path getTBSMetaPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSMetaName(tbs_path);
}

fs::path getTBSConfigPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSConfigName(tbs_path);
}

fs::path getTBSUserMsgPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSUserMsgName(tbs_path);
}

fs::path getTBSUserCppPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSUserCppName(tbs_path);
}

fs::path getTBSUserPyPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSUserPyName(tbs_path);
}

fs::path getTBSDRNPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "drone.tbsdrn";
}

fs::path getModifiedURDFPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "urdf" / "drone.xacro";
}

fs::path getMeshPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "mesh";
}

fs::path getControllerDynamicParamsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "controller_dynamic.yaml";
}

fs::path getObserverDynamicParamsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "observer_dynamic.yaml";
}

fs::path getRcTeleopDynamicParamsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "rc_teleop_dynamic.yaml";
}

fs::path getImuPreprocessDynamicParamsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "imu_preprocess_dynamic.yaml";
}

fs::path getSettingsPath(const fs::path& tbs_path)
{
  return getBackupPath(tbs_path) / "settings.yaml";
}

fs::path getOriginalURDFPath(const fs::path& tbs_path)
{
  return getBackupPath(tbs_path) / "original.urdf";
}
}  // namespace common
}  // namespace gui
