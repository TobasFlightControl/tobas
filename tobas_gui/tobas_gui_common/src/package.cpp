#include <tobas_std_tools/check.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_gui_common/constants.hpp"
#include "../include/tobas_gui_common/package.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
string getTBSName(const fs::path& tbs_path)
{
  TOBAS_CHECK(tbs_path.extension() == kTBSExtension);
  return tbs_path.stem();
}

string getTBSMetaName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path);
}

string getTBSConfigName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_config";
}

string getTBSUserName(const fs::path& tbs_path)
{
  return getTBSName(tbs_path) + "_user";
}

fs::path getTBSMetaPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSMetaName(tbs_path);
}

fs::path getTBSConfigPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSConfigName(tbs_path);
}

fs::path getTBSUserPath(const fs::path& tbs_path)
{
  return tbs_path / getTBSUserName(tbs_path);
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

fs::path getDynamicParamsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "dynamic_params.yaml";
}

fs::path getSettingsPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "backup" / "settings.yaml";
}

fs::path getOriginalURDFPath(const fs::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "backup" / "original.urdf";
}
}  // namespace common
}  // namespace gui
