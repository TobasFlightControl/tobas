#include <tobas_std_tools/check.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_tools/package.hpp"

using namespace std;

namespace tobas
{
string getTBSName(const filesystem::path& tbs_path)
{
  TOBAS_CHECK(tbs_path.extension() == tobas::kTBSExtension);
  return tbs_path.filename().stem();
}

string getTBSMetaName(const filesystem::path& tbs_path)
{
  return getTBSName(tbs_path);
}

string getTBSConfigName(const filesystem::path& tbs_path)
{
  return getTBSName(tbs_path) + "_config";
}

string getTBSUserName(const filesystem::path& tbs_path)
{
  return getTBSName(tbs_path) + "_user";
}

filesystem::path getTBSMetaPath(const filesystem::path& tbs_path)
{
  return tbs_path / getTBSMetaName(tbs_path);
}

filesystem::path getTBSConfigPath(const filesystem::path& tbs_path)
{
  return tbs_path / getTBSConfigName(tbs_path);
}

filesystem::path getTBSUserPath(const filesystem::path& tbs_path)
{
  return tbs_path / getTBSUserName(tbs_path);
}

filesystem::path getTBSDRNPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "drone.tbsdrn";
}

filesystem::path getModifiedURDFPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "urdf" / "drone.xacro";
}

filesystem::path getMeshPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "mesh";
}

filesystem::path getDynamicParamsPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "config" / "dynamic_params.yaml";
}

filesystem::path getSettingsPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "backup" / "settings.yaml";
}

filesystem::path getOriginalURDFPath(const filesystem::path& tbs_path)
{
  return getTBSConfigPath(tbs_path) / "backup" / "original.urdf";
}
}  // namespace tobas
