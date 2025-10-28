#include "tobas_gui_common/version.hpp"

#include <QDebug>

#include <tobas_constants/version.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
Version::Version()
{
}

Version::Version(int _major, int _minor, int _patch) : major(_major), minor(_minor), patch(_patch)
{
}

Version Version::Current()
{
  return Version(tobas::version::kMajor, tobas::version::kMinor, tobas::version::kPatch);
}

void Version::setToCurrent()
{
  major = tobas::version::kMajor;
  minor = tobas::version::kMinor;
  patch = tobas::version::kPatch;
}

bool Version::isCompatible() const
{
  if (major < 0 || minor < 0 || patch < 0) {
    qWarning() << "Version not initialized.";
    return false;
  }

  return major == tobas::version::kMajor && minor == tobas::version::kMinor;
}

QString Version::toString() const
{
  QString res = "v%1.%2.%3";
  return res.arg(major).arg(minor).arg(patch);
}

bool Version::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    qWarning() << QString::fromStdString(node.error());
    return false;
  }

  if (!yaml::load(kMajorKey, node.value(), major)) {
    return false;
  }
  if (!yaml::load(kMinorKey, node.value(), minor)) {
    return false;
  }
  if (!yaml::load(kPatchKey, node.value(), patch)) {
    return false;
  }

  return true;
}

bool Version::save(const fs::path& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMajorKey] = major;
  node[kMinorKey] = minor;
  node[kPatchKey] = patch;

  if (!yaml::save(path, node)) {
    return false;
  }

  return true;
}
}  // namespace cmn
}  // namespace gui
