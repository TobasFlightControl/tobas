// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/version.hpp"

#include <QDebug>

#include <tobas_version/version.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace fs = std::filesystem;

namespace tobas
{
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
  return Version(version::kMajor, version::kMinor, version::kPatch);
}

void Version::setToCurrent()
{
  major = version::kMajor;
  minor = version::kMinor;
  patch = version::kPatch;
}

bool Version::isValid() const
{
  return major >= 0 && minor >= 0 && patch >= 0;
}

bool Version::isCompatible(const Version& other) const
{
  if (!isValid() || !other.isValid()) {
    qWarning() << "Invalid versions cannot be compared.";
    return false;
  }

  return major == other.major && minor == other.minor;
}

bool Version::isCompatible() const
{
  return isCompatible(Version::Current());
}

QString Version::toString() const
{
  QString res = "v%1.%2.%3";
  return res.arg(major).arg(minor).arg(patch);
}

bool Version::fromString(QString str)
{
  // Remove the prefix.
  if (str.startsWith('v', Qt::CaseInsensitive)) {
    str.remove(0, 1);
  }

  // Split into three parts.
  const auto parts = str.split('.');
  if (parts.size() != 3) {
    qWarning().noquote().nospace() << "Invalid version format: " << str
                                   << ". Expected major.minor.patch (e.g., 1.2.3).";
    return false;
  }

  // Convert to numbers.
  bool ok1 = false, ok2 = false, ok3 = false;
  major = parts[0].toInt(&ok1);
  minor = parts[1].toInt(&ok2);
  patch = parts[2].toInt(&ok3);
  if (!ok1 || !ok2 || !ok3) {
    qWarning().noquote().nospace()
      << "Invalid version number: " << str
      << ". Each component must be an integer in the form major.minor.patch (e.g., 1.2.3).";
    return false;
  }

  return true;
}

bool Version::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    qWarning() << node.error().c_str();
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
}  // namespace tobas
