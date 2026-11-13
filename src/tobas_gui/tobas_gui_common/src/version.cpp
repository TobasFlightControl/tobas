// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/version.hpp"

#include <QDebug>

#include <tobas_version/version.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace tobas::gui::cmn
{
namespace
{
constexpr char kMajorKey[] = "major";
constexpr char kMinorKey[] = "minor";
constexpr char kPatchKey[] = "patch";
}  // namespace

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

bool Version::isValid() const
{
  return major >= 0 && minor >= 0 && patch >= 0;
}

bool Version::isCompatible(const Version& _other) const
{
  if (!isValid() || !_other.isValid()) {
    qWarning() << "Invalid versions cannot be compared.";
    return false;
  }

  return major == _other.major && minor == _other.minor;
}

QString Version::toString() const
{
  QString res = "v%1.%2.%3";
  return res.arg(major).arg(minor).arg(patch);
}

bool Version::fromString(QString _str)
{
  // Remove the prefix.
  if (_str.startsWith('v', Qt::CaseInsensitive)) {
    _str.remove(0, 1);
  }

  // Split into three parts.
  const auto parts = _str.split('.');
  if (parts.size() != 3) {
    qWarning().noquote().nospace() << "Invalid version format: " << _str
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
      << "Invalid version number: " << _str
      << ". Each component must be an integer in the form major.minor.patch (e.g., 1.2.3).";
    return false;
  }

  return true;
}

bool Version::load(const QString& _path)
{
  const auto node = yaml::load(_path.toStdString());
  if (!node) {
    qWarning() << node.error().c_str();
    return false;
  }

  if (!yaml::load(kMajorKey, *node, major)) {
    return false;
  }
  if (!yaml::load(kMinorKey, *node, minor)) {
    return false;
  }
  if (!yaml::load(kPatchKey, *node, patch)) {
    return false;
  }

  return true;
}

bool Version::save(const QString& _path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kMajorKey] = major;
  node[kMinorKey] = minor;
  node[kPatchKey] = patch;

  if (const auto result = yaml::save(_path.toStdString(), node); !result) {
    qWarning() << result.error().c_str();
    return false;
  }

  return true;
}
}  // namespace tobas::gui::cmn
