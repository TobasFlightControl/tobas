// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

namespace tobas
{
namespace gui
{
namespace cmn
{
class Version
{
  static constexpr char kMajorKey[] = "major";
  static constexpr char kMinorKey[] = "minor";
  static constexpr char kPatchKey[] = "patch";

public:
  int major = -1;
  int minor = -1;
  int patch = -1;

  explicit Version();
  explicit Version(int _major, int _minor, int _patch);

  static Version Current();

  bool isValid() const;
  bool isCompatible(const Version& other) const;

  QString toString() const;
  bool fromString(QString str);

  bool load(const QString& path);
  bool save(const QString& path) const;

  /* Automatically define comparison operators that compare lexicographically in member declaration order (>= C++20). */
  auto operator<=>(const Version&) const = default;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
