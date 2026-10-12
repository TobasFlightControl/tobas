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
public:
  int major = -1;
  int minor = -1;
  int patch = -1;

  explicit Version();
  explicit Version(int _major, int _minor, int _patch);

  static Version Current();

  bool isValid() const;
  bool isCompatible(const Version& _other) const;

  QString toString() const;
  bool fromString(QString _str);

  bool load(const QString& _path);
  bool save(const QString& _path) const;

  /**
   * Automatically define comparison operators
   * that compare lexicographically in member declaration order (>= C++20).
   */
  auto operator<=>(const Version&) const = default;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
