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
class SshConfig
{
public:
  QString host;
  QString user;

  bool load(const QString& path);
  bool save(const QString& path) const;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
