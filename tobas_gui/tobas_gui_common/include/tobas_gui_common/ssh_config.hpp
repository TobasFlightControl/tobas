// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace gui
{
namespace cmn
{
struct SshConfig
{
  QString host;
  QString user;
};

std::expected<SshConfig, QString> loadSshConfig(const QString& _path);
std::expected<void, QString> saveSshConfig(const QString& _path, const SshConfig& _config);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
