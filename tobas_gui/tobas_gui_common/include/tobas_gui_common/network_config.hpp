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
struct NetworkConfig
{
  QString interface;
};

std::expected<NetworkConfig, QString> loadNetworkConfig(const QString& _path);
std::expected<void, QString> saveNetworkConfig(const QString& _path, const NetworkConfig& _config);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
