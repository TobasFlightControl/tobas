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

std::expected<NetworkConfig, QString> loadNetworkConfig(const QString& path);
std::expected<void, QString> saveNetworkConfig(const QString& path, const NetworkConfig& config);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
