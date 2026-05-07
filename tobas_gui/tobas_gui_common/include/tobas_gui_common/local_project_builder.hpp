// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include <tobas_colcon_cpp/core.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
class LocalProjectBuilder
{
public:
  explicit LocalProjectBuilder();

  bool build(const std::filesystem::path& proj_path);

  const std::string& errorMessage() const;

private:
  colcon::Colcon colcon_;
};

/* Build a local project without blocking Qt’s main thread. */
std::expected<void, QString> buildLocalProject(const std::filesystem::path& proj_path);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
