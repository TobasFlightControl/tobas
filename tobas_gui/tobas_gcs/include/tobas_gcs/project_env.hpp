// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

namespace tobas
{
namespace gui
{
namespace gcs
{
struct ProjectEnv
{
  QString config_pkg;
  QString nic;
  QString drone;
  QString id;
};

ProjectEnv parseProjectEnv(const QString& _text);
QString exportProjectEnv(const ProjectEnv& _env);
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
