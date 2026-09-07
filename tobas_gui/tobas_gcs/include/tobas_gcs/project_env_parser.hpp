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
class ProjectEnvParser
{
public:
  QString config_pkg;
  QString nic;
  QString id;

  explicit ProjectEnvParser();

  bool parseFromText(const QString& text);

  QString exportText() const;
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
