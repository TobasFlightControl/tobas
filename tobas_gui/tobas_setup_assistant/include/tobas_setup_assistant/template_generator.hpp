// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>
#include <inja/inja.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class TemplateGenerator
{
public:
  explicit TemplateGenerator(const QString& tpl_dir);

  void generate(const inja::json& data, const QString& rel_path, const QString& out_dir, bool overwrite = true);

private:
  const QString tpl_dir_;

  inja::Environment env_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
