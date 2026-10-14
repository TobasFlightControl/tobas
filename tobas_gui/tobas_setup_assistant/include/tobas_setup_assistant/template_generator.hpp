// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>
#include <inja/inja.hpp>

namespace tobas::gui::sa
{
class TemplateGenerator
{
public:
  explicit TemplateGenerator(const QString& _tpl_dir);

  void generate(const inja::json& _data, const QString& _rel_path, const QString& _out_dir, bool _overwrite = true);

private:
  const QString tpl_dir_;

  inja::Environment env_;
};
}  // namespace tobas::gui::sa
