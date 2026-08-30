// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/template_generator.hpp"

#include <QDir>
#include <QFileInfo>

namespace tobas
{
namespace gui
{
namespace sa
{
TemplateGenerator::TemplateGenerator(const QString& tpl_dir) : tpl_dir_(tpl_dir)
{
}

void TemplateGenerator::generate(const inja::json& data, const QString& rel_path, const QString& out_dir, bool overwrite)
{
  const auto tpl_path = QDir(tpl_dir_).filePath(rel_path);
  const auto out_path = QDir(out_dir).filePath(QFileInfo(rel_path).completeBaseName());

  if (!overwrite && QFileInfo::exists(out_path)) {
    return;
  }

  const auto temp = env_.parse_template(tpl_path.toStdString());
  env_.write(temp, data, out_path.toStdString());
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
