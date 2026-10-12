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
TemplateGenerator::TemplateGenerator(const QString& _tpl_dir) : tpl_dir_(_tpl_dir)
{
}

void TemplateGenerator::generate(
  const inja::json& _data,
  const QString& _rel_path,
  const QString& _out_dir,
  bool _overwrite)
{
  const auto tpl_path = QDir(tpl_dir_).filePath(_rel_path);
  const auto out_path = QDir(_out_dir).filePath(QFileInfo(_rel_path).completeBaseName());

  if (!_overwrite && QFileInfo::exists(out_path)) {
    return;
  }

  const auto temp = env_.parse_template(tpl_path.toStdString());
  env_.write(temp, _data, out_path.toStdString());
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
