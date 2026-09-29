// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/parse_xacro.hpp"

#include <QProcess>

namespace tobas
{
namespace gui
{
namespace sa
{
std::expected<QString, QString> parseXacroFromPath(const QString& xacro_path)
{
  // Pass the path as a separate argument so shell metacharacters remain part of the file name.
  // Keep stderr separate from stdout to avoid including diagnostics in the generated URDF.
  QProcess process;
  process.start("xacro", { "--", xacro_path });
  if (!process.waitForStarted(-1)) {
    return std::unexpected("Failed to start XACRO: " + process.errorString());
  }

  const auto finished = process.waitForFinished(-1);
  if (!finished || process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    auto error = "Failed to parse XACRO '" + xacro_path + "': ";
    if (!finished || process.exitStatus() != QProcess::NormalExit) {
      error += process.errorString();
    }
    else {
      error += "Command exited with status " + QString::number(process.exitCode()) + ".";
    }
    const auto diagnostics = QString::fromUtf8(process.readAllStandardError());
    if (!diagnostics.isEmpty()) {
      error += "\n" + diagnostics;
    }
    return std::unexpected(error);
  }

  return QString::fromUtf8(process.readAllStandardOutput());
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
