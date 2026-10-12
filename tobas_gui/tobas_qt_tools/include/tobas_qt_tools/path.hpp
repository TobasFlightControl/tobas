// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace qt
{
/** Get the absolute path to the resource directory of this package. */
QString getResourcePath();

/** e.g. hoge/fuga/piyo.ext -> piyo */
QString getBaseName(const QString& _path);

/** e.g. ~/hoge/fuga -> /home/user/hoge/fuga */
QString expandUser(const QString& _path);

/**
 * Write text to a timestamped file.
 *
 * @param _content Text to write.
 * @param _dir_path Directory where the file is saved.
 * @param _prefix Filename prefix.
 * @param _suffix Filename suffix.
 * @param _ext File extension.
 * @return Full path of the created file.
 */
std::expected<QString, QString> writeTimestampedFile(
  const QString& _content,
  const QString& _dir_path,
  const QString& _prefix = "",
  const QString& _suffix = "",
  const QString& _ext = "txt");
}  // namespace qt
}  // namespace tobas
