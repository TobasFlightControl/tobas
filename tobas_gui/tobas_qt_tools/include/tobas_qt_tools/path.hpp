// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace qt
{
/* Get the absolute path to the resource directory of this package. */
QString getResourcePath();

/* e.g. hoge/fuga/piyo.ext -> piyo */
QString getBaseName(const QString& path);

/* e.g. ~/hoge/fuga -> /home/user/hoge/fuga */
QString expandUser(const QString& path);

/**
 * @brief Write text to a timestamped file.
 *
 * @param content Text to write.
 * @param dir_path Directory where the file is saved.
 * @param prefix Filename prefix.
 * @param suffix Filename suffix.
 * @param ext File extension.
 *
 * @return Full path of the created file.
 */
std::expected<QString, QString> writeTimestampedFile(
  const QString& content,
  const QString& dir_path,
  const QString& prefix = "",
  const QString& suffix = "",
  const QString& ext = "txt");
}  // namespace qt
}  // namespace tobas
