// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/path.hpp"

#include <filesystem>

#include <QDateTime>
#include <QDir>
#include <QSaveFile>
#include <QTextStream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace qt
{
QString getResourcePath()
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_qt_tools"));
  const auto resource_path = pkg_path / "resources";
  return QString::fromStdString(resource_path);
}

QString getBaseName(const QString& path)
{
  return path.left(path.lastIndexOf('.'));
}

QString expandUser(const QString& path)
{
  if (path == "~") {
    return QDir::homePath();
  }
  else if (path.startsWith("~/")) {
    return QDir(QDir::homePath()).filePath(path.mid(2));
  }
  else {
    return path;
  }
}

std::expected<QString, QString> writeTimestampedFile(
  const QString& content,
  const QString& dir_path,
  const QString& prefix,
  const QString& suffix,
  const QString& ext)
{
  QDir dir(dir_path);
  if (!dir.exists()) {
    if (!dir.mkpath(".")) {
      return std::unexpected("Failed to create directory: " + dir_path);
    }
  }

  // Generate the filename from date and time.
  auto filename = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
  if (!prefix.isEmpty()) {
    filename = prefix + '_' + filename;
  }
  if (!suffix.isEmpty()) {
    filename += '_' + suffix;
  }
  if (!ext.isEmpty()) {
    filename += '.' + ext;
  }

  // Write to the file.
  const auto path = dir.filePath(filename);
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return std::unexpected("Failed to open " + path);
  }

  QTextStream os(&file);
  os.setCodec("UTF-8");
  os << content;

  if (!file.commit()) {
    return std::unexpected("Failed to save " + path);
  }

  return path;
}
}  // namespace qt
}  // namespace tobas
