// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/path.hpp"

#include <QDateTime>
#include <QDir>
#include <QSaveFile>
#include <QTextStream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tobas::qt
{
QString getResourcePath()
{
  const auto pkg_path = ament_index_cpp::get_package_share_directory("tobas_qt_tools");
  return QDir(QString::fromStdString(pkg_path)).filePath("resources");
}

QString getBaseName(const QString& _path)
{
  return _path.left(_path.lastIndexOf('.'));
}

QString expandUser(const QString& _path)
{
  if (_path == "~") {
    return QDir::homePath();
  }
  else if (_path.startsWith("~/")) {
    return QDir(QDir::homePath()).filePath(_path.mid(2));
  }
  else {
    return _path;
  }
}

std::expected<QString, QString> writeTimestampedFile(
  const QString& _content,
  const QString& _dir_path,
  const QString& _prefix,
  const QString& _suffix,
  const QString& _ext)
{
  QDir dir(_dir_path);
  if (!dir.exists()) {
    if (!dir.mkpath(".")) {
      return std::unexpected("Failed to create directory: " + _dir_path);
    }
  }

  // Generate the filename from date and time.
  auto filename = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
  if (!_prefix.isEmpty()) {
    filename = _prefix + '_' + filename;
  }
  if (!_suffix.isEmpty()) {
    filename += '_' + _suffix;
  }
  if (!_ext.isEmpty()) {
    filename += '.' + _ext;
  }

  // Write to the file.
  const auto path = dir.filePath(filename);
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return std::unexpected("Failed to open " + path);
  }

  QTextStream os(&file);
  os.setCodec("UTF-8");
  os << _content;

  if (!file.commit()) {
    return std::unexpected("Failed to save " + path);
  }

  return path;
}
}  // namespace tobas::qt
