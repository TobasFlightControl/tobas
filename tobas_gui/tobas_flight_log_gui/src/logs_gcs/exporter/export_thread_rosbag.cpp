// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/logs_gcs/exporter/export_thread_rosbag.hpp"

#include <quazip/quazipfile.h>
#include <quazip/quazipnewinfo.h>
#include <QDir>
#include <QDirIterator>

#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/path.hpp>

namespace tobas
{
namespace gui
{
namespace log
{
ExportThreadRosbag::ExportThreadRosbag(const QString& log_name, const QString& save_path)
  : log_name_(log_name), save_path_(save_path)
{
}

void ExportThreadRosbag::run()
{
  const auto log_path = qt::expandUser(kRosbagDirHome) + '/' + log_name_;

  QuaZip zip(save_path_);

  if (!zip.open(QuaZip::mdCreate)) {
    Q_EMIT finished(false, "Failed to create zip file: " + save_path_);
    QFile::remove(save_path_);
    return;
  }

  QDir source_dir(log_path);

  // Add the top directory itself.
  if (!addZipEntry(zip, log_path, log_name_ + '/')) {
    zip.close();
    QFile::remove(save_path_);
    return;
  }

  // Add all directories and files under the top directory.
  QDirIterator it(
    log_path, QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System, QDirIterator::Subdirectories);

  while (it.hasNext()) {
    const auto abs_path = it.next();
    const QFileInfo info(abs_path);

    const auto relative_path = source_dir.relativeFilePath(info.absoluteFilePath());
    auto zip_path = log_name_ + '/' + relative_path;
    zip_path.replace("\\", "/");

    if (info.isDir()) {
      zip_path += '/';
    }

    if (!addZipEntry(zip, abs_path, zip_path)) {
      zip.close();
      QFile::remove(save_path_);
      return;
    }
  }

  zip.close();

  if (zip.getZipError() != 0) {
    Q_EMIT finished(false, "Failed to finalize zip file: " + save_path_);
    QFile::remove(save_path_);
    return;
  }

  Q_EMIT finished(true, "");
}

bool ExportThreadRosbag::addZipEntry(QuaZip& zip, const QString& abs_path, const QString& zip_path)
{
  const QFileInfo info(abs_path);

  QuaZipFile zip_file(&zip);

  QuaZipNewInfo zip_info(zip_path, abs_path);
  zip_info.setPermissions(info.permissions());

  // Create the zip file.
  if (!zip_file.open(QIODevice::WriteOnly, zip_info)) {
    Q_EMIT finished(false, "Failed to add entry to zip: " + zip_path);
    return false;
  }

  // If the source is a file, write its contents.
  if (info.isFile()) {
    QFile input_file(abs_path);

    if (!input_file.open(QIODevice::ReadOnly)) {
      Q_EMIT finished(false, "Failed to open file: " + abs_path);
      return false;
    }

    QByteArray buffer;
    buffer.resize(64 * (1 << 10));  // Read 64 KiB at a time.

    while (true) {
      const auto read_size = input_file.read(buffer.data(), buffer.size());

      if (read_size < 0) {
        Q_EMIT finished(false, "Failed to read file: " + abs_path);
        return false;
      }

      if (read_size == 0) {
        break;
      }

      const auto write_size = zip_file.write(buffer.constData(), read_size);

      if (write_size != read_size) {
        Q_EMIT finished(false, "Failed to write file to zip: " + zip_path);
        return false;
      }
    }
  }

  zip_file.close();

  if (zip_file.getZipError() != 0) {
    Q_EMIT finished(false, "Zip error while closing entry: " + zip_path);
    return false;
  }

  return true;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
