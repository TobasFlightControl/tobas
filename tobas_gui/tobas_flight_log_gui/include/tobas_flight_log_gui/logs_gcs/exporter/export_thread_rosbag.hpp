// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <quazip/quazip.h>

#include "./export_thread.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class ExportThreadRosbag : public ExportThread
{
  Q_OBJECT

public:
  explicit ExportThreadRosbag(const QString& log_name, const QString& save_path);

  void run() override;

private:
  const QString log_name_;
  const QString save_path_;

  bool addZipEntry(QuaZip& zip, const QString& abs_path, const QString& zip_path);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
