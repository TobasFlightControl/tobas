// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <quazip/quazip.h>

#include "./export_thread.hpp"

namespace tobas::gui::log
{
class ExportThreadRosbag : public ExportThread
{
  Q_OBJECT

public:
  explicit ExportThreadRosbag(const QString& _log_name, const QString& _save_path);

  void run() override;

private:
  const QString log_name_;
  const QString save_path_;

  bool addZipEntry(QuaZip& _zip, const QString& _abs_path, const QString& _zip_path);
};
}  // namespace tobas::gui::log
