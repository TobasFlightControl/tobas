// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QThread>

namespace tobas
{
namespace gui
{
namespace log
{
class ExportThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
