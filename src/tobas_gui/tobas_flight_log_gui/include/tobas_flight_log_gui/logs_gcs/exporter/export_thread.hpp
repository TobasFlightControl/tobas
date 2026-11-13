// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QThread>

namespace tobas::gui::log
{
class ExportThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool _success, const QString& _message);
};
}  // namespace tobas::gui::log
