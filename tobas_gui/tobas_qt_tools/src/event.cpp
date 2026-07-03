// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/event.hpp"

#include <QCoreApplication>
#include <QEvent>
#include <QEventLoop>
#include <QTimer>

namespace tobas
{
namespace qt
{
void processAllQueuedEvents()
{
  // Immediately deliver all Qt events except native events to the target object.
  QCoreApplication::sendPostedEvents();

  // Process all currently queued Qt events, including native events.
  QEventLoop loop;
  QTimer::singleShot(0, &loop, &QEventLoop::quit);
  loop.exec(QEventLoop::AllEvents | QEventLoop::ExcludeUserInputEvents);  // Exclude nested user operations.

  // One final processing pass, sometimes required for screen updates.
  QCoreApplication::processEvents();
}
}  // namespace qt
}  // namespace tobas
