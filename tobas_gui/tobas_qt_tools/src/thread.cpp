// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/thread.hpp"

namespace tobas
{
namespace qt
{
void startThreadAndWait(std::function<void()> func)
{
  const auto thread = QThread::create(std::move(func));

  // Prepare an event loop to catch results from another thread.
  QEventLoop loop;
  const auto conn = QObject::connect(thread, &QThread::finished, &loop, &QEventLoop::quit);

  // Wait for the thread to finish while running the event loop.
  thread->start();
  loop.exec();
  thread->wait();

  QObject::disconnect(conn);
}
}  // namespace qt
}  // namespace tobas
