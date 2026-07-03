// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <tuple>

#include <QEventLoop>
#include <QThread>

namespace tobas
{
namespace qt
{
namespace detail
{
template <class T>
void ensureMetaTypeRegistered()
{
  using D = std::decay_t<T>;

  // Ensure that the metatype is defined.
  static_assert(
    QMetaTypeId2<D>::Defined,
    "Signal argument type is not a Qt metatype. "
    "Add Q_DECLARE_METATYPE(Type) in global namespace and include the header here.");

  // Custom types often need to be registered before signal-slot connections.
  qRegisterMetaType<D>();
}

template <class... Ts>
void ensureMetaTypesRegistered()
{
  if constexpr (sizeof...(Ts) > 0) {
    (ensureMetaTypeRegistered<Ts>(), ...);
  }
}
}  // namespace detail

/* Wait for a thread to finish without stopping the GUI. */
template <typename Thread, typename... SigArgs>
auto startThreadAndWait(Thread& thread, void (Thread::*signal)(SigArgs...)) -> std::tuple<std::decay_t<SigArgs>...>
{
  static_assert(std::is_base_of_v<QThread, Thread>, "Thread must derive from QThread.");

  // Pre-register types required for `QueuedConnection`.
  detail::ensureMetaTypesRegistered<SigArgs...>();

  // Prepare the event loop.
  QEventLoop loop;

  // Catch results from another thread.
  std::tuple<std::decay_t<SigArgs>...> result;
  const auto result_conn = QObject::connect(
    &thread,
    signal,
    &loop,
    [&result](SigArgs... args) { result = std::make_tuple(std::forward<SigArgs>(args)...); },
    Qt::QueuedConnection);

  // End the event loop when the other thread finishes.
  const auto finished_conn =
    QObject::connect(&thread, &QThread::finished, &loop, &QEventLoop::quit, Qt::QueuedConnection);

  // Wait for the thread to finish while running the event loop.
  thread.start();
  loop.exec(QEventLoop::AllEvents);
  thread.wait();

  QObject::disconnect(result_conn);
  QObject::disconnect(finished_conn);

  return result;
}

/* Create a thread that runs a function and wait for it to finish without stopping the GUI. */
void startThreadAndWait(std::function<void()> func);

/* Sleep for the specified time without stopping the GUI. */
template <typename RepType, typename DurType>
void spinFor(std::chrono::duration<RepType, DurType> time)
{
  const auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
  startThreadAndWait([msec]() { QThread::msleep(msec); });
}
}  // namespace qt
}  // namespace tobas
