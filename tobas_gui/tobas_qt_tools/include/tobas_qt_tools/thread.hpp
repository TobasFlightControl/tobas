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

  // メタ型が定義されていることを保証
  static_assert(
    QMetaTypeId2<D>::Defined,
    "Signal argument type is not a Qt metatype. "
    "Add Q_DECLARE_METATYPE(Type) in global namespace and include the header here.");

  // 独自型はシグナルスロット接続前に登録が必要なことが多い
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

/* GUIを止めずにスレッドの終了を待機する． */
template <typename Thread, typename... SigArgs>
auto startThreadAndWait(Thread& thread, void (Thread::*signal)(SigArgs...)) -> std::tuple<std::decay_t<SigArgs>...>
{
  static_assert(std::is_base_of_v<QThread, Thread>, "Thread must derive from QThread.");

  // QueuedConnection で必要になる型を事前登録
  detail::ensureMetaTypesRegistered<SigArgs...>();

  // イベントループを用意
  QEventLoop loop;

  // 別スレッドの結果をキャッチ
  std::tuple<std::decay_t<SigArgs>...> result;
  const auto result_conn = QObject::connect(
    &thread,
    signal,
    &loop,
    [&result](SigArgs... args) { result = std::make_tuple(std::forward<SigArgs>(args)...); },
    Qt::QueuedConnection);

  // 別スレッドの終了に合わせてイベントループを終了
  const auto finished_conn =
    QObject::connect(&thread, &QThread::finished, &loop, &QEventLoop::quit, Qt::QueuedConnection);

  // イベントループを回しながらスレッドが終了するまで待機
  thread.start();
  loop.exec(QEventLoop::AllEvents);
  thread.wait();

  QObject::disconnect(result_conn);
  QObject::disconnect(finished_conn);

  return result;
}

/* 関数を実行するスレッドを作成し，GUIを止めずに終了まで待機する． */
void startThreadAndWait(std::function<void()> func);

/* GUIを止めずに指定した時間だけスリープする． */
template <typename RepType, typename DurType>
void spinFor(std::chrono::duration<RepType, DurType> time)
{
  const auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
  startThreadAndWait([msec]() { QThread::msleep(msec); });
}
}  // namespace qt
}  // namespace tobas
