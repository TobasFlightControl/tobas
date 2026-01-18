#pragma once

#include <tuple>

#include <QEventLoop>
#include <QThread>

namespace qt
{
/* GUIを止めずにQThreadの終了を待機する． */
template <typename Thread, typename... SigArgs>
auto startThreadAndWait(Thread& thread, void (Thread::*signal)(SigArgs...)) -> std::tuple<std::decay_t<SigArgs>...>
{
  static_assert(std::is_base_of_v<QThread, Thread>, "Thread must derive from QThread.");

  // 別スレッドの結果をキャッチするためのイベントループを用意
  QEventLoop loop;
  std::tuple<std::decay_t<SigArgs>...> result;
  const auto conn = QObject::connect(
    &thread,
    signal,
    &loop,
    [&](SigArgs... args)
    {
      result = std::make_tuple(std::forward<SigArgs>(args)...);
      loop.quit();
    });

  // イベントループを回しながらスレッドが終了するまで待機
  thread.start();
  loop.exec();
  thread.wait();

  QObject::disconnect(conn);
  return result;
}
}  // namespace qt
