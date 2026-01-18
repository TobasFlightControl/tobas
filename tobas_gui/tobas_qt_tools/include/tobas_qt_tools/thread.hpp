#pragma once

#include <tuple>

#include <QEventLoop>
#include <QThread>

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

/* GUIを止めずにQThreadの終了を待機する． */
template <typename Thread, typename... SigArgs>
auto startThreadAndWait(Thread& thread, void (Thread::*signal)(SigArgs...)) -> std::tuple<std::decay_t<SigArgs>...>
{
  static_assert(std::is_base_of_v<QThread, Thread>, "Thread must derive from QThread.");

  // QueuedConnection で必要になる型を事前登録
  detail::ensureMetaTypesRegistered<SigArgs...>();

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
