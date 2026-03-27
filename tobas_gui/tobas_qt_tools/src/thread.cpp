#include "tobas_qt_tools/thread.hpp"

namespace tobas
{
namespace qt
{
void startThreadAndWait(std::function<void()> func)
{
  const auto thread = QThread::create(std::move(func));

  // 別スレッドの結果をキャッチするためのイベントループを用意
  QEventLoop loop;
  const auto conn = QObject::connect(thread, &QThread::finished, &loop, &QEventLoop::quit);

  // イベントループを回しながらスレッドが終了するまで待機
  thread->start();
  loop.exec();
  thread->wait();

  QObject::disconnect(conn);
}
}  // namespace qt
}  // namespace tobas
