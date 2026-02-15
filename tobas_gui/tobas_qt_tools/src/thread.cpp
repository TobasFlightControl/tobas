#include "tobas_qt_tools/thread.hpp"

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

void spinFor(int time_ms)
{
  startThreadAndWait([time_ms]() { QThread::msleep(time_ms); });
}
}  // namespace qt
