#include "tobas_qt_tools/event.hpp"

#include <QCoreApplication>
#include <QEvent>
#include <QEventLoop>
#include <QTimer>

namespace qt
{
void processAllQueuedEvents()
{
  // ネイティブイベント以外の全てのQtイベントを，対象のオブジェクトに即座に配送する．
  QCoreApplication::sendPostedEvents();

  // ネイティブイベントも含め，現在キューに溜まっている全てのQtイベントを処理する．
  QEventLoop loop;
  QTimer::singleShot(0, &loop, &QEventLoop::quit);  // キューの末尾に入るため全てのイベントが処理されてから抜ける
  loop.exec(QEventLoop::AllEvents | QEventLoop::ExcludeUserInputEvents);  // ネスト中のユーザ操作は除外

  // ダメ押しの1処理 (画面更新のために必須だったりする)
  QCoreApplication::processEvents();
}
}  // namespace qt
