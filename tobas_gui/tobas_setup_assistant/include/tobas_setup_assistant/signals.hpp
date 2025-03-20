#pragma once

#include <QObject>

namespace gui
{
namespace sa
{
/**
 * @brief 共通のシグナル．
 * 階層をまたぐウィジェット同士の結合を疎にするためにインターフェースを別クラスにする．
 * ROSメッセージと同じ思想．
 */
class Signals : public QObject
{
  Q_OBJECT

Q_SIGNALS:
  void rotorLinkAdded(const QString& link_name);
  void rotorLinkRemoved(const QString& link_name);

  void isTiltRotorStateChanged(const QString& link_name, bool is_tilt);
  void tiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);
};
}  // namespace sa
}  // namespace gui
