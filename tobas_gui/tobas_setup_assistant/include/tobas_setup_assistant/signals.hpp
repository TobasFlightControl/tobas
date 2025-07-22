#pragma once

#include <QObject>

#include <tobas_drone_core/propulsion_system/type.hpp>

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
  void propulsionTypeChanged(const tobas::PropulsionSystem& type);
};
}  // namespace sa
}  // namespace gui
