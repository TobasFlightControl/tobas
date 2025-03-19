#pragma once

#include <QObject>
#include <QDebug>
#include <stdexcept>

namespace qt
{
template <typename T>
T* qPointerCast(QObject* obj)
{
  T* casted = qobject_cast<T*>(obj);
  if (!casted)
    throw std::bad_cast();
  return casted;
}

template <typename T>
const T* qConstPointerCast(const QObject* obj)
{
  const T* casted = qobject_cast<const T*>(obj);
  if (!casted)
    throw std::bad_cast();
  return casted;
}
}  // namespace qt
