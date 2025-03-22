#pragma once

#include <stdexcept>
#include <QObject>
#include <QDebug>

#include <tobas_std_tools/typeinfo.hpp>

namespace qt
{
template <typename T>
T* qPointerCast(QObject* obj)
{
  T* casted = qobject_cast<T*>(obj);
  if (!casted)
  {
    qCritical() << "Failed to cast " << obj->objectName() << " to " << tobas_std::getClassName<T>();
    throw std::bad_cast();
  }
  return casted;
}

template <typename T>
const T* qConstPointerCast(const QObject* obj)
{
  const T* casted = qobject_cast<const T*>(obj);
  if (!casted)
  {
    qCritical() << "Failed to cast " << obj->objectName() << " to " << tobas_std::getClassName<T>();
    throw std::bad_cast();
  }
  return casted;
}
}  // namespace qt
