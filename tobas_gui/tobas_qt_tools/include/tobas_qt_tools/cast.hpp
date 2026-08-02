// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDebug>
#include <QObject>
#include <stdexcept>

#include <tobas_std_tools/typeinfo.hpp>

namespace tobas
{
namespace qt
{
template <typename T>
T* qPointerCast(QObject* obj)
{
  if (!obj) {
    throw std::invalid_argument("Object must not be nullptr.");
  }

  T* casted = qobject_cast<T*>(obj);
  if (!casted) {
    qCritical() << "Failed to cast " << obj->objectName() << " to " << st::getClassName<T>();
    throw std::bad_cast();
  }

  return casted;
}

template <typename T>
const T* qConstPointerCast(const QObject* obj)
{
  if (!obj) {
    throw std::invalid_argument("Object must not be nullptr.");
  }

  const T* casted = qobject_cast<const T*>(obj);
  if (!casted) {
    qCritical() << "Failed to cast " << obj->objectName() << " to " << st::getClassName<T>();
    throw std::bad_cast();
  }

  return casted;
}
}  // namespace qt
}  // namespace tobas
