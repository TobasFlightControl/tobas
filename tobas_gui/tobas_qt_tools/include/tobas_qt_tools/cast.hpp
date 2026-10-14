// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDebug>
#include <QObject>
#include <stdexcept>

#include <tobas_std_tools/typeinfo.hpp>

namespace tobas::qt
{
template <typename T>
T* qPointerCast(QObject* _obj)
{
  if (!_obj) {
    throw std::invalid_argument("Object must not be nullptr.");
  }

  T* casted = qobject_cast<T*>(_obj);
  if (!casted) {
    qCritical() << "Failed to cast " << _obj->objectName() << " to " << st::getClassName<T>();
    throw std::bad_cast();
  }

  return casted;
}

template <typename T>
const T* qConstPointerCast(const QObject* _obj)
{
  if (!_obj) {
    throw std::invalid_argument("Object must not be nullptr.");
  }

  const T* casted = qobject_cast<const T*>(_obj);
  if (!casted) {
    qCritical() << "Failed to cast " << _obj->objectName() << " to " << st::getClassName<T>();
    throw std::bad_cast();
  }

  return casted;
}
}  // namespace tobas::qt
