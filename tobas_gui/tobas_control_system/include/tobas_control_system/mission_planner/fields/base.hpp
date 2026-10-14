// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas::gui::ctrl::field
{
class BaseFieldWidget : public QWidget
{
  Q_OBJECT

Q_SIGNALS:
  void updated();

public:
  virtual const char* label() const = 0;
};

template <typename T>
class FieldWidget : public BaseFieldWidget
{
public:
  virtual T getValue() const = 0;
  virtual void setValue(T _value) = 0;
};
}  // namespace tobas::gui::ctrl::field
