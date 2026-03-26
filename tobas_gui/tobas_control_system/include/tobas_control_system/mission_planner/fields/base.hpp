#pragma once

#include <QWidget>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
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
  virtual void setValue(T value) = 0;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
