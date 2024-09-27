#pragma once

#include <QWidget>

namespace gui
{
namespace control_system
{
namespace field
{
class BaseField : public QWidget
{
  Q_OBJECT

Q_SIGNALS:
  void updated();

public:
  virtual const char* label() const = 0;
};
}  // namespace field
}  // namespace control_system
}  // namespace gui
