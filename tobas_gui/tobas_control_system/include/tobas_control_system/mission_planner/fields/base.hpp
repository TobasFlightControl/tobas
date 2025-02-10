#pragma once

#include <QWidget>

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
