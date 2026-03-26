#pragma once

#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sc
{
class BaseMagCalibWidget : public QWidget
{
  Q_OBJECT

public:
  virtual void reset() = 0;
  virtual void setNamespace(const std::string& ns) = 0;
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
