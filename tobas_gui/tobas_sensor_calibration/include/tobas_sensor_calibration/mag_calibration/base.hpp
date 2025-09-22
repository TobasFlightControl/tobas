#pragma once

#include <QWidget>

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
