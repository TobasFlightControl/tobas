#pragma once

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace hw
{
class BaseHardwareSetupWidget : public qt::ScrollArea
{
  Q_OBJECT

public:
  explicit BaseHardwareSetupWidget();

  virtual const char* name() const = 0;
  virtual const char* title() const = 0;

  virtual void reset() = 0;

protected:
  QVBoxLayout* rows_;

private:
  QLabel* title_;

private Q_SLOTS:
  void initialize();
};
}  // namespace hw
}  // namespace gui
