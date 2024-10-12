#pragma once

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace gui
{
namespace hardware_setup
{
class BaseHardwareSetupWidget : public qt::ScrollArea
{
public:
  explicit BaseHardwareSetupWidget();

  void initialize();

  virtual const char* name() const = 0;
  virtual const char* title() const = 0;

  virtual void onInit() = 0;

protected:
  QVBoxLayout* rows_;
};
}  // namespace hardware_setup
}  // namespace gui
