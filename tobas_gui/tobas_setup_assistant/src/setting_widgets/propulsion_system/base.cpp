#include "tobas_setup_assistant/setting_tabs/propulsion_system/base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
BasePropulsionSystemWidget::BasePropulsionSystemWidget()
{
  enableWheelEvent(false);
  setTabSize(kTabWidth, kTabHeight);
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
