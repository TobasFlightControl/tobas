#include "tobas_control_system/mission_planner/command_button.hpp"

namespace gui
{
namespace control_system
{
CommandButton::CommandButton(const QString& text) : super(text)
{
  setMaximumWidth(kMaxWidth);
  setFixedHeight(kFixedHeight);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}
}  // namespace control_system
}  // namespace gui
