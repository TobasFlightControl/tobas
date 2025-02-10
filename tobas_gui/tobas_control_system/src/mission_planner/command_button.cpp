#include "tobas_control_system/mission_planner/command_button.hpp"

namespace gui
{
namespace gcs
{
CommandButton::CommandButton(const QString& text) : super(text)
{
  setMaximumWidth(kMaxWidth);
  setFixedHeight(kFixedHeight);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}
}  // namespace gcs
}  // namespace gui
