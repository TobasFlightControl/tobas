#include "tobas_gui_core/app_button.hpp"

namespace gui
{
namespace core
{
AppButton::AppButton(const QString& text, const QString& icon_path)
{
  setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  setText(text);
  setIcon(QIcon(icon_path));
  setIconSize(QSize(kIconSize, kIconSize));
  setFixedWidth(kButtonWidth);
  setCheckable(true);
}
}  // namespace core
}  // namespace gui
