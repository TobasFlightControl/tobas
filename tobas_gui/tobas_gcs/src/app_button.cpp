#include "tobas_gcs/app_button.hpp"

namespace gui
{
namespace gcs
{
AppButton::AppButton(const QString& text, const QString& icon_path)
{
  setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  setText(text);
  setIcon(QIcon(icon_path));
  setIconSize(QSize(kIconSize, kIconSize));
  setMaximumWidth(kButtonMaxWidth);
  setCheckable(true);
}
}  // namespace gcs
}  // namespace gui
