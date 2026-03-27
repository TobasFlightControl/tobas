#include "tobas_gcs/app_button.hpp"

namespace tobas
{
namespace gui
{
namespace gcs
{
AppButton::AppButton(const QString& text, const QString& icon_path)
{
  setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  setText(text);
  setIcon(QIcon(icon_path));
  setIconSize(QSize(kButtonWidth, kIconHeight));  // レイアウトを整えるためにアイコンの横幅をなるべく大きくとる
  setFixedWidth(kButtonWidth);
  setCheckable(true);
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
