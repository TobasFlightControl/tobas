#include "tobas_qt_tools/widgets/check_box.hpp"

namespace tobas
{
namespace qt
{
void CheckBox::setDisabledTextNormal()
{
  auto pal = this->palette();
  const auto normal = pal.color(QPalette::Active, QPalette::WindowText);
  pal.setColor(QPalette::Disabled, QPalette::WindowText, normal);
  pal.setColor(QPalette::Disabled, QPalette::Text, normal);
  this->setPalette(pal);
}
}  // namespace qt
}  // namespace tobas
