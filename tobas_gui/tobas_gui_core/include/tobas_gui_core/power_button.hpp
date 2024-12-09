#pragma once

#include <QPushButton>

namespace gui
{
namespace core
{
class PowerButton : public QPushButton
{
  Q_OBJECT

  using self = PowerButton;
  using super = QPushButton;

public:
  explicit PowerButton(int radius);
};
}  // namespace core
}  // namespace gui
