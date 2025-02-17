#pragma once

#include <QPushButton>

namespace gui
{
namespace core
{
class ShutdownButton : public QPushButton
{
  Q_OBJECT

  using self = ShutdownButton;
  using super = QPushButton;

public:
  explicit ShutdownButton(int radius);
};
}  // namespace core
}  // namespace gui
