#pragma once

#include <QPushButton>

namespace gui
{
namespace control_system
{
class CommandButton : public QPushButton
{
  Q_OBJECT

  using self = CommandButton;
  using super = QPushButton;

  static constexpr int kMaxWidth = 100;
  static constexpr int kFixedHeight = 40;

public:
  explicit CommandButton(const QString& text);
};
}  // namespace control_system
}  // namespace gui
