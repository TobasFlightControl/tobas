#pragma once

#include <QPushButton>

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
