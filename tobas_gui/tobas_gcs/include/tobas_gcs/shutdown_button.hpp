#pragma once

#include <QPushButton>

namespace tobas
{
namespace gui
{
namespace gcs
{
class ShutdownButton : public QPushButton
{
  Q_OBJECT

  using self = ShutdownButton;
  using super = QPushButton;

public:
  explicit ShutdownButton(int radius);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
