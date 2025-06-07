#pragma once

#include <QPushButton>

namespace gui
{
namespace gcs
{
class RestartButton : public QPushButton
{
  Q_OBJECT

  using self = RestartButton;
  using super = QPushButton;

public:
  explicit RestartButton(int radius);
};
}  // namespace gcs
}  // namespace gui
