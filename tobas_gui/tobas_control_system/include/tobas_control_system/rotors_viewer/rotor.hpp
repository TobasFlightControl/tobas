#pragma once

#include <QWidget>

namespace gui
{
namespace control_system
{
class RotorWiddget : public QWidget
{
  Q_OBJECT

  using self = RotorWiddget;
  using super = QWidget;

public:
  explicit RotorWiddget();
};
}  // namespace control_system
}  // namespace gui
