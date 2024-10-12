#pragma once

#include <tobas_qt_tools/widgets/circle_widget.hpp>

namespace gui
{
namespace control_system
{
class StatusWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kLEDSize = 20;
  static constexpr int kTextPSize = 12;

public:
  explicit StatusWidget(const QString& text);

  void setStatus(bool status);
  void reset();

private:
  qt::CircleWidget* led_;
};
}  // namespace control_system
}  // namespace gui
