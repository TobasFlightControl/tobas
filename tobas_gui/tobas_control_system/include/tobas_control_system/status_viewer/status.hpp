#pragma once

#include <tobas_qt_tools/widgets/circle_widget.hpp>

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
